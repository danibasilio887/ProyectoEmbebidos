#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h> 
#include <Preferences.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ThingSpeak.h>

#include "web_server.h" 
#include "motor_control.h"
#include "autopilot.h"
#include "sensors.h"

const char* ROVER_AP_SSID = "Explorer Rover";
const char* ROVER_AP_PASS = "12345678"; 
const int   ROVER_AP_CHAN = 1;

WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

bool runningInAPMode = false;     
bool wifiConnectedHost = false;   
String currentIp = "0.0.0.0";

// ============= CONFIGURACIÓN THINGSPEAK =============
unsigned long lastThingSpeakTime = 0;
const unsigned long THINGSPEAK_INTERVAL = 20000; // Enviar cada 20 seg (Seguro para cuenta gratis)

// DATOS DE THINGSPEAK
unsigned long myChannelNumber = 3221654;      // Channel ID
const char * myWriteAPIKey = "RVWSL3F70IZLZDX1"; // Write API Key

WiFiClient  tsClient; // Cliente para ThingSpeak

// Estructuras de datos para ESP-NOW
typedef struct struct_wifi_creds {
    char ssid[32];
    char pass[64];
} struct_wifi_creds;
struct_wifi_creds credsToSend;

typedef struct struct_cam_ip {
    char ip[16]; 
} struct_cam_ip;
struct_cam_ip camIPMsg;

String g_cam_ip = "desconocida"; 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 200; 

// Variables de Joystick
float joyL_x = 0, joyL_y = 0;
float joyR_x = 0, joyR_y = 0;

// Callback al recibir datos por ESP-NOW (IP de la cámara)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(camIPMsg)) {
    memcpy(&camIPMsg, incomingData, sizeof(camIPMsg));
    g_cam_ip = String(camIPMsg.ip);
    // Serial.print("Cámara IP recibida: "); Serial.println(g_cam_ip);
  }
}

// Enviar credenciales a la cámara (y al futuro Le Potato si se integra igual)
void broadcastCredentialsToCamera() {
  memset(&credsToSend, 0, sizeof(credsToSend)); 
  if (wifiConnectedHost) {
     String s = WiFi.SSID();
     String p = WiFi.psk();
     s.toCharArray(credsToSend.ssid, 32);
     p.toCharArray(credsToSend.pass, 64);
  } else {
     strcpy(credsToSend.ssid, ROVER_AP_SSID);
     strcpy(credsToSend.pass, ROVER_AP_PASS);
  }
  esp_now_send(broadcastAddress, (uint8_t *) &credsToSend, sizeof(credsToSend));
}

void processMovement() {
    if (autoPilotEnabled) return; 

    float throttle = joyR_y * 100.0; 
    float yaw      = joyR_x * 100.0;
    float roll     = joyL_x * 100.0; // Usado para modo cangrejo o inclinaciones especiales

    // Zona muerta (Deadzone) para evitar movimientos fantasma
    if (abs(throttle) < 5) throttle = 0;
    if (abs(yaw) < 5) yaw = 0;
    if (abs(roll) < 5) roll = 0;

    updateProportionalControl(throttle, yaw, roll);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        StaticJsonDocument<300> doc; 
        DeserializationError error = deserializeJson(doc, payload);
        if (error) return;

        const char* cmd = doc["cmd"];
        
        if (strcmp(cmd, "joyL") == 0) {
            joyL_x = doc["x"];
            joyL_y = doc["y"];
            processMovement();
        } 
        else if (strcmp(cmd, "joyR") == 0) {
            joyR_x = doc["x"];
            joyR_y = doc["y"];
            processMovement();
        }
        else if (strcmp(cmd, "stop") == 0) {
            joyL_x = 0; joyL_y = 0; joyR_x = 0; joyR_y = 0;
            autoPilotEnabled = false; 
            stopMotor();              
            enableMotors();           
            Serial.println("[CMD] STOP: AP Off, Manual On");
        }
        else if (strcmp(cmd, "enable") == 0) {
            enableMotors();
        }
        else if (strcmp(cmd, "ap_toggle") == 0) {
            autoPilotEnabled = !autoPilotEnabled;
            if (!autoPilotEnabled) {
                stopMotor();
                enableMotors(); 
            }
            Serial.println(autoPilotEnabled ? "[CMD] AP ON" : "[CMD] AP OFF");
        }
        else if (strcmp(cmd, "cam_tilt") == 0) {
            int angle = doc["angle"];
            setCameraTilt(angle);
        }
    }
}

// Función auxiliar para JSON
float safeVal(float val) {
  if (isnan(val) || isinf(val)) return 0.0;
  return val;
}

void setup() {
  // Desactivar detección de Brownout puede salvar reinicios si la batería cae momentáneamente al arrancar motores
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  Wire.begin(21, 22); 
  
  // Inicialización de subsistemas
  initMotors();
  sensors_init();
  initAutoPilot();
  
  if(!SPIFFS.begin(true)){ Serial.println("[ERR] SPIFFS no montado"); return; }

  // Configuración WiFi Dual (AP + STA)
  WiFi.mode(WIFI_AP_STA); 
  // Desactivar WiFi Sleep mejora la latencia de WebSockets
  WiFi.setSleep(false); 
  WiFi.softAP(ROVER_AP_SSID, ROVER_AP_PASS, ROVER_AP_CHAN);

  // Cargar credenciales guardadas
  preferences.begin("rover-wifi", true);
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("pass", "");
  preferences.end();

  // Intentar conectar a red conocida
  if (ssid != "") {
    Serial.println("Intentando conectar a: " + ssid);
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    // Esperar hasta 8 segundos
    while(millis() - start < 8000) { 
      if(WiFi.status() == WL_CONNECTED) {
        wifiConnectedHost = true;
        Serial.println("Conectado! IP: " + WiFi.localIP().toString());
        break;
      }
      delay(100);
    }
  }

  if (wifiConnectedHost) {
    currentIp = WiFi.localIP().toString();
    runningInAPMode = false; 
  } else {
    currentIp = WiFi.softAPIP().toString();
    runningInAPMode = true; 
    Serial.println("Modo AP iniciado. IP: " + currentIp);
  }

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) Serial.println("[ERR] ESP-NOW init falló");
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("[ERR] No se pudo añadir peer broadcast");
  }
  esp_now_register_recv_cb(OnDataRecv);

  initWebServer();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  
  // DNS Server para Portal Cautivo
  if(runningInAPMode) dnsServer.start(53, "*", WiFi.softAPIP());
  
  // mDNS para poder acceder como http://rover-host.local
  if(MDNS.begin("rover-host")) MDNS.addService("http", "tcp", 80);

  // Iniciar ThingSpeak
  ThingSpeak.begin(tsClient);
}

void loop() {
  unsigned long now = millis();

  // Tareas de red (Deben ejecutarse frecuentemente)
  webSocket.loop();
  server.handleClient();
  if(runningInAPMode) dnsServer.processNextRequest();
  
  sensors_update(); 

  // Lógica de Piloto Automático
  if (autoPilotEnabled) {
    runAutoPilotLogic(get_dist_front(), get_dist_left(), get_dist_right(), get_angle_roll(), get_angle_pitch());
  }

  // Sincronización con Cámara (Enviar credenciales si no tenemos IP aún)
  static unsigned long lastSync = 0;
  unsigned long syncInterval = (g_cam_ip == "desconocida") ? 1000 : 5000; 
  if (now - lastSync > syncInterval) { 
    lastSync = now;
    broadcastCredentialsToCamera();
  }

  char jsonBuffer[256]; 

if (now - lastTelemetry > TELEMETRY_INTERVAL) {
    lastTelemetry = now;
    
    snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{\"ap\":%s,\"cam_ip\":\"%s\",\"env\":{\"temp\":%.1f,\"hum\":%.0f},\"dist\":{\"f\":%ld,\"l\":%ld,\"r\":%ld},\"imu\":{\"r\":%.0f,\"p\":%.0f},\"bat\":%.2f}",
        autoPilotEnabled ? "true" : "false",
        g_cam_ip.c_str(),
        safeVal(get_temp()), safeVal(get_humidity()),
        get_dist_front(), get_dist_left(), get_dist_right(),
        safeVal(get_angle_roll()), safeVal(get_angle_pitch()),
        safeVal(get_battery_voltage())
    );
    
    webSocket.broadcastTXT(jsonBuffer);
  }

  // Enviar a ThingSpeak
  // Solo enviamos si han pasado 20s Y estamos conectados a Internet (Modo Cliente)
  if (now - lastThingSpeakTime > THINGSPEAK_INTERVAL && wifiConnectedHost) {
    
    // Asignar valores a los campos (Fields)
    ThingSpeak.setField(1, get_temp());            // Field 1: Temperatura
    ThingSpeak.setField(2, get_humidity());        // Field 2: Humedad
    
    // Enviar todo junto
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    
    if(x == 200){
      Serial.println("[CLOUD] Datos enviados a ThingSpeak correctamente.");
    } else {
      Serial.println("[CLOUD] Error enviando a ThingSpeak. Código HTTP: " + String(x));
    }
    
    lastThingSpeakTime = now;
  }
}