#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Declaraciones externas de app_httpd.cpp
extern void startCameraServer();
extern void setLedIntensity(int intensity);

// Estructuras de Datos (Deben coincidir con el Host)
typedef struct struct_wifi_creds {
    char ssid[32];
    char pass[64];
} struct_wifi_creds;

typedef struct struct_cam_ip {
    char ip[16]; 
} struct_cam_ip;

struct_cam_ip myIpMsg;
// Dirección de Broadcast para responder a todos (o solo al Host si supiéramos su MAC)
uint8_t hostAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// Variables de Estado
bool connectedToWiFi = false;
bool credsReceived = false;
String targetSSID = "";
String targetPASS = "";

// Función para mandar la direcció IP a la Le Potato 
void sendIPToRover(String ip) { 
    memset(myIpMsg.ip, 0, sizeof(myIpMsg.ip)); ip.toCharArray(myIpMsg.ip, sizeof(myIpMsg.ip)); 
    esp_now_send(hostAddress, (uint8_t*)&myIpMsg, sizeof(myIpMsg));
}


// Callback ESP-NOW
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Si recibimos credenciales wifi
  if (len == sizeof(struct_wifi_creds)) {
    struct_wifi_creds incomingCreds;
    memcpy(&incomingCreds, incomingData, sizeof(incomingCreds));
    
    String newSSID = String(incomingCreds.ssid);
    String newPASS = String(incomingCreds.pass);

    // Solo actuamos si la red es válida y diferente a la actual (o si no estamos conectados)
    if (newSSID.length() > 0 && (!connectedToWiFi || newSSID != targetSSID)) {
        Serial.print("[ESP-NOW] Credenciales recibidas: ");
        Serial.println(newSSID);
        
        targetSSID = newSSID;
        targetPASS = newPASS;
        credsReceived = true;
        connectedToWiFi = false; // Esto forzará la reconexión en el loop()
    }
  }
}

void connectWiFi() {
    Serial.println("[WIFI] Conectando a: " + targetSSID);
    
    // Indicar visualmente que estamos intentando conectar (Flash medio)
    setLedIntensity(20); 

    // Es vital apagar ESP-NOW antes de conectar WiFi STA para cambiar de canal libremente
    esp_now_deinit(); 
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // Desactivar ahorro de energía para mejor streaming
    WiFi.begin(targetSSID.c_str(), targetPASS.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(200); 
        // Parpadeo rápido mientras conecta
        static bool ledState = false;
        setLedIntensity(ledState ? 20 : 0);
        ledState = !ledState;

        if (millis() - start > 15000) { // Timeout 15s
            Serial.println("\n[ERR] Timeout WiFi. Volviendo a modo escucha...");
            credsReceived = false; 
            setLedIntensity(0); // Apagar LED
            
            // Volver a inicializar ESP-NOW para escuchar
            WiFi.disconnect();
            if (esp_now_init() == ESP_OK) {
                 esp_now_register_recv_cb(OnDataRecv);
            }
            return;
        }
    }
    
    // Éxito
    setLedIntensity(0); // Apagar LED (Ahorrar batería y evitar calor)
    Serial.println("\n[WIFI] Conectado! IP: " + WiFi.localIP().toString());
    connectedToWiFi = true;
    
    // Iniciar servidor Web (Puerto 81)
    startCameraServer(); 

    // Reiniciar ESP-NOW para poder enviar la IP de vuelta al Host
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, hostAddress, 6);
        peerInfo.channel = 0; // Usar canal actual del WiFi
        peerInfo.encrypt = false;
        
        if(!esp_now_is_peer_exist(hostAddress)) {
            esp_now_add_peer(&peerInfo);
        }
        esp_now_register_recv_cb(OnDataRecv); // Volver a registrar por si el Host cambia de red
    }
}

void setup() {
    Serial.begin(115200, SERIAL_8N1, 3, 1);
    Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
    Serial.setDebugOutput(true);
    Serial.println(); // Imprimir linea vacía para limpiar basura
    
    delay(2000); 
    Serial.println("--- ARRANQUE INICIADO ---");

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Optimización de buffer
    if(psramFound()){
        config.frame_size = FRAMESIZE_VGA; // 640x480
        config.jpeg_quality = 12;          // Calidad media-alta (menor número = mejor calidad)
        config.fb_count = 2;               // Doble buffer para video fluido
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_HVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error inicio camara: 0x%x", err);
        return;
    }

    // Inicialmente, escuchar en modo STA desconectado
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
    }
    
    Serial.println("[CAM] Listo. Esperando al Host...");
    
    // Inicializar control de LED (para parpadeo de búsqueda)
    // Nota: Esto se llama también dentro de startCameraServer, pero lo necesitamos antes.
    // Usamos una declaración externa de la función setupLedFlash si fuera pública, 
    // pero como está en app_httpd, usaremos la lógica en connectWiFi.
}

void loop() {
    // 1. Máquina de Estados: Conexión
    if (credsReceived && !connectedToWiFi) {
        connectWiFi();
    }

    // 2. Modo Escaneo (Si no estamos conectados)
    // El ESP32-CAM salta entre canales para encontrar en cuál está transmitiendo el Host
    if (!connectedToWiFi) {
        static unsigned long lastScan = 0;
        static int ch = 1;
        
        // Feedback visual lento (latido)
        static unsigned long lastBlink = 0;
        static int glow = 0;
        static int dir = 1;
        
        if (millis() - lastBlink > 20) {
            lastBlink = millis();
            glow += dir;
            if(glow > 10) dir = -1; // Max brillo bajo (10/255) para no molestar
            if(glow < 0) { glow = 0; dir = 1; }
            setLedIntensity(glow);
        }

        // Cambio de canal cada 200ms
        if (millis() - lastScan > 200) { 
            lastScan = millis();
            ch = (ch % 13) + 1;
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
            esp_wifi_set_promiscuous(false);
        }
    }

    // 3. Reportar IP (Si estamos conectados)
    if (connectedToWiFi) {
        static unsigned long lastPing = 0;
        if (millis() - lastPing > 1000) { // Cada segundo
            lastPing = millis();
            if (WiFi.status() == WL_CONNECTED) {
                strcpy(myIpMsg.ip, WiFi.localIP().toString().c_str());
                esp_now_send(hostAddress, (uint8_t *) &myIpMsg, sizeof(myIpMsg));
            } else {
                // Se cayó el WiFi
                Serial.println("[WIFI] Desconectado inesperadamente");
                connectedToWiFi = false; 
                credsReceived = false;
                setLedIntensity(0);
                
                WiFi.disconnect();
                if (esp_now_init() == ESP_OK) {
                     esp_now_register_recv_cb(OnDataRecv);
                }
            }
        }
    }
}