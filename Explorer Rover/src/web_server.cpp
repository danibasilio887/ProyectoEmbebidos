#include "web_server.h" 
#include <SPIFFS.h>
#include <Preferences.h>
#include "motor_control.h"

// Prototipos de Handlers
void handleSaveWifi();
void handleRoot();

// Función auxiliar para servir archivos desde SPIFFS con caché
void serveFile(const char* path, const char* contentType) {
  String fullPath = path;
  if (!fullPath.startsWith("/")) {
    fullPath = "/" + fullPath;
  }
  
  File file = SPIFFS.open(fullPath, "r");
  if (!file || file.isDirectory()) {
    server.send(404, "text/plain", "Archivo no encontrado: " + fullPath);
    return;
  }
  
  // Headers vitales
  server.sendHeader("Access-Control-Allow-Origin", "*");
  // Cache-Control ayuda a que la web cargue más rápido en el móvil
  server.sendHeader("Cache-Control", "max-age=86400"); 
  
  server.streamFile(file, contentType);
  file.close();
}

// Guardar credenciales y reiniciar
void handleSaveWifi() {
  stopMotor(); 
  
  String ssid = server.arg("ssid");
  String pass = server.arg("password");

  if (ssid.length() > 0) {
    Preferences preferences;
    preferences.begin("rover-wifi", false);
    preferences.putString("ssid", ssid);
    preferences.putString("pass", pass);
    preferences.end();

    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head>";
    html += "<body style='background:#121a33;color:#fff;font-family:sans-serif;text-align:center;padding:40px;'>";
    html += "<h1>Guardado</h1><p>Reiniciando sistema...</p><p>Por favor, conectate a la nueva red.</p></body></html>";
    server.send(200, "text/html", html);
    
    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Error: SSID vacio");
  }
}

// LÓGICA DE REDIRECCIÓN PRINCIPAL
void handleRoot() {
  // Escenario 1: No tenemos internet -> Forzar configuración
  if (runningInAPMode) {
    serveFile("/apWifi.html", "text/html");
  } 
  // Escenario 2: Todo bien -> Ir al Dashboard
  else {
    server.sendHeader("Location", "/home");
    server.send(302, "text/plain", "Redireccionando al Dashboard...");
  }
}

void initWebServer() {
  // RUTAS PRINCIPALES
  server.on("/home", HTTP_GET, []() { serveFile("/home.html", "text/html"); });
  server.on("/control", HTTP_GET, []() { serveFile("/control.html", "text/html"); });
  server.on("/cam", HTTP_GET, []() { serveFile("/cam.html", "text/html"); });
  server.on("/stats", HTTP_GET, []() { serveFile("/stats.html", "text/html"); }); // ¡Nueva ruta!

  // RUTAS DE SISTEMA
  server.on("/", HTTP_GET, handleRoot);
  server.on("/apWifi.html", HTTP_GET, []() { serveFile("/apWifi.html", "text/html"); });
  server.on("/setup", HTTP_GET, []() { serveFile("/apWifi.html", "text/html"); }); // Acceso directo para reconfigurar
  server.on("/save-wifi", HTTP_POST, handleSaveWifi);

  // MANEJO DE ERRORES / ARCHIVOS ESTÁTICOS / PORTAL CAUTIVO
  server.onNotFound([]() {
    // Evitar spam de iconos
    if (server.uri().endsWith("/favicon.ico")) { server.send(204); return; }
    
    // Si estamos en "Modo Atrapado" (AP sin internet), cualquier URL desconocida
    // (como cuando el celular prueba connectivitycheck.gstatic.com)
    // la redirigimos a nuestra IP para que salte el popup de "Iniciar Sesión".
    if (runningInAPMode) {
      server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/");
      server.send(302, "text/plain", "Portal Cautivo Rover");
      return;
    }

    // Si estamos navegando normal, intentamos buscar el archivo (CSS, JS, IMAGENES)
    String path = server.uri();
    String contentType = "text/plain";
    
    if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".js")) contentType = "application/javascript";
    else if (path.endsWith(".png")) contentType = "image/png";
    else if (path.endsWith(".jpg")) contentType = "image/jpeg";
    else if (path.endsWith(".svg")) contentType = "image/svg+xml";
    
    if (SPIFFS.exists(path)) {
        serveFile(path.c_str(), contentType.c_str());
    } else {
        server.send(404, "text/plain", "404: Not Found");
    }
  });

  server.begin();
  Serial.println("[HTTP] Servidor Web iniciado");
}