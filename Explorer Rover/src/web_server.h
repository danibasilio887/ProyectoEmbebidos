#pragma once
#include <WebServer.h>

extern WebServer server;

// Variables globales definidas en main.cpp
extern bool runningInAPMode; 
extern bool wifiConnectedHost;

// Inicializador
void initWebServer();