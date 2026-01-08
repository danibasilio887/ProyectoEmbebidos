#include "autopilot.h"
#include <Arduino.h>
#include "motor_control.h" // Para llamar a stopMotor(), etc.

bool autoPilotEnabled = false;

// ============= CONSTANTES DE AJUSTE =============

// Distancias (en cm)
const int MIN_DISTANCE_FRONT = 30; // Obstáculo frontal
const int MIN_DISTANCE_SIDE = 20;  // Obstáculo lateral

// Giros (para chasis 6x6)
// (Valores 0-100)
const int YAW_SHARP = 80;   // Giro cerrado (esquivar lateral)
const int YAW_SPIN = 90;    // Giro en el sitio (evasión frontal)

// Seguridad (en grados)
const float MAX_PITCH_ANGLE = 50.0; // Inclinación max (frontal)
const float MAX_ROLL_ANGLE = 35.0;  // Inclinación max (lateral)

// Tiempos de Maniobra (en ms)
const int REVERSE_TIME = 1000; // 1 segundo de reversa
const int SPIN_TIME = 1500;    // 1.5 segundos de giro

// Máquina de Estados
enum AutoPilotState { IDLE, FORWARD, AVOIDING_FRONT, AVOIDING_LEFT, AVOIDING_RIGHT };
static AutoPilotState apState = IDLE;
static unsigned long stateTimer = 0;

void initAutoPilot() {
    apState = IDLE;
    autoPilotEnabled = false;
    Serial.println("[AP] Lógica de Autopilot inicializada (sensores locales + IMU).");
}

// ============= LOOP DE PILOTO AUTOMÁTICO =============
// Esta función es llamada por main.cpp cada vez que se leen nuevos datos de sensores
void runAutoPilotLogic(long distFront, long distLeft, long distRight, float roll, float pitch) {
  
  unsigned long now = millis();

  // 1. Si no está habilitado, asegurar que el estado sea IDLE
  if (!autoPilotEnabled) {
    if (apState != IDLE) {
        stopMotor(); // Detenerse si se acaba de desactivar
        apState = IDLE;
    }
    return;
  }

  // 2. ============= CONTROL DE SEGURIDAD ANTI-VUELCO =============
  // Si el rover está en peligro, se detiene.
  if (abs(roll) > MAX_ROLL_ANGLE || abs(pitch) > MAX_PITCH_ANGLE) {
    Serial.println("-----------------------------------------");
    Serial.println("[AP] ¡PELIGRO! Ángulo de inclinación excesivo.");
    Serial.printf("[AP] Roll: %.1f°, Pitch: %.1f°\n", roll, pitch);
    Serial.println("[AP] PARADA DE EMERGENCIA.");
    Serial.println("-----------------------------------------");
    
    emergencyStop();      // Detiene motores Y centra servos
    autoPilotEnabled = false; // Desactiva el AP por seguridad
    apState = IDLE;
    return; // No ejecutar el resto de la lógica
  }
  // =====================================================================

  // 3. Máquina de Estados de Decisión
  switch (apState) {
    
    case IDLE:
      Serial.println("[AP] Autopilot activado. Iniciando...");
      apState = FORWARD;
      break;

    case FORWARD:
      updateProportionalControl(60, 0, 0); // Avanzar a velocidad media
      
      // La prioridad de los sensores es: Frontal > Izquierdo > Derecho
      if (distFront < MIN_DISTANCE_FRONT) {
        Serial.println("[AP] Obstáculo frontal. Deteniendo.");
        stopMotor();
        apState = AVOIDING_FRONT;
        stateTimer = now; // Iniciar temporizador para la maniobra
      } else if (distLeft < MIN_DISTANCE_SIDE) {
        Serial.println("[AP] Obstáculo izq. Corrigiendo...");
        apState = AVOIDING_LEFT;
      } else if (distRight < MIN_DISTANCE_SIDE) {
        Serial.println("[AP] Obstáculo der. Corrigiendo...");
        apState = AVOIDING_RIGHT;
      }
      break;
      
    case AVOIDING_LEFT: // Muy cerca a la izquierda
      // Giro más agresivo
      // (Menos velocidad, mucho más giro)
      Serial.println("[AP] Evasión izquierda (Giro cerrado)");
      updateProportionalControl(40, YAW_SHARP, 0); // Avanzar girando FUERTE a la derecha
      
      if (distLeft > MIN_DISTANCE_SIDE * 1.2) apState = FORWARD; // Volver si hay espacio
      if (distFront < MIN_DISTANCE_FRONT) apState = AVOIDING_FRONT; // Obstáculo frontal tiene prioridad
      break;
      
    case AVOIDING_RIGHT: // Muy cerca a la derecha
      // Giro más agresivo
      Serial.println("[AP] Evasión derecha (Giro cerrado)");
      updateProportionalControl(40, -YAW_SHARP, 0); // Avanzar girando FUERTE a la izquierda
      
      if (distRight > MIN_DISTANCE_SIDE * 1.2) apState = FORWARD;
      if (distFront < MIN_DISTANCE_FRONT) apState = AVOIDING_FRONT; // Obstáculo frontal tiene prioridad
      break;
      
    case AVOIDING_FRONT:
      // Maniobra de evasión: 1. Retroceder, 2. Girar, 3. Reanudar
      
      if (now - stateTimer < REVERSE_TIME) {
        // 1. Retroceder
        updateProportionalControl(-50, 0, 0); // Reversa
      } 
      // Maniobra de giro
      else if (now - stateTimer < REVERSE_TIME + SPIN_TIME) {
        // 2. Girar (1.5 segundos)
        Serial.println("[AP] Evasión: Girando en el sitio...");
        
        // Decidir dirección de giro basado en los sensores laterales
        if (distLeft > distRight) {
          updateProportionalControl(0, -YAW_SPIN, 0); // Rotar izquierda (spin)
        } else {
          updateProportionalControl(0, YAW_SPIN, 0); // Rotar derecha (spin)
        }
      } 
      else {
        // 3. Reanudar
        Serial.println("[AP] Evasión completada. Reanudando.");
        apState = FORWARD;
      }
      break;
  }
}