#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// ============= INICIALIZACIÓN =============
// Nota: Se usa el ESP32 para PWM + INx,
// y el PCA9685 solo para Servos.
void initMotors();

// ============= CONTROL DE MOVIMIENTO =============
void updateProportionalControl(float throttle, float yaw, float roll);
void stopMotor(); // Detener motores DC

// ============= CONTROL DE CÁMARA =============
void setCameraTilt(int angle); // Ángulo absoluto 0-180

// ============= FUNCIONES DE EMERGENCIA =============
void emergencyStop(); // Detener motores Y centrar servos
void enableMotors();
void centerAllServos(); // Centrar solo los servos de dirección

// ============= VARIABLES DE ESTADO (para main.cpp) =============
extern bool motorsEnabled;
extern float currentThrottle;
extern float currentYaw;
extern float currentRoll;

#endif // MOTOR_CONTROL_H