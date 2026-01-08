#include "motor_control.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); 

#define SERVO_FREQ 50 
#define US2PWM(x) ((int)(x / (1000000.0 / SERVO_FREQ) * 4096.0))
#define MIN_PULSE_US 500   
#define MAX_PULSE_US 2500  

const int pwmMin = US2PWM(MIN_PULSE_US); 
const int pwmMax = US2PWM(MAX_PULSE_US); 

#define STEERING_SERVOS 6
const int steeringServoChannels[STEERING_SERVOS] = {0, 1, 2, 3, 4, 5};
const int cameraServoChannel = 6; 
int servoOffsets[STEERING_SERVOS] = {0, 0, 0, 0, 0, 0};
#define NEUTRAL_ANGLE 90

// ============= CONFIGURACIÓN DE PINES MOTORES =============
//                M0, M1, M2  M3  M4  M5
int in1Pins[6] = {16, 18, 25, 32, 13, 27}; 
int in2Pins[6] = {17, 19, 33, 23, 14, 26};

// Canales PWM internos del ESP32 (0-15 disponibles)
const int in1PwmChannels[6] = {0, 1, 2, 3, 4, 5};
const int in2PwmChannels[6] = {6, 7, 8, 9, 10, 11};
const int pwmFrequency = 20000; 
const int pwmResolution = 8; 

// ============= VARIABLES DE ESTADO =============
bool motorsEnabled = true; 
extern bool autoPilotEnabled;
float currentThrottle = 0;
float currentYaw = 0;
float currentRoll = 0;
int lastServoAngles[16]; 
float smoothedSpeed = 0;

// Prototipos locales
void setMotor(int motorID, int dir, int speed);
void setServoAngle(int channel, int angle, int offset);
void executeNormalMovement();
void executeCrabMovement();
void executeRotation();

void initMotors() {
  //Wire.setClock(400000); 
  for (int i = 0; i < 6; i++) {
    pinMode(in1Pins[i], OUTPUT);
    ledcSetup(in1PwmChannels[i], pwmFrequency, pwmResolution);
    ledcAttachPin(in1Pins[i], in1PwmChannels[i]);
    pinMode(in2Pins[i], OUTPUT);
    ledcSetup(in2PwmChannels[i], pwmFrequency, pwmResolution);
    ledcAttachPin(in2Pins[i], in2PwmChannels[i]);
  }
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  for(int i=0; i<16; i++) lastServoAngles[i] = -1;
  
  enableMotors(); // Asegurar estado inicial activo
}

void updateProportionalControl(float throttle, float yaw, float roll) {
  if (!motorsEnabled) {
    stopMotor();
    return;
  }
  currentThrottle = throttle;
  currentYaw = yaw;
  currentRoll = roll;

  if (abs(currentRoll) > 20) {
    executeCrabMovement();
  }
  else if (abs(currentYaw) > 15 && abs(currentThrottle) < 15) {
    executeRotation();
  }
  else {
    executeNormalMovement();
  }
}

void executeNormalMovement() {
  // Joystick Izquierda (-100) -> Engranaje invierte -> Llanta va a Izquierda
  int angleLeft_Front = map(currentYaw, -100, 100, 45, 135);
  int angleRight_Front = map(currentYaw, -100, 100, 45, 135); 
  
  // Las traseras giran opuesto a las delanteras para reducir radio de giro
  int angleLeft_Rear = map(currentYaw, -100, 100, 135, 45);
  int angleRight_Rear = map(currentYaw, -100, 100, 135, 45);

  setServoAngle(steeringServoChannels[0], angleLeft_Front, servoOffsets[0]);
  setServoAngle(steeringServoChannels[1], angleRight_Front, servoOffsets[1]);
  setServoAngle(steeringServoChannels[2], NEUTRAL_ANGLE, servoOffsets[2]);
  setServoAngle(steeringServoChannels[3], NEUTRAL_ANGLE, servoOffsets[3]);
  setServoAngle(steeringServoChannels[4], angleLeft_Rear, servoOffsets[4]);
  setServoAngle(steeringServoChannels[5], angleRight_Rear, servoOffsets[5]);

  // Lógica de velocidad diferencial 
  int baseSpeed = map(abs(currentThrottle), 0, 100, 0, 255);
  int dir = (currentThrottle >= 0) ? 1 : -1; 

  float yawEffect = map(abs(currentYaw), 0, 100, 1.0, 0.5); 
  int speedLeft = baseSpeed;
  int speedRight = baseSpeed;

  if (currentYaw > 15) speedRight *= yawEffect; // Girando derecha, derecha frena
  else if (currentYaw < -15) speedLeft *= yawEffect; // Girando izquierda, izquierda frena

  // Motores lado izquierdo (Pares: 0, 2, 4)
  for(int i=0; i<6; i+=2) setMotor(i, dir, speedLeft);  
  
  // Motores lado derecho (Impares: 1, 3, 5)
  for(int i=1; i<6; i+=2) setMotor(i, dir, speedRight);  
}

void executeCrabMovement() {
  const int CRAB_ANGLE = 2; 
  int dir = (currentRoll > 0) ? -1 : 1;

  for (int i = 0; i < STEERING_SERVOS; i++) {
    setServoAngle(steeringServoChannels[i], CRAB_ANGLE, servoOffsets[i]);
  }

  int targetSpeed = map(abs(currentRoll), 20, 100, 100, 190); 
  
  int finalSpeed = targetSpeed;
  
  for (int i = 0; i < 6; i++) setMotor(i, dir, finalSpeed);
}

void executeRotation() {
  // Modo Tanque (Giro sobre su eje)
  int angleLeft = map(abs(currentYaw), 20, 100, 90, 45);
  int angleRight = map(abs(currentYaw), 20, 100, 90, 135);

  setServoAngle(steeringServoChannels[0], angleLeft, servoOffsets[0]);
  setServoAngle(steeringServoChannels[2], 90, servoOffsets[2]);
  setServoAngle(steeringServoChannels[4], angleRight, servoOffsets[4]);
  
  setServoAngle(steeringServoChannels[1], angleRight, servoOffsets[1]);
  setServoAngle(steeringServoChannels[3], 90, servoOffsets[3]);
  setServoAngle(steeringServoChannels[5], angleLeft, servoOffsets[5]);

  int speed = map(abs(currentYaw), 20, 100, 120, 220); 
  int dir = (currentYaw > 0) ? 1 : -1; 

  // Motores izquierdos en una dirección
  setMotor(0, dir, speed);
  setMotor(2, dir, speed);
  setMotor(4, dir, speed);
  // Motores derechos en dirección opuesta
  setMotor(1, -dir, speed);
  setMotor(3, -dir, speed);
  setMotor(5, -dir, speed);
}

void stopMotor() {
  smoothedSpeed = 0;
  for (int i = 0; i < 6; i++) setMotor(i, 0, 0);
  currentThrottle = 0;
  currentYaw = 0;
  currentRoll = 0;
}

void emergencyStop() {
  stopMotor();
  centerAllServos(); 
  motorsEnabled = false; 
  // IMPORTANTE: No ponemos autoPilotEnabled = false aquí, 
  // eso lo maneja el main.cpp para no tener dependencias circulares.
}

void enableMotors() {
  motorsEnabled = true;
}

void centerAllServos() {
  for (int i = 0; i < STEERING_SERVOS; i++) {
    setServoAngle(steeringServoChannels[i], NEUTRAL_ANGLE, servoOffsets[i]);
  }
}

void setCameraTilt(int inputWeb) {
  // SERVO_UP: Ángulo donde la cámara mira hacia arriba (Tope derecho del slider)
  // SERVO_DOWN: Ángulo donde la cámara mira al frente/abajo (Tope izquierdo del slider)
  const int SERVO_UP = 0;    
  const int SERVO_DOWN = 110;

  // MAPEO INVERSO Y ESCALADO
  int targetAngle = map(inputWeb, 0, 180, SERVO_DOWN, SERVO_UP);
  int safeAngle = constrain(targetAngle, min(SERVO_UP, SERVO_DOWN), max(SERVO_UP, SERVO_DOWN));

  setServoAngle(cameraServoChannel, safeAngle, 0);
}

void setMotor(int motorID, int dir, int speed) {
  if (motorID < 0 || motorID > 5) return;
  speed = constrain(speed, 0, 255);
  
  int in1 = in1PwmChannels[motorID];
  int in2 = in2PwmChannels[motorID];

  // Lógica L298N sin pines Enable (PWM directo a IN1/IN2)
  if (dir == 1) {
    ledcWrite(in1, speed);
    ledcWrite(in2, 0);
  } else if (dir == -1) {
    ledcWrite(in1, 0);
    ledcWrite(in2, speed);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}


void setServoAngle(int channel, int angle, int offset) {
  if (channel < 0 || channel >= 16) return;
  int targetAngle = constrain(angle + offset, 0, 180);
  if (lastServoAngles[channel] == targetAngle) return; 
  lastServoAngles[channel] = targetAngle;
  int pulselen = map(targetAngle, 0, 180, pwmMin, pwmMax);
  pwm.setPWM(channel, 0, pulselen);
}