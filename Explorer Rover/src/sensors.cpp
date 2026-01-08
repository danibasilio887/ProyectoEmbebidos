#include "sensors.h"
#include <Arduino.h>
#include <DHT.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor.h>

// ============= PINES =============
#define DHTPIN 4
#define DHTTYPE DHT11

// Pines Ultrasónicos
#define TRIGGER_PIN_FRONT 5
#define ECHO_PIN_FRONT    39
#define TRIGGER_PIN_LEFT  15
#define ECHO_PIN_LEFT     34
#define TRIGGER_PIN_RIGHT 12
#define ECHO_PIN_RIGHT    35
#define MAX_DISTANCE 200     

// Batería
#define BAT_ADC_PIN 36
#define NUM_SAMPLES 20

// ============= OBJETOS =============
static DHT dht(DHTPIN, DHTTYPE);
static NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
static NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
static NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
static Adafruit_MPU6050 mpu;
static Adafruit_Madgwick filter;

// ============= VARIABLES =============
static float g_temp = 0.0;
static float g_humidity = 0.0;
static long  g_distFront = 999;
static long  g_distLeft = 999;
static long  g_distRight = 999;
static float g_roll = 0.0;
static float g_pitch = 0.0;
static float g_battery = 0.0; // Variable para batería

// ============= TIMERS =============
static unsigned long lastSonarPoll = 0;
static unsigned long lastImuPoll = 0;
static unsigned long lastDhtPoll = 0;
static unsigned long lastBatPoll = 0; // Timer batería

const unsigned long SONAR_INTERVAL = 100; 
const unsigned long IMU_INTERVAL = 10;    
const unsigned long DHT_INTERVAL = 2000;  
const unsigned long BAT_INTERVAL = 1000;  // Leer batería cada 1s

void sensors_init() {
  pinMode(BAT_ADC_PIN, INPUT); // Configurar pin ADC
  dht.begin();
  if (!mpu.begin()) {
    // Manejo de error silencioso o Serial
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    filter.begin(100); 
  }
}

void sensors_update() {
  unsigned long now = millis();

  // 1. IMU
  if (now - lastImuPoll >= IMU_INTERVAL) {
    lastImuPoll = now;
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
        filter.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z);
        g_roll = filter.getPitch(); // Lo que el sensor ve como Pitch, para el rover es Roll (Lateral)
        g_pitch = filter.getRoll(); // Lo que el sensor ve como Roll, para el rover es Pitch (Pendiente)
    }
  }

  // 2. Sonar
  if (now - lastSonarPoll >= SONAR_INTERVAL) {
    lastSonarPoll = now;
    long d = sonarFront.ping_cm(); g_distFront = (d == 0) ? 999 : d;
    d = sonarLeft.ping_cm(); g_distLeft = (d == 0) ? 999 : d;
    d = sonarRight.ping_cm(); g_distRight = (d == 0) ? 999 : d;
  }

  // 3. DHT
  if (now - lastDhtPoll > DHT_INTERVAL) {
      lastDhtPoll = now;
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (!isnan(h)) g_humidity = h;
      if (!isnan(t)) g_temp = t;
  }

  // 4. Batería
  if (now - lastBatPoll > BAT_INTERVAL) {
      lastBatPoll = now;
      long sum = 0;
      // Tomar 20 lecturas rápidas y promediarlas
      for(int i=0; i<NUM_SAMPLES; i++) {
          sum += analogReadMilliVolts(BAT_ADC_PIN);
          delay(2); // Pequeña pausa entre lecturas
      }
      float avgMv = sum / (float)NUM_SAMPLES;
      
      // Convertir a voltaje real
      // Factor = (R1 + R2) / R2
      // R1=33k, R2=10k -> Factor = 4.3
      float factor = 4.3; 
      float voltage = (avgMv / 1000.0) * factor; 
      
      g_battery = voltage;
  }
}

float get_temp() { return g_temp; }
float get_humidity() { return g_humidity; }
long  get_dist_front() { return g_distFront; }
long  get_dist_left() { return g_distLeft; }
long  get_dist_right() { return g_distRight; }
float get_angle_roll() { return g_roll; }
float get_angle_pitch() { return g_pitch; }
float get_battery() { return g_battery; }
float get_battery_voltage() { return g_battery; }