/* Global defining */
#define ONE_WIRE_BUS 7
#define DHT_SENSOR_PIN 11 
#define DHT_SENSOR_TYPE DHT11
#define IMU_ADDRESS 0x19
#define PERFORM_CALIBRATION
#include <SoftwareSerial.h>

/* Librarys */
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "FastIMU.h"
#include <Servo.h>
#include <Arduino_FreeRTOS.h>

#include <Filters.h>

/* Objects */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
BMI055 IMU;
calData calib = { 0 };
AccelData accelData, accelContainer;
GyroData gyroData , gyroContainer;
Servo myservo;
SoftwareSerial mySerial(19, 18); //SIM800L Tx & Rx is connected to Arduino #3 & #2

/* Constantes */ 
const String phone = "+213697586836";

/* Variables */
bool window = 0;
float temp = 0;
float gaz = 0;
float humidity = 0;
float flm = 0;
int position_servo = 0;


/* Seuil */
float seuil_temp = 30;
float seuil_gaz = 40;
float seuil_ecg = 200;
float seuil_accel_cap = 0;
float seuil_gyro_cap = 0;
int count_zeros_ecg = 0;

/* Commande */
bool act_ecg = 0;
bool act_accel = 0;
bool act_gyro = 0;
bool act_oxy = 0;
bool act_house = 0;
bool act_tempCorp = 0;

/* Alerte */
bool alert = 0;
String message_alert =  "";


/* Pins */
const int
  dht_11_Pin = 11,
  lop = 9,
  lom = 8,
  out = A0,
  cap_GAZ = A2,
  cap_FLM = 10,
  clim_Pin=12,
  servo_pin = 13;
