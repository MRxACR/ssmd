void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (mySerial.available())
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

/* Call */
void TaskInitGSM()
{
  mySerial.begin(9600);

  Serial.println("{\"log\" : \"GSM Initializing\" }");


  mySerial.println("AT");
  while (1) {
    if (alert) {
      Serial.println("Aletre");
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\""+phone+"\"");
      updateSerial();
      mySerial.print(message_alert); //text content
      updateSerial();
      mySerial.write(26);
      updateSerial();

      vTaskDelay(3000 / portTICK_PERIOD_MS);

      mySerial.println("ATD+ "+phone+";");
      updateSerial();
      vTaskDelay(10000);
      updateSerial();
      mySerial.println("ATH"); //hang up


      alert = 0;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/* Servo moteur */
void decServo() {
  myservo.attach(servo_pin);  // attaches the servo on pin 9 to the servo object
}

void servoOn() {
  myservo.write(180);
}

void servoOff() {
  myservo.write(0);
}

void servoDeg(float deg) {
  myservo.write( map(deg, 0, 360, 0, 255) ); // Map et utilisée pour convertire le degré en une tension PWM
}

void decClim() {
  pinMode(clim_Pin,OUTPUT);
}

void climOn() {
  digitalWrite(clim_Pin,1);
}

void climOff() {
  digitalWrite(clim_Pin,0);
}

/*** RTOS Functions ***/

/* Task Handles*/
TaskHandle_t TH_Ecg;
TaskHandle_t TH_Oxy;
TaskHandle_t TH_House;
TaskHandle_t TH_Corp;
TaskHandle_t TH_AccelGyro;
TaskHandle_t TH_SerialRead;
TaskHandle_t TH_TaskInitGSM;

/* Accéléromètre */
void TaskAccelGyro() {

error:
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0 && (act_accel | act_gyro) ) {
    Serial.println("{\"log\" : \"Error initializing IMU:\" }");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    goto error;
  }

  //#ifdef PERFORM_CALIBRATION
  //  IMU.calibrateAccelGyro(&calib);
  //  delay(3000);
  //  IMU.init(calib, IMU_ADDRESS);
  //#endif

  if (err != 0) goto error;
  while (1) {
    if (act_gyro || act_accel) {
      IMU.update();
      IMU.getAccel(&accelData);
      IMU.getGyro(&gyroData);
      /*
            seuil_accel_cap = sqrt(accelData.accelX * accelData.accelX + accelData.accelY * accelData.accelY + accelData.accelZ * accelData.accelZ);
            seuil_gyro_cap = sqrt(gyroData.gyroX * gyroData.gyroX + gyroData.gyroY * gyroData.gyroY + gyroData.gyroY * gyroData.gyroY);

            Serial.println(seuil_accel_cap);
            //Serial.println(seuil_gyro_cap);
      */
      if (act_accel) Serial.println("{\"accel\" : { \"x\" : " + String(accelData.accelX) + " , \"y\" : " + String(accelData.accelY) + " , \"z\" : " + String(accelData.accelZ) + " } }");
      if (act_gyro) Serial.println("{\"gyro\" : { \"x\" : " + String(gyroData.gyroX) + " , \"y\" : " + String(gyroData.gyroY) + " , \"z\" : " + String(gyroData.gyroZ) + " } }");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS); //  5s
  }
}

/* ECG */
void TaskEcg( void *pvParameters )
{
  // declaration
  pinMode(lop, INPUT);  // Setup for leads off detection LO +
  pinMode(lom, INPUT);  // Setup for leads off detection LO -

  // corp
  while (1) {
    if (act_ecg) {
      if ((digitalRead(lop) == 1) || (digitalRead(lom) == 1)) {
        Serial.println("{\"log\" : \"Ecg error\" }");
      } else {
        int ecg_read = analogRead(out);

        if (ecg_read < 5) count_zeros_ecg++;
        else count_zeros_ecg = 0;

        if (count_zeros_ecg > seuil_ecg) {
          message_alert = "dysfonctionnement éléctrique au niveau du coeur";
          alert = 1;
        }
        //Serial.println(ecg_read);
        Serial.println("{\"ecg\" : " + String( ecg_read ) + "}");
      }
    }

    vTaskDelay(30 / portTICK_PERIOD_MS); //  1ms
  }
};

/* House : Gaz,Flamme,Temperature,Humidité  */
void TaskHouse( void *pvParameters )
{
  pinMode(cap_GAZ, INPUT);
  pinMode(cap_FLM, INPUT_PULLUP);
  DHT dht_sensor(dht_11_Pin, DHT11);

  while (1) {
    gaz = analogRead(cap_GAZ) * 5 / 1024;
    flm = digitalRead(cap_FLM);
    temp = dht_sensor.readTemperature();
    humidity = dht_sensor.readHumidity();

    if (!flm) {
      window = 1;
      servoOn();
      message_alert = "Alerte de Flamme";
      alert = 1;
    }
    else {
      window = 0;
    }

    if (gaz >= seuil_gaz) {
      window = 1;
      message_alert = "Alerte de Gaz";
      alert = 1;
      servoOn();
    }
    else  window = 0;


    if (act_house) {
      Serial.println("{\"House\" : 21}") ;
      Serial.println("{\"flm\" : " + String( flm ) + "}") ;
      Serial.println("{\"gaz\" : " + String( gaz ) + "}") ;
      Serial.println("{\"temperature\" :" + String( temp ) + "}") ;
      Serial.println("{\"humedity\" :" + String(  humidity ) + "}") ;
    }


    vTaskDelay(1000 / portTICK_PERIOD_MS); //  5s
  }
};

/* Corp */
void TaskCorp( void *pvParameters )
{
  sensors.begin();
  while (1) {
    //sensors.getTempCByIndex(0)
    if (act_tempCorp) {
      sensors.requestTemperatures();
      Serial.println("{\"tempCorp\" : " + String( random(30, 40) ) + "}") ;
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); //  2s
  }
};
String a;


/* Serial Read */
void TaskSerialRead( void *pvParameters )
{
  (void) pvParameters;
  // Pour Activer ou désactiver les capteurs
  for (;;)
  {
    if (Serial.available() > 0) {
      char c = Serial.read();
      switch (c) {

        // ECG
        case 'a':
          act_ecg = 1;
          break;
        case 'b':
          act_ecg = 0;
          break;

        // OXY
        case 'c':
          act_oxy = 1;
          break;
        case 'd':
          act_oxy = 0;
          break;

        // TempCorp
        case 'e':
          act_tempCorp = 1;
          break;
        case 'f':
          act_tempCorp = 0;
          break;

        // Accéleromètre
        case 'g':
          act_accel = 1;
          break;
        case 'h':
          act_accel = 0;
          break;

        // Gyroscope
        case 'i':
          act_gyro = 1;
          break;
        case 'j':
          act_gyro = 0;
          break;

        // House
        case 'k':
          act_house = 1;
          break;
        case 'l':
          act_house = 0;
          break;

        case 'm':
          servoOn();
          break;

        case 'n':
          servoOff();
          break;

          
        case 'o':
          climOn();
          break;
          
        case 'p':
          climOff();
          break;

        default:
          // statements
          break;
      }

    }
    vTaskDelay( 250 / portTICK_PERIOD_MS );
  }
};



void setup() {
  Serial.begin(115200);

  decServo();
  decClim();
  /**/
    // Create task for Arduino led
    xTaskCreate(TaskInitGSM,
                "GSM",
                256, // Stack size
                NULL,
                8, // Priority
                &TH_TaskInitGSM );
  
  // Create task for Arduino led
  xTaskCreate(TaskSerialRead,
              "Serial Read",
              256, // Stack size
              NULL,
              3, // Priority
              &TH_SerialRead );


  /**/
  xTaskCreate(
    TaskEcg,
    "Eléctrocardiogramme",
    128,
    NULL,
    3,
    &TH_Ecg);

  xTaskCreate(
    TaskHouse,
    "Domotique",
    512,
    NULL,
    1,
    &TH_House);
  /**/
  xTaskCreate(
    TaskCorp,
    "Temperature Corp",
    128,
    NULL,
    1,
    &TH_Corp);

  xTaskCreate(
    TaskOxy,
    "Oxymètre de poul",
    256,
    NULL,
    2,
    &TH_Oxy);

  /**/
  xTaskCreate(
    TaskAccelGyro,
    "Accéléromètre",
    256,
    NULL,
    2,
    &TH_AccelGyro);

}

void loop() {
  // put your main code here, to run repeatedly:

}
