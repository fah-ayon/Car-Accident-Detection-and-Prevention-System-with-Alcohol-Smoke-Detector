/*
Setup Instructions for ESP32

1. Select the correct board:
   - Go to Tools → Board → ESP32 → Select your ESP32 model.
2. If using ESP32 for the first time:
   - Open File → Preferences.
   - In "Additional Boards Manager URLs" paste this link:
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   - Press OK.
   - Then go to Tools → Board → Boards Manager, search for "esp32" and install it.
3. Required Libraries:
   - Go to Sketch → Include Library → Manage Libraries.
   - Install the following:
       i. LiquidCrystal_I2C (by Frank de Brabander)
      ii. MPU6050 (by Electronic Cats)
4. Select the correct port:
   - Tools → Port → Your Port (e.g., COM3, COM4).
   - To find the port number:
       - Press Win + R, type devmgmt.msc, and press Enter.
       - Check under "Ports (COM & LPT)" in Device Manager.
5. Uploading Code (important for some ESP32 boards):
   - If you see "Connecting..." error during upload:
       a) Press and hold the BOOT button on ESP32 while uploading.
       b) Release the BOOT button when "Connecting..." appears.
   - If it still doesn’t upload:
       a) Press and hold the BOOT button again while uploading.
       b) This time, press and release the RESET button once while still holding BOOT.
       c) Release BOOT when "Connecting..." appears.
6. Serial Monitor:
   - Make sure the baud rate is set to 115200,
     otherwise you’ll see text that looks like it’s from the 15th century
That’s it! You are ready to go.
Best of luck
— Abdullah Al Fahad  
LinkedIn: https://www.linkedin.com/in/abdullahalfahadayon/
*/


#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MPU6050.h"

// --- Wi-Fi & Telegram ---
const char* ssid = "Place your WiFi ssid here";
const char* password = "Place your WiFi password here";
String botToken = "Place Your BOT token here";
String chatIDs[] = {"Place your telegram chatIDs here"}; 
int numChats = sizeof(chatIDs) / sizeof(chatIDs[0]);


// --- Motor & Joystick ---
#define VRX_PIN 32  // ADC1 pin for joystick X
#define ENA_PIN 18
#define INA_PIN 13
#define INB_PIN 12

int center = 1920;  //0     // calibrated
int deadzone = 300;      // prevent jitter
int maxAnalog = 4095;
bool ignitionOn = false;
bool lastIgnitionState = HIGH;

// --- Ignition & LEDs ---
#define IGNITION_PIN 14
#define LED_IGNITION 19
#define LED_WARNING 2
#define BUZZER_PIN 5
#define RESET_PIN 27

// --- Gas & Alcohol Sensors ---
#define MQ2_PIN 35
#define MQ3_PIN 34
#define GAS_THRESHOLD 2300
#define ALCOHOL_THRESHOLD 1700
bool gasDetected = false;
bool alcoholDetected = false;

// --- Moving average buffers ---
#define SENSOR_SAMPLES 5
int mq2Buffer[SENSOR_SAMPLES] = {0};
int mq3Buffer[SENSOR_SAMPLES] = {0};
int sensorIndex = 0;
int avgMQ2() { long sum=0; for(int i=0;i<SENSOR_SAMPLES;i++) sum+=mq2Buffer[i]; return sum/SENSOR_SAMPLES; }
int avgMQ3() { long sum=0; for(int i=0;i<SENSOR_SAMPLES;i++) sum+=mq3Buffer[i]; return sum/SENSOR_SAMPLES; }

// --- Telegram flags ---
bool gasSent=false, alcoholSent=false, collisionSent=false;
bool gasLocationSent=false, alcoholLocationSent=false, collisionLocationSent=false;

// --- MPU6050 ---
MPU6050 mpu;
bool collisionDetected=false;
#define COLLISION_X_THRESHOLD 0.85
#define COLLISION_Z_THRESHOLD 0.7
unsigned long collisionTime=0;
const unsigned long COLLISION_DEBOUNCE=200;
const int COLLISION_COUNT_LIMIT=3;
int collisionCounter=0;

// --- Vibration sensor ---
#define VIB_PIN 25   // updated
unsigned long vibLastTime = 0;
const unsigned long VIB_DEBOUNCE = 100; // ms
int vibCounter = 0;
bool vibDetected = false;

// --- Ignition debounce ---
unsigned long lastIgnitionPress=0;

// --- GPS ---
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial gpsSerial(1);
String gpsLatitude = "";
String gpsLongitude = "";

// --- LCD ---
LiquidCrystal_I2C lcd(0x27,16,2);


// --- LCD Helper ---
void lcdPrintFixed(int col, int row, String text) {
  while (text.length() < 16) text += " "; // pad with spaces
  lcd.setCursor(col, row);
  lcd.print(text);
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // GPS UART
  delay(100);

  // Connect WiFi
  WiFi.begin(ssid,password);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart=millis();
  while(WiFi.status()!=WL_CONNECTED){
    delay(300); Serial.print(".");
    if(millis()-wifiStart>15000){ Serial.println("\nWiFi connect timeout"); break; }
  }
  if(WiFi.status()==WL_CONNECTED){
    Serial.println("\nConnected to WiFi");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  }

  // Pins
  pinMode(VRX_PIN, INPUT);
  pinMode(ENA_PIN, OUTPUT); pinMode(INA_PIN, OUTPUT); pinMode(INB_PIN, OUTPUT);
  pinMode(IGNITION_PIN, INPUT_PULLUP);
  pinMode(LED_IGNITION, OUTPUT); pinMode(LED_WARNING, OUTPUT); pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(MQ2_PIN, INPUT); pinMode(MQ3_PIN, INPUT);
  pinMode(VIB_PIN, INPUT);

  // Initial motor & LEDs
  digitalWrite(INA_PIN, LOW); digitalWrite(INB_PIN, LOW); analogWrite(ENA_PIN,0);
  digitalWrite(LED_IGNITION, LOW); digitalWrite(LED_WARNING, LOW); digitalWrite(BUZZER_PIN, LOW);

  // LCD
  Wire.begin(21,22);
  lcd.init(); lcd.backlight();
  lcdPrintFixed(0,0,"Ignition: OFF");
  lcdPrintFixed(0,1,"Motor: STOP");


  // MPU6050
  mpu.initialize();
  if(mpu.testConnection()) Serial.println("MPU6050 ready");
  else Serial.println("MPU6050 connection failed");
}

// --- URL encode & Telegram ---
String urlEncode(const String &str){
  String encoded=""; char c;
  for(unsigned int i=0;i<str.length();i++){
    c=str[i];
    if((c>='0'&&c<='9')||(c>='A'&&c<='Z')||(c>='a'&&c<='z')||c=='-'||c=='_'||c=='.'||c=='~') encoded+=c;
    else if(c==' ') encoded+="%20";
    else{ char buf[4]; sprintf(buf,"%%%02X",(uint8_t)c); encoded+=buf; }
  }
  return encoded;
}

void sendTelegram(const String &message){
  if(WiFi.status()!=WL_CONNECTED){ Serial.println("WiFi not connected"); return; }
  HTTPClient http;

  for(int i=0; i<numChats; i++){
    String url="https://api.telegram.org/bot"+botToken+"/sendMessage?chat_id="+chatIDs[i]+"&text="+urlEncode(message);
    Serial.println("Sending Telegram to " + chatIDs[i] + ": " + message);
    http.begin(url);
    int httpResponse=http.GET();
    Serial.print("HTTP response: "); Serial.println(httpResponse);
    http.end();
  }
}

// Buzzer pattern
void alertBuzzer(){
  for(int i=0;i<2;i++){ digitalWrite(BUZZER_PIN,HIGH); delay(150); digitalWrite(BUZZER_PIN,LOW); delay(150); }
}

// --- Parse GPS NMEA for latitude and longitude ---
void parseGPS(String nmea){
  if(nmea.startsWith("$GPGGA")){
    int commaIndex[15]; int j=0;
    for(int i=0;i<nmea.length();i++){
      if(nmea[i]==','){ 
        commaIndex[j++]=i; 
        if(j>=15) break; 
      }
    }
    if(j>=6){
      String rawLat = nmea.substring(commaIndex[1]+1, commaIndex[2]);
      String rawLon = nmea.substring(commaIndex[3]+1, commaIndex[4]);

      // Convert from NMEA (DDMM.MMMM) → Decimal Degrees (DD.DDDDDD)
      gpsLatitude = String(convertToDecimal(rawLat), 6);
      gpsLongitude = String(convertToDecimal(rawLon), 6);
    }
  }
}

// --- Convert NMEA (DDMM.MMMM) to Decimal Degrees (DD.DDDDDD) ---
double convertToDecimal(String nmeaCoord) {
  if (nmeaCoord == "") return 0.0;
  double val = nmeaCoord.toDouble();
  int degrees = (int)(val / 100);
  double minutes = val - (degrees * 100);
  return degrees + (minutes / 60.0);
}



void loop() {
  // Reset button
  if(digitalRead(RESET_PIN)==LOW){
    gasDetected=false; alcoholDetected=false; collisionDetected=false;
    collisionCounter=0; ignitionOn=false;
    gasSent=false; alcoholSent=false; collisionSent=false;
    gasLocationSent=false; alcoholLocationSent=false; collisionLocationSent=false;
    lcd.clear();
    lcdPrintFixed(0,0,"Ignition: OFF");
    lcdPrintFixed(0,1,"Motor: STOP");
    digitalWrite(LED_WARNING, LOW); digitalWrite(BUZZER_PIN, LOW); digitalWrite(LED_IGNITION, LOW);
    Serial.println("System reset pressed. Alerts cleared.");
    delay(500);
  }

  // Ignition button
  bool ignitionBtn=digitalRead(IGNITION_PIN);
  if(ignitionBtn==LOW && lastIgnitionState==HIGH){
    ignitionOn=!ignitionOn;
    digitalWrite(LED_IGNITION, ignitionOn?HIGH:LOW);
    lcdPrintFixed(0,0,"Ignition: " + String(ignitionOn ? "ON" : "OFF"));
    lastIgnitionPress=millis();
    delay(200);
  }
  lastIgnitionState=ignitionBtn;
  if(millis()-lastIgnitionPress<200) return;

  // Sensors moving average
  mq2Buffer[sensorIndex] = analogRead(MQ2_PIN);
  mq3Buffer[sensorIndex] = analogRead(MQ3_PIN);
  sensorIndex = (sensorIndex + 1) % SENSOR_SAMPLES;
  int mq2Value = avgMQ2();
  int mq3Value = avgMQ3();
  if(mq2Value >= GAS_THRESHOLD) gasDetected=true;
  if(mq3Value >= ALCOHOL_THRESHOLD) alcoholDetected=true;

  // MPU6050
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float accelX=ax/16384.0, accelY=ay/16384.0, accelZ=az/16384.0;

  // Vibration sensor
  bool vib = digitalRead(VIB_PIN);
  if(vib){
    if(millis()-vibLastTime>VIB_DEBOUNCE){
      vibCounter++;
      vibLastTime = millis();
    }
  }
  if(vibCounter >= 3){
    collisionDetected = true;
    vibDetected = true;
    vibCounter = 0;
    Serial.println("Vibration alert triggered!");
  } else vibDetected = false;

  // Collision detection MPU6050
  if(abs(accelX)>COLLISION_X_THRESHOLD || accelZ<COLLISION_Z_THRESHOLD){
    collisionCounter++;
  } else collisionCounter=0;
  if(collisionCounter>=COLLISION_COUNT_LIMIT && millis()-collisionTime>COLLISION_DEBOUNCE){
    collisionDetected=true; collisionTime=millis();
    Serial.println("Collision latched (X="+String(accelX)+", Y="+String(accelY)+", Z="+String(accelZ)+")");
  }

  // --- Read GPS data ---
  while(gpsSerial.available()){
    String nmea = gpsSerial.readStringUntil('\n');
    parseGPS(nmea);
  }
    if (gpsLatitude.length() > 0 && gpsLongitude.length() > 0) {
    Serial.println("GPS LOCKED | Lat: " + gpsLatitude + " Lon: " + gpsLongitude);
  } else {
    Serial.println("GPS NOT LOCKED yet...");
  }


  // Alerts
  if(gasDetected || alcoholDetected || collisionDetected){
    ignitionOn=false;
    digitalWrite(LED_IGNITION, LOW); digitalWrite(LED_WARNING,HIGH); alertBuzzer();
    lcd.setCursor(0,0);
    String msg="";
    // Build a combined message and send alerts + locations for each active incident
    if(collisionDetected){
      msg += "COLLISION ";
      if(!collisionSent){ sendTelegram("ALERT: Collision detected!"); collisionSent=true; }
      if(collisionSent && !collisionLocationSent && gpsLatitude.length()>0 && gpsLongitude.length()>0){
        //sendTelegram("Location: Lat " + gpsLatitude + " Lon " + gpsLongitude);
        sendTelegram("Location: https://www.google.com/maps?q=" + gpsLatitude + "," + gpsLongitude);
        collisionLocationSent = true;
      }
    }
    if(gasDetected){
      msg += "GAS ";
      if(!gasSent){ sendTelegram("ALERT: Gas detected!"); gasSent=true; }
      if(gasSent && !gasLocationSent && gpsLatitude.length()>0 && gpsLongitude.length()>0){
        //sendTelegram("Location: Lat " + gpsLatitude + " Lon " + gpsLongitude);
        sendTelegram("Location: https://www.google.com/maps?q=" + gpsLatitude + "," + gpsLongitude);
        gasLocationSent = true;
      }
    }
    if(alcoholDetected){
      msg += "ALCOHOL ";
      if(!alcoholSent){ sendTelegram("ALERT: Alcohol detected!"); alcoholSent=true; }
      if(alcoholSent && !alcoholLocationSent && gpsLatitude.length()>0 && gpsLongitude.length()>0){
        //sendTelegram("Location: Lat " + gpsLatitude + " Lon " + gpsLongitude);
        sendTelegram("Location: https://www.google.com/maps?q=" + gpsLatitude + "," + gpsLongitude);
        alcoholLocationSent = true;
      }
    }
    // Print combined message on LCD (truncate to 16 chars if necessary)
    if(msg.length()==0) msg = "Incident";
    if(msg.length()>16) msg = msg.substring(0,16);
    lcdPrintFixed(0,0,msg);
    lcdPrintFixed(0,1,"MQ2:" + String(mq2Value) + " MQ3:" + String(mq3Value));

  } else {
    digitalWrite(BUZZER_PIN,LOW); digitalWrite(LED_WARNING,LOW);
  }

  // Motor control
  int joyX = analogRead(VRX_PIN);

  // Serial Monitor
  Serial.print("Joystick X: "); Serial.print(joyX);
  Serial.print(" | MQ2: "); Serial.print(mq2Value);
  Serial.print(" | MQ3: "); Serial.print(mq3Value);
  Serial.print(" | AccelX: "); Serial.print(accelX);
  Serial.print(" | AccelY: "); Serial.print(accelY);
  Serial.print(" | AccelZ: "); Serial.print(accelZ);
  Serial.print(" | Vibration: "); Serial.println(vibDetected ? "YES" : "NO");

  if(ignitionOn && !gasDetected && !alcoholDetected && !collisionDetected){
    int speed = 0;
    String motorState = "STOP";

    if(joyX < center - deadzone){
      digitalWrite(INA_PIN, LOW); digitalWrite(INB_PIN, HIGH);
      speed = map(center - joyX, 0, center, 0, 255); speed = constrain(speed,0,255);
      analogWrite(ENA_PIN, speed); motorState = "BACKWARD";
    } else if(joyX > center + deadzone){
      digitalWrite(INA_PIN, HIGH); digitalWrite(INB_PIN, LOW);
      speed = map(joyX - center, 0, maxAnalog - center, 0, 255); speed = constrain(speed,0,255);
      analogWrite(ENA_PIN, speed); motorState = "FORWARD";
    } else {
      digitalWrite(INA_PIN, LOW); digitalWrite(INB_PIN, LOW); analogWrite(ENA_PIN,0);
      motorState = "STOP";
    }
    lcdPrintFixed(0,1,"Motor: " + motorState);
  } else {
    digitalWrite(INA_PIN,LOW); digitalWrite(INB_PIN,LOW); analogWrite(ENA_PIN,0);
    if(!gasDetected && !alcoholDetected) lcdPrintFixed(0,1,"Motor: STOP");
  }

  delay(50);
}