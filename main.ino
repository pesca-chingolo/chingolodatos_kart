// DEBUG MODE
//#define DEBUGGPS 1
//#define DEBUGWIFI 1
//#define DEBUGRPM 1
//#define DEBUGRPMFULL 1

// CONNECTION SETUP
#define SETUPBT 1
//#define SETUPWIFI 1

// INCLUDES BLUETOOTH
#ifdef SETUPBT
#include "BluetoothSerial.h"
#endif

// INCLUDES WIFI
#ifdef SETUPWIFI
#include "WiFi.h"
#endif

// INCLUDES GPS MODULE
#include "HardwareSerial.h"

// INCLUDES RPM MODULE
#include "RunningMedian.h"

/* DEFINES BLUETOOTH */
#ifdef SETUPBT
/* Check if Bluetooth configurations are enabled in the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#endif

/* DEFINES WIFI */
#ifdef SETUPWIFI
const char* wifiSSID     = "ChingoloDatos_Kart";
const char* wifiPassword = "PASSW0RD";
#endif

/* DEFINES GPS MODULE */
#define pinGPS_tx 17
#define pinGPS_rx 16

/* DEFINES RPM MODULE */
#define pulseTimeTimeout 600
#define pinRPM_rx 27

/* VARIABLES BLUETOOTH */
#ifdef SETUPBT
BluetoothSerial SerialBT;
const char *btPasswd = "8686";
#endif

/* VARIABLES WIFI */
#ifdef SETUPWIFI
WiFiServer wifiServer(8686);
WiFiClient wifiClient;
IPAddress ipServer(172,16,86,1);
IPAddress ipGateway(172,16,86,1);
IPAddress ipSubnet(255,255,255,0);
#endif

/* VARIABLES GPS */
HardwareSerial SerialGPS(1);

/* VARIABLES RPM MODULE */
RunningMedian pulseTimeSamples = RunningMedian(15);
unsigned long pulseTimeNow = 0;
unsigned long pulseTimeBefore = 0;
float pulseTimeCalc = 0;
float pulseRPM = 0;
int pulseFlag = 0;
int pulseCounter = 0;
unsigned long rpmTransmitTimeNow = 0;
unsigned long rpmTransmitTimeBefore = 0;
unsigned long rpmTransmitTimeCalc = 0;
unsigned long rpmTransmitCounter = 0;
String rpmTransmitSentence;
byte rpmTransmitChecksum = 0;


void setup() {
  // SETUP SERIAL
  Serial.begin(115200);

  // SETUP BLUETOOTH
  #ifdef SETUPBT
  SerialBT.begin("ChingoloDatos_Kart");
  SerialBT.setPin(btPasswd);
  Serial.println("Bluetooth Started! Ready to pair...");
  #endif

  // SETUP WIFI
  #ifdef SETUPWIFI
  Serial.print("Setting AP configuration...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ipServer, ipGateway, ipSubnet);
  WiFi.softAP(wifiSSID, wifiPassword);
  Serial.print("AP Started! Listening with ip:");
  Serial.println(WiFi.softAPIP());
  wifiServer.begin();
  #endif

  // SETUP GPS MODULE
  SerialGPS.setRxBufferSize(4096);
  SerialGPS.begin(115200, SERIAL_8N1, pinGPS_rx, pinGPS_tx);
  Serial.println("GPS Started! Ready to recieve data...");

  // SETUP RPM MODULE
  pinMode(pinRPM_rx, INPUT_PULLDOWN);
  attachInterrupt(27, pulseTrigger, HIGH);
}

void loop() {
  // GPS MODULE
  while (SerialGPS.available() > 0) {
    byte gpsData = SerialGPS.read();
    // SEND GPS DATA VIA BLUETOOTH
    #ifdef SETUPBT
    SerialBT.write(gpsData);
    #endif
    #ifdef DEBUGGPS
      Serial.write(gpsData);
    #endif
    if(gpsData == '\n') { // END OF LINE DETECTION
      break;
    }
  }
  // RPM MODULE
  if (pulseFlag == 1) {
    pulseTimeNow = millis();
    pulseTimeCalc = pulseTimeNow - pulseTimeBefore;
    pulseTimeSamples.add(pulseTimeCalc);
    pulseTimeBefore = pulseTimeNow;
    pulseCounter = pulseCounter + 1;
    if (pulseTimeSamples.getCount() >= 15) {
      pulseTimeCalc = pulseTimeSamples.getMedian();
      pulseRPM = 60 * (1 / (pulseTimeCalc / 1000));
      #ifdef DEBUGRPMFULL
        Serial.print("RPM: ");
        Serial.print(pulseRPM);
        Serial.print("\n");
      #endif
    }
    pulseFlag = 0;
  }
  else {
    pulseTimeNow = millis();
    pulseTimeCalc = pulseTimeNow - pulseTimeBefore;
    if (pulseTimeCalc >= pulseTimeTimeout) {
      pulseRPM = 0;
      #ifdef DEBUGRPMFULL
        Serial.print("RPM: ");
        Serial.print(pulseRPM);
        Serial.print("\n");
      #endif
    }
  }

  // SEND RPM DATA AT 10HZ
  rpmTransmitTimeNow = millis();
  rpmTransmitTimeCalc = rpmTransmitTimeNow - rpmTransmitTimeBefore;
  if (rpmTransmitTimeCalc >= 100) {
    rpmTransmitCounter = rpmTransmitCounter + 1;
    rpmTransmitTimeBefore = rpmTransmitTimeNow;
    //rpmTransmitSentence = "RC3,,,,,,,,,"+(String)pulseRPM+"0"+",,,,,,,,,,,,,,,,"; // RC3 Sentences
    rpmTransmitSentence = "RC2,,,,,,"+(String)pulseRPM+"0"+",,,,,,,,,"; // RC2 Sentences (mas simple)
    // XOR CHECKSUM CALCULATION
    for (int i = 0; i < rpmTransmitSentence.length(); i++) { 
      rpmTransmitChecksum = rpmTransmitChecksum ^ (byte)rpmTransmitSentence[i];
    }
    // SEND RPM DATA VIA BT
    #ifdef SETUPBT
    SerialBT.println("$"+rpmTransmitSentence+"*"+String(rpmTransmitChecksum, HEX));
    #endif
    // SEND RPM DATA VIA WIFI
    #ifdef SETUPWIFI
    if (!wifiClient.connected()) {
      wifiClient = wifiServer.available();
    }
    if (wifiClient.connected()) {
      wifiClient.println("$"+rpmTransmitSentence+"*"+String(rpmTransmitChecksum, HEX));
      #ifdef DEBUGWIFI
        Serial.print("Data sent over WiFi");
      #endif
    }
    #endif
    #ifdef DEBUGRPM
      Serial.print("$"+rpmTransmitSentence+"*"+String(rpmTransmitChecksum, HEX));
      Serial.print("\n");
    #endif
    rpmTransmitChecksum = 0;
  }
}

// RPM PULSE INTERRUPT
void pulseTrigger()
{
  pulseFlag = 1;
}
