//**************************** Bibliotheken ****************************************************/
#include <ESP8266WiFi.h>  // ESP8266 Bibliothek
#include <Wire.h>  
#include <PubSubClient.h>	// Bibliothek für MQTT
#include <ArduinoJson.h>	// ArduinoJson V6 Bibliothek
#include <SoftwareSerial.h> // Use software serial since the hardware serial port is used by the USB interface. 
//Make sure to use ESP8266 version of SoftwareSerial library. See "EspSoftwareSerial".
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>     // Update over the air
#include <TelnetStream.h>   // TelnetStream für PUTTY-Telnet
#include <WiFiUdp.h> 
#include <ModbusMaster.h> //https://github.com/4-20ma/ModbusMaster //For Modbus 

//************************ SDM72 ***************************************************************/
//* SDM72 Energy Meter to Modbus RS485 to WIFI MQTT
//*
//* Modbus registers:
//-----------------------------------------------------------------------------------------------
//      REGISTERS LIST FOR SDM 72                                                               |
//-----------------------------------------------------------------------------------------------
//      REGISTER NAME                                 REGISTER ADDRESS              UNIT        |
//-----------------------------------------------------------------------------------------------
#define SDM_PHASE_1_VOLTAGE                           0x0000                    //  V           |
#define SDM_PHASE_2_VOLTAGE                           0x0002                    //  V           |
#define SDM_PHASE_3_VOLTAGE                           0x0004                    //  V           |
#define SDM_PHASE_1_CURRENT                           0x0006                    //  A           |
#define SDM_PHASE_2_CURRENT                           0x0008                    //  A           |
#define SDM_PHASE_3_CURRENT                           0x000A                    //  A           |
#define SDM_PHASE_1_POWER                             0x000C                    //  W           |
#define SDM_PHASE_2_POWER                             0x000E                    //  W           |
#define SDM_PHASE_3_POWER                             0x0010                    //  W           |
#define SDM_PHASE_1_APPARENT_POWER                    0x0012                    //  VA          |
#define SDM_PHASE_2_APPARENT_POWER                    0x0014                    //  VA          |
#define SDM_PHASE_3_APPARENT_POWER                    0x0016                    //  VA          |
#define SDM_PHASE_1_REACTIVE_POWER                    0x0018                    //  VAr         |
#define SDM_PHASE_2_REACTIVE_POWER                    0x001A                    //  VAr         |
#define SDM_PHASE_3_REACTIVE_POWER                    0x001C                    //  VAr         |
#define SDM_PHASE_1_POWER_FACTOR                      0x001E                    //              |
#define SDM_PHASE_2_POWER_FACTOR                      0x0020                    //              |
#define SDM_PHASE_3_POWER_FACTOR                      0x0022                    //              |
#define SDM_AVERAGE_L_TO_N_VOLTS                      0x002A                    //  V           |
#define SDM_AVERAGE_LINE_CURRENT                      0x002E                    //  A           |
#define SDM_SUM_LINE_CURRENT                          0x0030                    //  A           |
#define SDM_TOTAL_SYSTEM_POWER                        0x0034                    //  W           |
#define SDM_TOTAL_SYSTEM_APPARENT_POWER               0x0038                    //  VA          |
#define SDM_TOTAL_SYSTEM_REACTIVE_POWER               0x003C                    //  VAr         |
#define SDM_TOTAL_SYSTEM_POWER_FACTOR                 0x003E                    //              |
#define SDM_FREQUENCY                                 0x0046                    //  Hz          |
#define SDM_IMPORT_ACTIVE_ENERGY                      0x0048                    //  kWh/MWh     |
#define SDM_EXPORT_ACTIVE_ENERGY                      0x004A                    //  kWh/MWh     |
#define SDM_LINE_1_TO_LINE_2_VOLTS                    0x00C8                    //  V           |
#define SDM_LINE_2_TO_LINE_3_VOLTS                    0x00CA                    //  V           |
#define SDM_LINE_3_TO_LINE_1_VOLTS                    0x00CC                    //  V           |
#define SDM_AVERAGE_LINE_TO_LINE_VOLTS                0x00CE                    //  V           |
#define SDM_NEUTRAL_CURRENT                           0x00E0                    //  A           |
#define SDM_TOTAL_ACTIVE_ENERGY                       0x0156                    //  kWh         |
#define SDM_TOTAL_REACTIVE_ENERGY                     0x0158                    //  kVArh       |
#define SDM_CURRENT_RESETTABLE_TOTAL_ACTIVE_ENERGY    0x0180                    //  kWh         |
#define SDM_CURRENT_RESETTABLE_IMPORT_ENERGY          0x0184                    //  kWh         |
#define SDM_CURRENT_RESETTABLE_EXPORT_ENERGY          0x0186                    //  kWh         |
#define SDM_NET_KWH                                   0x018C                    //  kWh         |
#define SDM_IMPORT_POWER                              0x0500                    //  W           |
#define SDM_EXPORT_POWER                              0x0502                    //  W           |
//-----------------------------------------------------------------------------------------------
double Voltage_L1;
double Voltage_L2;
double Voltage_L3;
double Current_L1;
double Current_L2;
double Current_L3;
double Power_L1;
double Power_L2;
double Power_L3;
double Import_W;
double Export_W;
double Total_W;
double Import_kWh;
double Export_kWh;
double Total_kWh;
double Total_PF;
double Frequency;
double Baud;
  
char Voltage_L1_str[5];
char Voltage_L2_str[5];
char Voltage_L3_str[5];
char Current_L1_str[8];
char Current_L2_str[8];
char Current_L3_str[8];
char Power_L1_str[10];
char Power_L2_str[10];
char Power_L3_str[10];
char Import_W_str[10];
char Export_W_str[10];
char Total_W_str[10];
char Import_kWh_str[10];
char Export_kWh_str[10];
char Total_kWh_str[10];
char Total_PF_str[7];
char Frequency_str[5];
char Baud_str[5];


//************************ Init ModbusMaster Object ********************************************/

ModbusMaster node;
SoftwareSerial mySerial(14,12); // using pins 14 for RX and 12 for TX


//************************ Räume ***************************************************************/

#define RAUM "SDM72"

//************************* WiFi Access Point **************************************************/

#define WLAN_SSID       "FRITZ!Box"
#define WLAN_PASS       "xxxxxxxxx"

/************************* TIME STUFF **********************************************************/

#define MAX_LOOP_TIME_MS     10000  // Watchdog-Timer, if WLAN or MQTT failed
#define SEND_TIME_INTERVAL   60000  // Send Time Interval Publish MQTT

int pubCounter = 0; // Counter for Publish-Test
unsigned long lastPublishTime; // last Publish Time
unsigned long elapsedPublishTime; // elapsed Time since last Publish

unsigned long startTime;   // Start Time
unsigned long monitTime;   // Monitoring Time
unsigned long stopTime;   // Stop Time

//************************* MQTT ***************************************************************/

IPAddress MQTTServer(192, 168, 188, 64);  // IP-Adress of MQTT-Server - the Raspi
WiFiClient MQTTClient;
PubSubClient client(MQTTClient);
#define USER "User"
#define PASSW "xxxxxx"

//clientID : Name for Login at Server, muß einmalig sein im Netzwerk
#define CLIENT_ID "ESP_SDM72_WemosD1"

const char* outTopic ="SDM72"; // Send - Topic

//************************************ JSON ****************************************************/
//Beispiel
// Berechnung hier gemacht: https://arduinojson.org/v6/assistant/
// Größe für diese Daten : 384

// {
	// "Raum": "SDM72",
	// "Data": {
		// "Power_L1": "12345.1",
		// "Power_L2": "12345.1",
		// "Power_L3": "12345.1",
		// "Import_W": "12345.1",
		// "Export_W": "12345.1",
		// "Total_W": "12345.1",
		// "Import_kWh": "123456.12",
		// "Export_kWh": "123456.12",
		// "Total_kWh": "123456.12",
		// "Total_PF": "1.123",
		// "Frequency": "12.12"
	// }
// }


// Setup wird nur einmal durchlaufen ***********************************************************/
void setup() {
  Serial.begin(115000); //Serielle Verbindung starten
  Serial.println("Booting");
  
  Serial.println("Start");
  startTime = millis();  // Startzeit erfassen
  monitTime = startTime;  // Überwachungszeit erfassen
  
  // Wait for serial to initialize.
  while(!Serial) { }
  
  Serial.println("Using saved SSID: " + WiFi.SSID());
  delay(1);
  setupWifi();
  client.setServer(MQTTServer, 1883);
  
  
  // using SoftwareSerial initialize Modbus communication baud rate
  mySerial.begin(9600);

  // communicate with Modbus slave ID 1 over Serial.
  node.begin(1, mySerial);

  // TelnetStream
  TelnetStream.begin();

  // OTA essentiell
  ArduinoOTA.setHostname("SDM72");
  ArduinoOTA.setPassword("arduino05");
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
} // Ende void setup()

// Funktion "setupWifi"*************************************************************************/
void setupWifi() {
  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID); // Kontrollelement im seriellen Monitor
  String wifiname;
  wifiname = WiFi.SSID();
  Serial.print("Gespeichertes WiFi heißt: ");
  Serial.println(wifiname); // Kontrollelement im seriellen Monitor
  
  if (WiFi.SSID() != WLAN_SSID) {
    Serial.println("Configuring persistent wifi...");
    WiFi.begin(WLAN_SSID, WLAN_PASS);  // WLAN Verbindung mit WLAN_SSID Passwort herstellen
	  WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
  } else {
    Serial.println("Gespeichertes WiFi wird benutzt...");
  }
  WiFi.hostname(CLIENT_ID); // Client-ID wird auch für WiFi-Namen im Netz benutzt
}

// Funktion "Wait for WiFi"*********************************************************************/
void waitForWifi() {
  //Serial.print("Connecting to WiFi.");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {  // Solange noch keine WLAN-Verbindung besteht....
    delay(500);
    Serial.print("w"); // ... sollen Punkte ausgegeben werden. Die Punkte dienen als Kontrollelement.
    if ((millis() - monitTime) > MAX_LOOP_TIME_MS){
      Serial.println("WIFI FAILED !  Rebooting...");
      delay(3000);
      ESP.restart();}
  }
	//Serial.println("");
  //Serial.println("WiFi connected");
  //Serial.println("IP address: ");
  //Serial.println(WiFi.localIP()); // Nun wird die sogenannte localIP ausgegeben.
}

// Funktion "reconnect" ************************************************************************/
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(CLIENT_ID, USER, PASSW)) {   // CLIENT-ID, dann User und Passwort
      Serial.println("connected");
      //client.publish(outTopic, RAUM);
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      // Wait 3 seconds before retrying
      delay(3000);
      if ((millis() - monitTime) > MAX_LOOP_TIME_MS){
        Serial.println("MQTT FAILED !");
        ESP.restart();}
    }
  }
}

// Funktion "Generate Float from 2 UINT Modbus" ************************************************/
// reconstruct the float from 2 unsigned integers
float f_2uint_float(unsigned int uint1, unsigned int uint2) {

  union f_2uint {
    float f;
    uint16_t i[2];
  };

  union f_2uint f_number;
  f_number.i[0] = uint1;
  f_number.i[1] = uint2;
  //Serial.printf("Data Union: 0 Inhalt: %d \n",f_number.i[0]);
  //Serial.printf("Data Union: 1 Inhalt: %d \n",f_number.i[1]);

  return f_number.f;

}

// Funktion "Generate Long from 2 UINT Modbus" ************************************************/
// reconstruct the long from 2 unsigned integers
long l_2uint_long(unsigned int uint1, unsigned int uint2) {

  union l_2uint {
    long l;
    uint16_t i[2];
  };

  union l_2uint l_number;
  l_number.i[0] = uint1;
  l_number.i[1] = uint2;

  return l_number.l;

}
// Continuous Program **************************************************************************/
void loop() {
  // OTA
  ArduinoOTA.handle();
  
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];

  // Wifi
  waitForWifi();
   
  // MQTT
  if (!client.connected()) {
      reconnect();
  }
  client.loop();

  delay(5700);

    //*************** Read from Input Registers = Modbus function 0x04 *************************/
    result = node.readInputRegisters(0x0000, 6);  // Voltage L1, L2, L3
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 5; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Voltage_L1 = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Voltage_L1,5,1,Voltage_L1_str);
        Serial.printf("Voltage L1: %.1f V \n",Voltage_L1);
        TelnetStream.print("Voltage L1: ");
        TelnetStream.println(Voltage_L1_str);
        Voltage_L2 = f_2uint_float(data[3], data[2]);
        dtostrf(Voltage_L2,5,1,Voltage_L2_str);
        Serial.printf("Voltage L2: %.1f V \n",Voltage_L2);
        TelnetStream.print("Voltage L2: ");
        TelnetStream.println(Voltage_L2_str);
        Voltage_L3 = f_2uint_float(data[5], data[4]);
        dtostrf(Voltage_L3,5,1,Voltage_L3_str);
        Serial.printf("Voltage L3: %.1f V \n",Voltage_L3);
        TelnetStream.print("Voltage L3: ");
        TelnetStream.println(Voltage_L3_str);
    }
    
    delay(100);
    result = node.readInputRegisters(0x0006, 6);  // Current L1, L2, L3
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 5; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Current_L1 = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Current_L1,5,3,Current_L1_str);
        Serial.printf("Current L1: %s A \n",Current_L1_str);
        Current_L2 = f_2uint_float(data[3], data[2]);
        dtostrf(Current_L2,5,3,Current_L2_str);
        Serial.printf("Current L2: %s A \n",Current_L2_str);
        Current_L3 = f_2uint_float(data[5], data[4]);
        dtostrf(Current_L3,5,3,Current_L3_str);
        Serial.printf("Current L3: %s A \n",Current_L3_str);
    }
    
    delay(100);
    result = node.readInputRegisters(0x000C, 6);  // Power L1, L2, L3
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 5; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Power_L1 = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        Power_L1 = (-1)*Power_L1;
        dtostrf(Power_L1,3,1,Power_L1_str);
        Serial.printf("Power L1: %s W \n",Power_L1_str);
        Power_L2 = f_2uint_float(data[3], data[2]);
        Power_L2 = (-1)*Power_L2;
        dtostrf(Power_L2,3,1,Power_L2_str);
        Serial.printf("Power L2: %s W \n",Power_L2_str);
        Power_L3 = f_2uint_float(data[5], data[4]);
        Power_L3 = (-1)*Power_L3;
        dtostrf(Power_L3,3,1,Power_L3_str);
        Serial.printf("Power L3: %s W \n",Power_L3_str);
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_IMPORT_POWER, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Export_W = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Export_W,3,1,Export_W_str);
        Serial.printf("Export W: %s W \n",Export_W_str); 
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_EXPORT_POWER, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Import_W = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Import_W,3,1,Import_W_str);
        Serial.printf("Import W: %s W \n",Import_W_str); 
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_TOTAL_SYSTEM_POWER, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Total_W = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        Total_W = (-1) * Total_W;
        dtostrf(Total_W,3,1,Total_W_str);
        Serial.printf("Total W: %s W \n",Total_W_str);
        TelnetStream.print("Total_W: ");
        TelnetStream.println(Total_W_str); 
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_IMPORT_ACTIVE_ENERGY, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Export_kWh = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Export_kWh,5,3,Export_kWh_str);
        Serial.printf("Export kWh: %s kWh \n",Export_kWh_str); 
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_EXPORT_ACTIVE_ENERGY, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Import_kWh = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Import_kWh,5,3,Import_kWh_str);
        Serial.printf("Import kWh: %s kWh \n",Import_kWh_str); 
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_TOTAL_ACTIVE_ENERGY, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Total_kWh = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Total_kWh,5,3,Total_kWh_str);
        //Serial.printf("Total kWh: %.2f kWh \n",Total_kWh); 
        Serial.printf("Total kWh: %s kWh \n",Total_kWh_str);
        TelnetStream.print("Total_kWh: ");
        TelnetStream.println(Total_kWh_str); 
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_TOTAL_SYSTEM_POWER_FACTOR, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Total_PF = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        Total_PF = (-1) * Total_PF;
        dtostrf(Total_PF,5,3,Total_PF_str);
        //Serial.printf("Powerfactor: %.3f \n",Total_PF);
        Serial.printf("Powerfactor: %s \n",Total_PF_str);
    }
    
    delay(100);
    result = node.readInputRegisters(SDM_FREQUENCY, 2);
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Frequency = f_2uint_float(data[1], data[0]);  // get float from 2 unsigned int
        dtostrf(Frequency,5,2,Frequency_str);
        Serial.printf("Frequency: %.s Hz \n",Frequency_str);
    }
    
    //***************** Read from Holding Registers = Modbus function 0x03 *********************/
    delay(100);
    result = node.readHoldingRegisters(0xFC00, 2);    // Seriennummer
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        long SerNum = l_2uint_long(data[1], data[0]);  // get long from 2 unsigned int
        Serial.printf("Serial Num: %d \n",SerNum);  // Seriennummer
    }
    
    delay(100);
    result = node.readHoldingRegisters(0x001C, 2);    //Network Baud Rate
    if (result == node.ku8MBSuccess){
        for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          //Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
        Baud = f_2uint_float(data[1], data[0]);    // get float from 2 unsigned int
        dtostrf(Baud,3,1,Baud_str);
        Serial.printf("Baudrate: %s \n",Baud_str); // 2.0 = 9600 Baud
       }
        
    Serial.println("Fertig !");
    delay(100);
    
    
    // *****************************************************************************************/

    // Publish SDM72 alle 60sec.
    elapsedPublishTime = millis() - lastPublishTime; // verstrichene Zeit seit letztem Publish
    if (elapsedPublishTime >= SEND_TIME_INTERVAL){
      pubCounter = pubCounter + 1;
      Serial.printf("MQTT gesendet %d\n",pubCounter); //Im seriellen Monitor anzeigen
      TelnetStream.print("MQTT gesendet: ");
      TelnetStream.println(pubCounter); 
      lastPublishTime = millis(); // letzter Publish Zeitpunkt merken
      
      // Build JSON
      StaticJsonDocument<384> doc;
      doc["Raum"] = RAUM;
      JsonObject Data = doc.createNestedObject("Data");
    
      Data["Power_L1"] = Power_L1_str;
      Data["Power_L2"] = Power_L2_str;
      Data["Power_L3"] = Power_L3_str;
      Data["Import_W"] = Import_W_str;
      Data["Export_W"] = Export_W_str;
      Data["Total_W"]  = Total_W_str;
      Data["Import_kWh"] = Import_kWh_str;
      Data["Export_kWh"] = Export_kWh_str;
      Data["Total_kWh"] = Total_kWh_str;
      //Data["Total_PF"] = Total_PF_str;
      Data["Frequency"] = Frequency_str;
      

      // Serialize JSON
      char output[384];  // recommended size for ArduinoJson V6
      serializeJson(doc, output);
      // publish JSON
      client.publish(outTopic, output);
      Serial.println(output);
      delay(1000);
    }
}
