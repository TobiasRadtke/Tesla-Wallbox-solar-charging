#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <ModbusIP_ESP8266.h>
#include <ArduinoOTA.h>

int ip1 = 0;
int ip2 = 0;
int ip3 = 0;
int ip4 = 0;

uint16_t port = 502;
ModbusIP mb;  //ModbusIP object
IPAddress remote(ip1, ip2, ip3, ip4);  // AdddataFromBuss of Modbus Slave device


SoftwareSerial RS485Serial;
//SoftwareSerial RS485Serial(RXPin, TXPin); // RX, TX
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
HTTPClient https;
WiFiClientSecure client;
ESP8266WebServer server(80);

void handleRoot();              // function prototypes for HTTP handlers


byte heartBeatMessageStayAlive[] = {0xC0, 0xFB, 0xE0, 0x77, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 , 0x00, 0x00 , 0xFF, 0xC0};

byte heartBeatMessageData[] = {0xC0, 0xFB, 0xE0, 0x77, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC0};

/*
  Byte 6 and 7 slave adress
  Byte 8, 05 Tell slave charger to limit power to number of amps in bytes 2-3.
  09 Tell slave charger to limit power to number of amps in bytes 2 - 3.
  This command replaces the 05 command in Protocol 1. However, 05
  continues to be used, but only to set an amp value to be used
  before a car starts charging. If 05 is sent after a car is
  already charging, it is ignored.
*/



byte messageFromTwcSlave[22];

const byte masterId[] = {0x77, 0x77};
byte slaveId[2];
const byte masterSign[] = {0x77};

unsigned long timeLastHeartbeat = 0;
unsigned long lastSolarCheck = 0;
unsigned long lastModbusConnect = 0;

int ampsToCharge = 0;
byte ampsToWriteSlave[2];
int switchVar1 = 1;
float reportedAmpsActual;
String buttonState = "ON";
bool useSolar = true;
bool firstMaxCurrent = true;
int minAmpsToCharge;
float solarAmps;
int16_t dataConvertToInt;
String modbusConnect = "Error!";
String boxConnect = "Error!";
String buttonOne = "rgb(10,177,26)";
String buttonTwo = "rgb(245,245,245)";
String dataFromSlave;
String dataFromMaster;
String st;
int checksumStatus;
uint16_t meter = 0;
uint8_t unit = 0;
int meterDataWatt[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int meterDataAmps[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int voltage = 0;
int plusOrMinus = 0;
int plusOrMinus2 = 0;
int ampsMeterSelect1 = 0;
int ampsMeterSelect2 = 0;
int ampsMeterSelect3 = 0;
int solarWatts = 0;
float batteryData[2];

struct status
{
  bool ready;
  bool charging;
  bool error;
  bool pluggedIn;
  bool plInReadyCharge;
  bool busy;
  bool startingCharge;

} statusSlave;

String esid;
String epass = "";

void setup() {
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  Serial.begin(9600);

  WiFi.disconnect();
  EEPROM.begin(512);

  EEPROM.get(100, minAmpsToCharge);
  Serial.println("minAmpsToCHarge");
  Serial.println(minAmpsToCharge);


  for (int i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);
  Serial.println("Reading EEPROM pass");



  for (int i = 32; i < 96; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);



  String ipAdress1 = "";
  for (int i = 300; i < 315; ++i)
  {
    ipAdress1 += char(EEPROM.read(i));
  }
  Serial.print("ip Adress inverter1:");
  Serial.println(ipAdress1);
  ip1 = ipAdress1.toInt();

  String ipAdress2 = "";
  for (int i = 315; i < 330; ++i)
  {
    ipAdress2 += char(EEPROM.read(i));
  }
  Serial.print("ip Adress inverter2:");
  Serial.println(ipAdress2);
  ip2 = ipAdress2.toInt();

  String ipAdress3 = "";
  for (int i = 330; i < 345; ++i)
  {
    ipAdress3 += char(EEPROM.read(i));
  }
  Serial.print("ip Adress inverter3:");
  Serial.println(ipAdress3);
  ip3 = ipAdress3.toInt();

  String ipAdress4 = "";
  for (int i = 345; i < 370; ++i)
  {
    ipAdress4 += char(EEPROM.read(i));
  }
  Serial.print("ip Adress inverter4:");
  Serial.println(ipAdress4);
  ip4 = ipAdress4.toInt();

  String port2 = "";
  for (int i = 370; i < 400; ++i)
  {
    port2 += char(EEPROM.read(i));
  }
  Serial.print("MODBUS Port:");
  Serial.println(port2);
  port = port2.toInt();


  remote = {ip1, ip2, ip3, ip4}; // AdddataFromBuss of Modbus Slave device
  Serial.print("Inverterip");
  Serial.print(remote);

  String meter1 = "";
  for (int i = 445; i < 460; ++i)
  {
    meter1 += char(EEPROM.read(i));
  }
  Serial.print("meter1:");
  Serial.println(meter1);
  meter = meter1.toInt();

  String unit1 = "";
  for (int i = 460; i < 470; ++i)
  {
    unit1 += char(EEPROM.read(i));
  }
  Serial.print("unit1:");
  Serial.println(unit1);
  unit = unit1.toInt();

  String voltage1 = "";
  for (int i = 470; i < 480; ++i)
  {
    voltage1 += char(EEPROM.read(i));
  }
  Serial.print("voltage:");
  Serial.println(voltage1);
  voltage = voltage1.toInt();

  String meterSelect11 = "";
  for (int i = 480; i < 490; ++i)
  {
    meterSelect11 += char(EEPROM.read(i));
  }
  Serial.print("meterSelect1:");
  Serial.println(meterSelect11);
  ampsMeterSelect1 = meterSelect11.toInt();

  String plusOrMinus1 = "";
  for (int i = 490; i < 500; ++i)
  {
    plusOrMinus1 += char(EEPROM.read(i));
  }
  Serial.print("plusOrMinus:");
  Serial.println(plusOrMinus1);
  plusOrMinus = plusOrMinus1.toInt();

  String meterSelect21 = "";
  for (int i = 500; i < 510; ++i)
  {
    meterSelect21 += char(EEPROM.read(i));
  }
  Serial.print("meterSelect2ge:");
  Serial.println(meterSelect21);
  ampsMeterSelect2 = meterSelect21.toInt();


  String plusOrMinus21 = "";
  for (int i = 510; i < 520; ++i)
  {
    plusOrMinus21 += char(EEPROM.read(i));
  }
  Serial.print("plusOrMinus2:");
  Serial.println(plusOrMinus21);
  plusOrMinus2 = plusOrMinus21.toInt();

  String meterSelect31 = "";
  for (int i = 520; i < 530; ++i)
  {
    meterSelect31 += char(EEPROM.read(i));
  }
  Serial.print("meterSelect3:");
  Serial.println(meterSelect31);
  ampsMeterSelect3 = meterSelect31.toInt();

  Serial.print("unit1:");
  Serial.println(unit1);
  unit = unit1.toInt();



  WiFi.hostname("TWC-Bridge");
  WiFi.begin ( esid.c_str(), epass.c_str() );
  if (testWifi())
  {
    Serial.println("Succesfully Connected!!!");

  }
  else
  {
    Serial.println("Turning the HotSpot On");

    setupAP();// Setup HotSpot
  }





  RS485Serial.begin(9600, SWSERIAL_8N1, D5, D6);
  server.begin();                            // Actually start the server
  Serial.println("HTTP server started");
  Serial.print ( "Connected to " ); Serial.println ( esid );
  Serial.print ( "IP address: " ); Serial.println ( WiFi.localIP() );
  server.on("/",  handleRoot);        // Call the 'handleRoot' function when a client requests URI "/"

  while ((WiFi.status() != WL_CONNECTED))
  {
    Serial.print(".");
    delay(100);
    server.handleClient();
  }


  //GetCurrentTime();



  mb.client();

  if (mb.isConnected(remote))
  {
    modbusConnect = "Connected!";
  }
  else
  {
    mb.connect(remote, port);          // Try to connect if no connection
    Serial.println("connecting to Modbus");
    server.handleClient();
  }
  Serial.println("Modbus Connected");
  handleModbusData(false);


  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();


}

void loop() {
  switch (switchVar1) {

    case 1:
      Serial.println( "sending link ready 1" );
      for (int i = 0; i < 5; i++)
      {
        sendLinkReady1();
        delay(100);
        if (RS485Serial.available() >= 18)
        {
          getSlaveData();

          slaveId[0] = messageFromTwcSlave[3];
          slaveId[1] = messageFromTwcSlave[4];
          heartBeatMessageStayAlive[5] = slaveId[0];
          heartBeatMessageStayAlive[6] = slaveId[1];
          switchVar1 = 3;
          i = 6;
          return;
        }
      }
      Serial.println( "sending link ready 2" );
      for (int i = 0; i < 5; i++)
      {
        sendLinkReady2();
        delay(100);
        if (RS485Serial.available() >= 18)
        {
          getSlaveData();

          slaveId[0] = messageFromTwcSlave[3];
          slaveId[1] = messageFromTwcSlave[4];
          heartBeatMessageStayAlive[5] = slaveId[0];
          heartBeatMessageStayAlive[6] = slaveId[1];
          switchVar1 = 3;
          i = 6;
          return;
        }
      }
      switchVar1 = 2;

      break;

    case 2:
      // receive from Twc
      if (RS485Serial.available() >= 18)
      {
        getSlaveData();

        slaveId[0] = messageFromTwcSlave[3];
        slaveId[1] = messageFromTwcSlave[4];
        heartBeatMessageStayAlive[5] = slaveId[0];
        heartBeatMessageStayAlive[6] = slaveId[1];

        switchVar1 = 3;

      }


      server.handleClient();                     // Listen for HTTP requests from clients

      break;

    case 3:
      // Send Heartbeat

      if (millis() - timeLastHeartbeat > 1000)
      {
        Serial.println( "" );
        Serial.println( "sending master heartbeat" );
        sendMasterHeartbeat();
        timeLastHeartbeat = millis();

        switchVar1 = 5;
      }

      server.handleClient();                     // Listen for HTTP requests from clients
      break;


    case 4:
      if (millis() - timeLastHeartbeat > 1000)
      {
        if (useSolar == true)
        {
          handleModbusData(true);
        }
        else
        {
          ampsToCharge = 32;
          convertAmpsHex(ampsToCharge);
        }
        if (ampsToCharge != reportedAmpsActual) {
          Serial.println( "" );
          Serial.println( "sending master heartbeat max Current" );
          sendMasterHeartbeatMaxCurrent();
          timeLastHeartbeat = millis();
          switchVar1 = 5;
        } else {
          switchVar1 = 3;
        }
      }

      server.handleClient();                     // Listen for HTTP requests from clients
      break;

    case 5:
      if (RS485Serial.available() >= 18)
      {
        getSlaveData();

        if ((statusSlave.plInReadyCharge == true || statusSlave.startingCharge == true) && firstMaxCurrent == true)
        {
          switchVar1 = 4;

        }

        else if ((millis() - lastSolarCheck > 30000) && ( statusSlave.charging == true || statusSlave.startingCharge == true))
        {
          lastSolarCheck = millis();
          switchVar1 = 4;

        }


        else
        {
          switchVar1 = 3;
        }

      }

      if (millis() - lastModbusConnect > 30000 )
      {
        lastModbusConnect = millis();
        handleModbusData(false);
      }
      ArduinoOTA.handle();
      server.handleClient();                     // Listen for HTTP requests from clients
      break;

  }

  server.handleClient();

}

void sendMasterHeartbeat()
{
  byte checksum = 0x00;
  heartBeatMessageStayAlive[16] = 0x00;
  for (int i = 2; i < sizeof(heartBeatMessageStayAlive) - 1; i++)
  {
    checksum += heartBeatMessageStayAlive[i];
  }


  heartBeatMessageStayAlive[16] = checksum;
  checksum = 0x00;


  RS485Serial.write(heartBeatMessageStayAlive, sizeof(heartBeatMessageStayAlive));

  for (int i = 0; i < sizeof(heartBeatMessageStayAlive); i++)
  {

    Serial.print( heartBeatMessageStayAlive[i], HEX );
    Serial.print( " ");

  }
  dataFromMaster = "";
  for ( int i = 0; i < 18; i++)
  {
    dataFromMaster += String(heartBeatMessageStayAlive[i], HEX);
    dataFromMaster += " ";
  }
}

void sendMasterHeartbeatMaxCurrent()
{
  heartBeatMessageData[5] = slaveId[0];
  heartBeatMessageData[6] = slaveId[1];
  heartBeatMessageData[8] = ampsToWriteSlave[0];
  heartBeatMessageData[9] = ampsToWriteSlave[1];

  if (statusSlave.charging == true || statusSlave.startingCharge == true)
  {
    heartBeatMessageData[7] =  0x09;
  }
  else if (statusSlave.charging == false)
  {
    heartBeatMessageData[7] =  0x05;
  }


  byte checksum = 0x00;
  heartBeatMessageData[16] = 0x00;
  for (int i = 2; i < sizeof(heartBeatMessageData) - 1; i++)
  {
    checksum += heartBeatMessageData[i];
  }

  heartBeatMessageData[16] = checksum;




  RS485Serial.write(heartBeatMessageData, sizeof(heartBeatMessageData));

  for (int i = 0; i < sizeof(heartBeatMessageData); i++)
  {
    Serial.print( heartBeatMessageData[i], HEX );
    Serial.print( " ");
  }

  dataFromMaster = "";
  for ( int i = 0; i < 18; i++)
  {
    dataFromMaster += String(heartBeatMessageData[i], HEX);
    dataFromMaster += " ";
  }
}


void getSlaveData()
{

  Serial.println( " ");
  Serial.println(" data received!");

  for (int i = 0; i < sizeof(messageFromTwcSlave); i++)
  {
    messageFromTwcSlave[i] = 0x00;
  }
  RS485Serial.readBytesUntil(0xC0, messageFromTwcSlave, 16);




  int b = 1;
  while (RS485Serial.available() > 0)
  {
    messageFromTwcSlave[b] = RS485Serial.read();
    Serial.print( messageFromTwcSlave[b], HEX );
    Serial.print( " ");
    b++;
  }

  byte checksum = 0x00;
  //heartBeatMessageStayAlive[16] = 0x00;
  for (int i = 2; i < 16; i++)
  {
    checksum += messageFromTwcSlave[i];
  }


  if (checksum == messageFromTwcSlave[16])
  {

    dataFromSlave = "";
    for ( int i = 0; i < 18; i++)
    {
      dataFromSlave += String(messageFromTwcSlave[i], HEX);
      dataFromSlave += " ";
    }

    if (messageFromTwcSlave[9] == 0x00 && messageFromTwcSlave[8] == 0x00)
    {
      firstMaxCurrent = true;
    }

    if (messageFromTwcSlave[9] != 0x00 && messageFromTwcSlave[8] != 0x00)
    {
      firstMaxCurrent = false;
    }


    convertHexAmps(messageFromTwcSlave[10], messageFromTwcSlave[11]);


    statusSlave.ready = false;
    statusSlave.charging = false;
    statusSlave.error = false;
    statusSlave.pluggedIn = false;
    statusSlave.plInReadyCharge = false;
    statusSlave.startingCharge = false;

    if (messageFromTwcSlave[7] == 0x00)
    {
      statusSlave.ready = true;
      boxConnect = "Connected!";
    }
    if (messageFromTwcSlave[7] == 0x01)
    {
      statusSlave.charging = true;
      firstMaxCurrent = true;
      boxConnect = "Charging!";
    }
    if (messageFromTwcSlave[7] == 0x02)
    {
      statusSlave.error = true;
      boxConnect = "Error! " ;
      switchVar1 = 1;

    }
    if (messageFromTwcSlave[7] == 0x03)
    {
      statusSlave.pluggedIn = true;
      boxConnect = "Plugged in not chraging!";
    }
    if (messageFromTwcSlave[7] == 0x04)
    {
      statusSlave.plInReadyCharge = true;
      boxConnect = "Plugged in ready to charge!";
    }
    if (messageFromTwcSlave[7] == 0x08)
    {
      statusSlave.startingCharge = true;
      boxConnect = "Starting to charge!";
    }



  }

  else
  {
    boxConnect = "Data corrupted";
    checksumStatus++;
  }
}


void sendLinkReady1()
{
  byte linkReady1[] = {0xc0, 0xFC, 0xE1, 0x77, 0x77, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0xc0};

  RS485Serial.write(linkReady1, sizeof(linkReady1));
}

void sendLinkReady2()
{
  byte linkReady2[] = {0xc0, 0xFB, 0xE2, 0x77, 0x77, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x47, 0xc0};

  RS485Serial.write(linkReady2, sizeof(linkReady2));
}



void handleModbusData(bool dataWrite) {



  meterDataAmps[7] = reportedAmpsActual;

  uint16_t meterAdress[] = {40083, 40206, 40380, 40555};
  bool isUint[] = {true, true, true, true};
  bool isEncoded[] = {true , false, false, false};
  uint16_t mySize[] = {1, 1, 1, 1};

  for (int i = 0; i <= meter; i++) {
    delay(50);
    meterDataWatt[i] = getModbusData(meterAdress[i], isUint[i], isEncoded[i], mySize[i]);

    meterDataAmps[i] = meterDataWatt[i] / voltage;
  }

  batteryData[0] = getBatteryData(62836);
  batteryData[1] = getBatteryData(62852);

  int ampsGetSolar = 0;
  int batAmps = (int)batteryData[0] / voltage;

  if (plusOrMinus == 0 && plusOrMinus2 == 0) {
    ampsGetSolar = meterDataAmps[ampsMeterSelect1] - meterDataAmps[ampsMeterSelect2] - meterDataAmps[ampsMeterSelect3];
    solarWatts = meterDataWatt[ampsMeterSelect1] - meterDataWatt[ampsMeterSelect2] - meterDataWatt[ampsMeterSelect3];
  }

  if (plusOrMinus == 1 && plusOrMinus2 == 1) {
    ampsGetSolar = meterDataAmps[ampsMeterSelect1] + meterDataAmps[ampsMeterSelect2] + meterDataAmps[ampsMeterSelect3];
    solarWatts = meterDataWatt[ampsMeterSelect1] + meterDataWatt[ampsMeterSelect2] + meterDataWatt[ampsMeterSelect3];
  }

  if (plusOrMinus == 1 && plusOrMinus2 == 0) {
    ampsGetSolar = meterDataAmps[ampsMeterSelect1] + meterDataAmps[ampsMeterSelect2] - meterDataAmps[ampsMeterSelect3];
    solarWatts = meterDataWatt[ampsMeterSelect1] + meterDataWatt[ampsMeterSelect2] - meterDataWatt[ampsMeterSelect3];
  }

  if (plusOrMinus == 0 && plusOrMinus2 == 1) {
    ampsGetSolar = meterDataAmps[ampsMeterSelect1] - meterDataAmps[ampsMeterSelect2] + meterDataAmps[ampsMeterSelect3];
    solarWatts = meterDataWatt[ampsMeterSelect1] - meterDataWatt[ampsMeterSelect2] + meterDataWatt[ampsMeterSelect3];
  }
  solarAmps = ampsGetSolar;
  if (ampsMeterSelect3 == 9 && batAmps < 0) {
    solarAmps -= batAmps;
  }


  if (ampsGetSolar < minAmpsToCharge)
  {
    ampsGetSolar = minAmpsToCharge;
  }
  else if (ampsGetSolar > 32)
  {
    ampsGetSolar = 32;
  }




  if (dataWrite)
  {
    ampsToCharge = ampsGetSolar;
    convertAmpsHex(ampsToCharge);
    Serial.println("charging with:");
    Serial.println(ampsToCharge);
  }


}

int getModbusData(int registerr, bool isUint, bool isEncoded, uint16_t mysize)
{
  Serial.println("REG:" + String(registerr));
  const uint16_t REG = registerr;
  uint16_t dataFromBus = 0;
  const uint8_t unit2 = unit;
  uint16_t id = 0;
  const uint16_t size = mysize;

  if (mb.isConnected(remote)) {

    modbusConnect = "Connected!";
    id = mb.readHreg(remote, REG, &dataFromBus, size, NULL, unit2); // Initiate Read Coil from Modbus Slave

    while (mb.isTransaction(id)) // Check if transaction is active
    {
      mb.task();
      delay(10);

    }
  }

  else
  {
    mb.connect(remote, port);          // Try to connect if no connection
    Serial.println("connecting to MODBUS");
    modbusConnect = "Error!";
  }

  mb.task();
  if (isUint) {
    for (int i = 0; i < 16; i++)
    {
      bool memory;
      memory = bitRead(dataFromBus, i);
      bitWrite(dataConvertToInt, i, memory);
      Serial.print(memory);
    }
  }
  else {
    for (int i = 0; i < 16; i++)
    {
      bool memory;
      memory = bitRead(dataFromBus, i);
      bitWrite(dataConvertToInt, 15 - i, memory);
      Serial.print(memory);
    }

  }

  if (isEncoded) {
    dataConvertToInt = (int)dataConvertToInt * pow(10, getModbusData(REG + 1, true, false, 1));
  } else {

  }

  Serial.println("received Modbus Data");
  Serial.print(dataConvertToInt);
  Serial.println("W");
  return dataConvertToInt;

}

float getBatteryData(uint16_t reg) {
  uint32_t combinedData = 0;
  uint32_t combinedDataTurn = 0;
  uint16_t dataFromBus[2];
  const uint16_t size = 2;
  uint16_t id = 0;
  uint16_t REG = reg;

  if (mb.isConnected(remote)) {

    modbusConnect = "Connected!";

    id = mb.readHreg(remote, REG, dataFromBus, size, NULL, unit); // Initiate Read Coil from Modbus Slave

    while (mb.isTransaction(id)) // Check if transaction is active
    {
      mb.task();
      delay(10);

    }
  }
  else
  {
    mb.connect(remote, port);          // Try to connect if no connection
    Serial.println("connecting to MODBUS");
    modbusConnect = "Error!";
  }
  mb.task();

  for (int i = 0; i < 16; i++)
  {
    bool memory;
    memory = bitRead(dataFromBus[0], i);
    bitWrite(combinedData, i, memory);

    bool memory2;
    memory2 = bitRead(dataFromBus[1], i);
    bitWrite(combinedData, i + 16, memory2);

  }
  Serial.println("Databits");



  for (int i = 0; i < 32; i++) {

    bool mem = bitRead(combinedData, i);
    bitWrite(combinedDataTurn, 31 - i, mem);
  }

  for (int i = 0; i < 32; i++) {

    Serial.print(bitRead(combinedDataTurn, i));
  }
  Serial.println("");

  byte byte1 = 0;
  byte byte2 = 0;
  byte byte3 = 0;
  byte byte4 = 0;

  for (int i = 0; i < 8; i++)
  {
    bool memory1;
    memory1 = bitRead(combinedDataTurn, i);
    bitWrite(byte1, 7 - i, memory1);

    bool memory2;
    memory2 = bitRead(combinedDataTurn, i + 8);
    bitWrite(byte2, 7 - i, memory2);

    bool memory3;
    memory3 = bitRead(combinedDataTurn, i + 16);
    bitWrite(byte3, 7 - i, memory3);

    bool memory4;
    memory4 = bitRead(combinedDataTurn, i + 24);
    bitWrite(byte4, 7 - i, memory4);
  }




  float f;
  byte b[] = {byte4, byte3, byte2, byte1};
  memcpy(&f, &b, sizeof(f));

  return f;


}

void convertAmpsHex(int amps)
{
  int hundretamps = amps * 100;
  ampsToWriteSlave[0] = (hundretamps >> 8) & 0xFF;
  ampsToWriteSlave[1] = hundretamps & 0xFF;

}

void convertHexAmps(byte firstbyte, byte secondbyte)
{
  reportedAmpsActual = ((firstbyte << 8) + secondbyte) / 100;
}


void handleRoot() {
  if ( server.hasArg("D7") )
  {
    Serial.println("settings");
    server.send ( 200, "text/html", Settings() );
  }
  else if ( server.hasArg("exit") )
  {
    Serial.println("exit");
    server.send ( 200, "text/html", getPage() );
  }
  else if ( server.hasArg("save") ) {
    Serial.println("save settings");
    handlesave();
    server.send ( 200, "text/html", getPage() );
  }
  else if ( server.hasArg("refresh") ) {
    handleModbusData(false);
    server.send ( 200, "text/html", getPage() );
  }
  else if ( server.hasArg("D5") ) {
    Serial.println("on");
    buttonState = "On";
    useSolar = true;
    handleModbusData(true);
    buttonTwo = "rgb(245,245,245)";
    buttonOne = "rgb(10,177,26)";
    switchVar1 = 4;
    server.send ( 200, "text/html", getPage() );
  }
  else if ( server.hasArg("D6") )
  {
    Serial.println("off");
    buttonState = "Off";
    useSolar = false;
    ampsToCharge = 32;
    handleModbusData(false);
    ampsToCharge = 32;
    buttonOne = "rgb(245,245,245)";
    buttonTwo = "rgb(210,52,52)";
    switchVar1 = 4;
    server.send ( 200, "text/html", getPage() );
  }

  else {
    server.send ( 200, "text/html", getPage() );
  }
}

String getPage() {
  String page = "<html>";

  page +=  "<head>";
  page +=  " <meta name='viewport' content='width = device - width, initial - scale = 1.0, shrink - to - fit = no'>";
  page += "<link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css'<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.1/jquery.min.js'></script><script src='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js'></script>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += " <meta charset='utf - 8'>";
  page += " <meta Cache-Control: max-age=180>";
  page +=    " <title>TWC</title>";

  page +=    "</head>";

  page +=    "<body style='padding-left:10px;'>";
  page +=   " <div></div>";
  page +=   "<h1 style='color: rgb(6, 6, 232); '>TWC SolarEdge</h1>";
  page += "<div class='btn - group' role='group'> <form action='/' method='POST'><button type='button submit' name='D5' value='1' button class='btn btn - primary' type='button' style='background:" + buttonOne + "; '>ON</button><button type='button submit' name='D6' value='1' button class='btn btn - primary' type='button' style='background:" + buttonTwo + "; '>OFF</button>";
  page += " </div>";


  page += " <p>Charging with:";
  page += "<strong>";
  page += String(reportedAmpsActual);
  page += " Amps";
  page += "</strong>";
  page += "</p>";

  page += " <p>Max Current:";
  page += "<strong>";
  page += String(ampsToCharge);
  page += " Amps";
  page += "</strong>";

  page += " <p>Meter Current:";
  page += "<strong>";
  page += String(solarAmps);
  page += " Amps";
  page += "</strong>";

  page += " <p>Meter Watts:";
  page += "<strong>";
  page += String(solarWatts);
  page += " W";
  page += "</strong>";

  page += " <p>Battery Watts:";
  page += "<strong>";
  page += String(batteryData[0]);
  page += " W";
  page += "</strong>";

  page += " <p>Battery SOC:";
  page += "<strong>";
  page += String(batteryData[1]);
  page += " %";
  page += "</strong>";

  page += " <p>Twc status:";
  page += "<strong>";
  page += boxConnect + String(messageFromTwcSlave[7], HEX);
  page += "</strong>";

  page += " <p>Failed transmitions:";
  page += "<strong>";
  page += String(checksumStatus);
  page += "</strong>";

  page += " <p>Data slave:";
  page += "<strong>";
  page += dataFromSlave;
  page += "</strong>";


  page += " <p>Data master:";
  page += "<strong>";
  page += dataFromMaster;
  page += "</strong>";


  page += " <p>Modbus status:";
  page += "<strong>";
  page += modbusConnect;
  page += "</strong>";
  page += "</p>";


  page += " <p>";
  page += "<form action='/' method='POST'><button type='button submit' name='D7' value='1' button class='btn btn - primary' type='button' style='background: rgb(10, 177, 26); '>Settings</button>";
  page += "</p>";


  page += " <p>";
  page += "<form action='/' method='POST'><button type='button submit' name='refresh' value='1' button class='btn btn - primary' type='button' style='background: rgb(255,165,0); '>Refresh</button>";
  page += "</p>";


  page +=  "</body>";
  page +=  "</html>";

  return page;
}

String Settings() {

  String page = "<html>";

  page += "<head>";
  page +=  " <meta name='viewport' content='width = device - width, initial - scale = 1.0, shrink - to - fit = no'>";
  page += "<link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css'<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.1/jquery.min.js'></script><script src='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js'></script>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += " <meta charset='utf - 8'>";
  page +=   "<title>Settings</title>";

  page += "</head>";

  page += "<body style='padding-left:10px;'>";
  page += " <h1 style='color: rgb(0,124,248);'>Settings</h1>";
  page += "<form action='/' method='POST'>";
  page += " <p>SSID:<input type='text' name='SSID1' placeholder='" + String(esid) + "'></p>";
  page += " <p>Password:<input type='password' name='password1' placeholder='password'></p>";
  page += " <p>IP-Adress inverter:<input type='text' name='ipAdress1' placeholder='" + String(ip1) + "'style='width:36px;'><input type='text' name='ipAdress2' placeholder='" + String(ip2) + "'style='width:36px;'><input type='text' name='ipAdress3' placeholder='" + String(ip3) + "'style='width:36px;'><input type='text' name='ipAdress4' placeholder='" + String(ip4) + "'style='width:36px;'></p>";
  page += " <p>Port:<input type='text' name='port1' placeholder='" + String(port) + "' style='width:36px;' ></p>";
  page += "<p>Modbus unit:<select name='unit'><option ></option><option value='1'>Adress 1</option><option value='2'>Adress 2</option><option value='3'>Adress 3</option><option value='4'>Adress 4</option></select></p>";
  page += "<p>Meter amount: <select name='meterSelect'><option ></option><option value='1'>inverter+1</option><option value='2'>inverter+2</option><option value='3'>inverter+3</option><option value='4'>inverter+3+Battery</option></select></p>";

  page += " <p>Voltage <input type='text' name='voltageInput' placeholder='" + String(voltage) + "'></p>";

  page += "<p>Amps calculation: <select name='ampsMeterSelect1'><option ></option><option value='0'>Inverter Power</option><option value='1'>Meter 1</option><option value='2'>Meter 2</option><option value='3'>Meter 3</option></select>";
  page += "<select name='plusOrMinus'><option ></option><option value='0'>-</option><option value='1'>+</option></select>";
  page += " <select name='ampsMeterSelect2'><option ></option><option value='0'>Inverter Power</option><option value='1'>Meter 1</option><option value='2'>Meter 2</option><option value='3'>Meter 3</option><option value='8'>Charger Amps</option><option value='9'>nothing</option></select>";
  page += "<select name='plusOrMinus2'><option ></option><option value='0'>-</option><option value='1'>+</option></select>";
  page += " <select name='ampsMeterSelect3'><option ></option><option value='0'>Inverter Power</option><option value='1'>Meter 1</option><option value='2'>Meter 2</option><option value='3'>Meter 3</option><option value='7'>Charger Amps</option><option value='8'>Battery Amps</option><option value='9'>nothing</option></select>";

  page += "</p>";
  page += "<p >Min Current: ";
  page += "<label id='currentView'>";
  page += minAmpsToCharge;
  page += "</label>";
  page += "</p>";
  page += "<input type='range' name='currentSlider' min='6' max='12' step='1' style='width:200px;' value= '";
  page += String(minAmpsToCharge);
  page += "' oninput='document.getElementById(\"currentView\").innerHTML=this.value'><output></output>";


  page += "</p>";
  page += "<p><input class='btn btn-primary' type='submit' name='save' value='Save'  style='background: rgb(32,234,76);'></button><button class='btn btn-primary' type='button submit' name='exit' value='1' style='background: rgb(225,31,31);'>Exit</button></p>";
  page += "</form>";
  page += "</body>";

  page += "</html>";
  return page;
}

void handlesave()
{
  EEPROM.begin(600);
  //if ( server.arg("SSID1") == NULL || server.arg("password1") == NULL || server.arg("API-Key1") == NULL || server.arg("Plantnumber1") == NULL)



  String currentslider = server.arg("currentSlider");

  int minAmps = currentslider.toInt();
  if (minAmps != minAmpsToCharge)
  {
    for (int i = 100; i < 115; ++i) {
      EEPROM.write(i, 0);
    }
    EEPROM.put(100, minAmps);
    Serial.print(minAmps);
    Serial.print("writing min Amps");
    minAmpsToCharge = minAmps;
    lastSolarCheck -= 200000;
  }


  if ( server.arg("SSID1") != NULL && server.arg("password1") != NULL )
  {
    String ssid = server.arg("SSID1") ;
    String password = server.arg("password1") ;


    Serial.println("clearing eeprom");
    for (int i = 0; i < 96; ++i) {
      EEPROM.write(i, 0);
    }



    Serial.println("writing eeprom ssid:");
    for (int i = 0; i < ssid.length(); ++i)
    {
      EEPROM.write(i, ssid[i]);
      Serial.print("Wrote: ");
      Serial.println(ssid[i]);
    }

    Serial.println("writing eeprom pass:");
    for (int i = 0; i < password.length(); ++i)
    {
      EEPROM.write(32 + i, password[i]);
      Serial.print("Wrote: ");
      Serial.println(password[i]);
    }

  }



  if ( (server.arg("ipAdress1") != NULL && server.arg("ipAdress2") != NULL && server.arg("ipAdress3") != NULL && server.arg("ipAdress4") != NULL) || server.arg("port1") != NULL)
  {
    for (int i = 300; i < 400; ++i) {
      EEPROM.write(i, 0);
    }



    String ipAdress1 = server.arg("ipAdress1") ;
    String ipAdress2 = server.arg("ipAdress2") ;
    String ipAdress3 = server.arg("ipAdress3") ;
    String ipAdress4 = server.arg("ipAdress4") ;

    String port = server.arg("port1") ;

    Serial.println("writing eeprom ipAdress1:");
    for (int i = 0; i < ipAdress1.length(); ++i)
    {
      EEPROM.write(300 + i, ipAdress1[i]);
      Serial.print("Wrote: ");
      Serial.println(ipAdress1[i]);
    }
    Serial.println("writing eeprom ipAdress2:");
    for (int i = 0; i < ipAdress2.length(); ++i)
    {
      EEPROM.write(315 + i, ipAdress2[i]);
      Serial.print("Wrote: ");
      Serial.println(ipAdress2[i]);
    }
    Serial.println("writing eeprom ipAdress3:");
    for (int i = 0; i < ipAdress3.length(); ++i)
    {
      EEPROM.write(330 + i, ipAdress3[i]);
      Serial.print("Wrote: ");
      Serial.println(ipAdress3[i]);
    }
    Serial.println("writing eeprom ipAdress4:");
    for (int i = 0; i < ipAdress4.length(); ++i)
    {
      EEPROM.write(345 + i, ipAdress4[i]);
      Serial.print("Wrote: ");
      Serial.println(ipAdress4[i]);
    }


    Serial.println("writing eeprom plantnumber:");
    for (int i = 0; i < port.length(); ++i)
    {
      EEPROM.write(370 + i, port[i]);
      Serial.print("Wrote: ");
      Serial.println(port[i]);
    }
  }

  if (server.arg("meterSelect") != NULL) {
    for (int i = 445; i < 460; ++i) {
      EEPROM.write(i, 0);
    }
    String meter1 = server.arg("meterSelect") ;
    meter = meter1.toInt();
    Serial.println("writing eeprom Meter:");
    for (int i = 0; i < meter1.length(); ++i)
    {
      EEPROM.write(445 + i, meter1[i]);
      Serial.print("Wrote: ");
      Serial.println(meter1[i]);
    }
  }

  if (server.arg("unit") != NULL) {
    for (int i = 460; i < 470; ++i) {
      EEPROM.write(i, 0);
    }
    String unitWrite = server.arg("unit") ;
    unit = unitWrite.toInt();
    Serial.println("writing eeprom Meter:");
    for (int i = 0; i < unitWrite.length(); ++i)
    {
      EEPROM.write(460 + i, unitWrite[i]);
      Serial.print("Wrote: ");
      Serial.println(unitWrite[i]);
    }
  }

  if (server.arg("voltageInput") != NULL) {
    for (int i = 470; i < 480; ++i) {
      EEPROM.write(i, 0);
    }
    String voltageInputWrite = server.arg("voltageInput") ;
    voltage = voltageInputWrite.toInt();
    Serial.println("writing eeprom Meter:");

    for (int i = 0; i < voltageInputWrite.length(); ++i)
    {
      EEPROM.write(470 + i, voltageInputWrite[i]);
      Serial.print("Wrote: ");
      Serial.println(voltageInputWrite[i]);
    }
  }

  if (server.arg("ampsMeterSelect1") != NULL) {
    for (int i = 480; i < 490; ++i) {
      EEPROM.write(i, 0);
    }
    String ampsMeterSelect1write = server.arg("ampsMeterSelect1") ;
    ampsMeterSelect1 = ampsMeterSelect1write.toInt();
    Serial.println("writing eeprom Meterselect1:");
    for (int i = 0; i < ampsMeterSelect1write.length(); ++i)
    {
      EEPROM.write(480 + i, ampsMeterSelect1write[i]);
      Serial.print("Wrote: ");
      Serial.println(ampsMeterSelect1write[i]);
    }
  }

  if (server.arg("plusOrMinus") != NULL) {
    for (int i = 490; i < 500; ++i) {
      EEPROM.write(i, 0);
    }
    String plusOrMinusWrite = server.arg("plusOrMinus") ;
    plusOrMinus = plusOrMinusWrite.toInt();
    Serial.println("writing eeprom plus or minus:");
    for (int i = 0; i < plusOrMinusWrite.length(); ++i)
    {
      EEPROM.write(490 + i, plusOrMinusWrite[i]);
      Serial.print("Wrote: ");
      Serial.println(plusOrMinusWrite[i]);
    }
  }

  if (server.arg("ampsMeterSelect2") != NULL) {
    for (int i = 500; i < 510; ++i) {
      EEPROM.write(i, 0);
    }
    String ampsMeterSelect2Write = server.arg("ampsMeterSelect2") ;
    ampsMeterSelect2 = ampsMeterSelect2Write.toInt();
    Serial.println("writing eeprom Meterselect2:");
    for (int i = 0; i < ampsMeterSelect2Write.length(); ++i)
    {
      EEPROM.write(500 + i, ampsMeterSelect2Write[i]);
      Serial.print("Wrote: ");
      Serial.println(ampsMeterSelect2Write[i]);
    }
  }


  if (server.arg("plusOrMinus2") != NULL) {
    for (int i = 510; i < 520; ++i) {
      EEPROM.write(i, 0);
    }
    String plusOrMinus2Write = server.arg("plusOrMinus2") ;
    plusOrMinus2 = plusOrMinus2Write.toInt();
    Serial.println("writing eeprom plusOrMinus2:");
    for (int i = 0; i < plusOrMinus2Write.length(); ++i)
    {
      EEPROM.write(510 + i, plusOrMinus2Write[i]);
      Serial.print("Wrote: ");
      Serial.println(plusOrMinus2Write[i]);
    }
  }

  if (server.arg("ampsMeterSelect3") != NULL) {
    for (int i = 520; i < 530; ++i) {
      EEPROM.write(i, 0);
    }
    String ampsMeterSelect3Write = server.arg("ampsMeterSelect3") ;
    ampsMeterSelect3 = ampsMeterSelect3Write.toInt();
    Serial.println("writing eeprom ampsMeterSelect3:");
    for (int i = 0; i < ampsMeterSelect3Write.length(); ++i)
    {
      EEPROM.write(520 + i, ampsMeterSelect3Write[i]);
      Serial.print("Wrote: ");
      Serial.println(ampsMeterSelect3Write[i]);
    }
  }




  EEPROM.commit();
  EEPROM.end();
  // ESP.reset();
}

void setupAP(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
  st = "<ol>";
  for (int i = 0; i < n; ++i)
  {
    // Print SSID and RSSI for each network found
    st += "<li>";
    st += WiFi.SSID(i);
    st += " (";
    st += WiFi.RSSI(i);

    st += ")";
    st += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*";
    st += "</li>";
  }
  st += "</ol>";
  delay(100);
  WiFi.softAP("TWC Manager", "");
  Serial.println("softap");

  Serial.println("over");
}
bool testWifi(void)
{
  int c = 0;
  Serial.println("Waiting for Wifi to connect");
  while ( c < 20 ) {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(500);
    Serial.print("*");
    c++;
  }
  Serial.println("");
  Serial.println("Connect timed out, opening AP");
  return false;
}
