#include <ModbusMaster.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <avr/wdt.h>

#define USE_LCDI2C false

#define MAX485_DE_AC      48
#define MAX485_RE_NEG_AC  49
#define MAX485_DE_DC      47
#define MAX485_RE_NEG_DC  46

#define n_Ac 2
#define n_Dc 6
#define n_AcMax n_Ac+1
#define n_DcMax n_Dc+1

uint8_t nCntAC, nCntDC;

// instantiate ModbusMaster object
ModbusMaster node_AC[n_AcMax];
ModbusMaster node_DC[n_DcMax];

char *acStr[] = {"AC0", "AC1", "AC2", "AC3"};
char *dcStr[] = {"DC0", "DC1", "DC2", "DC3", "DC4", "DC5", "DC6", "DC7"};

float voltage_AC[n_AcMax];
float current_AC[n_AcMax];
float power_AC[n_AcMax];
float energy_AC[n_AcMax];
float frequency_AC[n_AcMax];
float powerfactor_AC[n_AcMax];
float alarm_AC[n_AcMax];
bool measureUpdate_AC[n_AcMax];

float voltage_DC[n_DcMax];
float current_DC[n_DcMax];
float power_DC[n_DcMax];
float energy_DC[n_DcMax];
float frequency_DC[n_DcMax];
float powerfactor_DC[n_DcMax];
float alarm_DC[n_DcMax];
bool measureUpdate_DC[n_DcMax];

#if USE_LCDI2C
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

HardwareSerial &Serial_AC = Serial1;
HardwareSerial &Serial_DC = Serial2;
HardwareSerial &Serial_Bluetooth = Serial3;

String inputString0 = "";         // a String to hold incoming data
boolean stringComplete0 = false;  // whether the string is complete
String inputString3 = "";         // a String to hold incoming data
boolean stringComplete3 = false;  // whether the string is complete

unsigned long currentMillis = 0;
unsigned long previousOneSecMillis = 0;
unsigned long previousMeasureMillis_AC = 0;
unsigned long previousMeasureMillis_DC = 0;
unsigned long previousReportMillis = 0;
const long oneSecInterval = 1000;
const long oneMinInterval = 60;
const long measureInterval_AC = 100;
const long measureInterval_DC = 100;
const long reportInterval = 3000;//3 sec

uint16_t recv_data[10];
uint16_t slaveID;

uint8_t secCnt = 0;

//{"AC2":{"V":224.2,"I":0,"P":0,"E":699,"F":50,"PF":0,"A":0}
//,"DC5":{"V":17.67,"I":0.002,"P":0.3,"E":18,"F":0,"PF":0,"A":0}}

void preTransmission_AC()
{
  digitalWrite(MAX485_RE_NEG_AC, 1);
  digitalWrite(MAX485_DE_AC, 1);
}
void postTransmission_AC()
{
  digitalWrite(MAX485_RE_NEG_AC, 0);
  digitalWrite(MAX485_DE_AC, 0);
}

void preTransmission_DC()
{
  digitalWrite(MAX485_RE_NEG_DC, 1);
  digitalWrite(MAX485_DE_DC, 1);
}
void postTransmission_DC()
{
  digitalWrite(MAX485_RE_NEG_DC, 0);
  digitalWrite(MAX485_DE_DC, 0);
}

void setup()
{
  wdt_disable();
  Serial.begin(57600, SERIAL_8N1);// use Serial (port 0); RPI communication
  while (!Serial) continue;
  Serial_AC.begin(9600, SERIAL_8N1);// use Serial (port 1); initialize Modbus AC
  Serial_DC.begin(9600, SERIAL_8N2);// use Serial (port 2, 2 stopbits); initialize Modbus DC
  Serial_Bluetooth.begin(9600, SERIAL_8N1);// use Serial (port 3); Bluetooth communication

  pinMode(MAX485_RE_NEG_AC, OUTPUT);
  pinMode(MAX485_DE_AC, OUTPUT);
  pinMode(MAX485_RE_NEG_DC, OUTPUT);
  pinMode(MAX485_DE_DC, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG_AC, 0);
  digitalWrite(MAX485_DE_AC, 0);
  digitalWrite(MAX485_RE_NEG_DC, 0);
  digitalWrite(MAX485_DE_DC, 0);


  checkDevice();

  //  result = node_DC[5].writeSingleRegister(3,2);//set current range(2,200A)
  //  Serial.print("set current range ");
  //  Serial.println(result);

#if USE_LCDI2C
  lcd.begin();
  lcd.setCursor(0, 0);
  lcd.print("UNICORN PROJECT");
  lcd.setCursor(0, 1);
  lcd.print("EGAT");
#endif

  wdt_enable(WDTO_8S);
}

void loop()
{
  uint8_t j, result;

  currentMillis = millis();
  if (currentMillis - previousOneSecMillis >= oneSecInterval) {
    previousOneSecMillis = currentMillis;
    wdt_reset();

    secCnt++;
  }

  if (secCnt >= oneMinInterval) { //every minute
    secCnt = 0;
    wdt_reset();

    statusDisplay();
  }

  currentMillis = millis();
  if (currentMillis - previousMeasureMillis_AC >= measureInterval_AC) {
    previousMeasureMillis_AC = currentMillis;
    wdt_reset();

    //nCntAC = 2;

    nCntAC++;
    if (nCntAC > n_Ac) {
      nCntAC = 1;
    }

    // read (8) 16-bit registers starting at 0 to RX buffer
    result = node_AC[nCntAC].readInputRegisters(0, 10);
    wdt_reset();

    // do something with data if read is successful
    if (result == node_AC[nCntAC].ku8MBSuccess)
    {
      for (j = 0; j < 10; j++)
      {
        recv_data[j] = node_AC[nCntAC].getResponseBuffer(j);
      }

      voltage_AC[nCntAC] = (float)recv_data[0] / 10;
      current_AC[nCntAC] = (float)((unsigned int)recv_data[2] << 8 | recv_data[1]) / 1000;
      power_AC[nCntAC] = (float)((unsigned int)recv_data[4] << 8 | recv_data[3]) / 10;
      energy_AC[nCntAC] = (float)((unsigned int)recv_data[6] << 8 | recv_data[5]);
      frequency_AC[nCntAC] = (float)recv_data[7] / 10;
      powerfactor_AC[nCntAC] = (float)recv_data[8] / 100;
      alarm_AC[nCntAC] = (float)recv_data[9];
      measureUpdate_AC[nCntAC] = true;
    } else {
#if USE_LCDI2C
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("node_AC[?] error"));
    }
#endif
  }

  currentMillis = millis();
  if (currentMillis - previousMeasureMillis_DC >= measureInterval_DC) {
    previousMeasureMillis_DC = currentMillis;
    wdt_reset();

    //nCntDC = 5;

    nCntDC++;
    if (nCntDC > n_Dc) {
      nCntDC = 1;
    }

    // read (8) 16-bit registers starting at 0 to RX buffer
    result = node_DC[nCntDC].readInputRegisters(0, 8);
    wdt_reset();

    // do something with data if read is successful
    if (result == node_DC[nCntDC].ku8MBSuccess)
    {
      for (j = 0; j < 8; j++)
      {
        recv_data[j] = node_DC[nCntDC].getResponseBuffer(j);
      }

      voltage_DC[nCntDC] = (float)recv_data[0] / 100;
      current_DC[nCntDC] = (float)recv_data[1] / 100;
      power_DC[nCntDC] = (float)((unsigned int)recv_data[3] << 8 | recv_data[2]) / 10;
      energy_DC[nCntDC] = (float)((unsigned int)recv_data[5] << 8 | recv_data[4]);
      alarm_DC[nCntDC] = (float)((unsigned int)recv_data[7] << 8 | recv_data[6]);
      measureUpdate_DC[nCntDC] = true;
    } else {
      #if USE_LCDI2C
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("node_DC[?] error"));
      #endif
    }
  }

  if (measureUpdate_AC[2]) {
    //calculate operating state and security check
  }
  if (measureUpdate_DC[5]) {
    //calculate operating state and security check
  }

  currentMillis = millis();
  if (currentMillis - previousReportMillis >= reportInterval) {
    previousReportMillis = currentMillis;
    wdt_reset();

    reportMeasuredData();
  }


  if (stringComplete0) {//JSON parser package from RPI
    StaticJsonDocument<120> jsonDoc_recv;

    wdt_reset();
    
    DeserializationError error = deserializeJson(jsonDoc_recv, inputString0);
    if (error) {
      #if USE_LCDI2C
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("parseObject() failed"));
      lcd.setCursor(0, 1);
      lcd.print(error.c_str());
      #endif
      //      if (inputString0.startsWith("h")) {
      //        Serial.println("no help");
      //      }
      Serial3.println(inputString0);
      inputString0 = "";
      stringComplete0 = false;
      return;
    }

    //    voltage_AC[2] = jsonDoc_recv["V"];
    //    current_AC[2] = jsonDoc_recv["I"];
    //    power_AC[2] = jsonDoc_recv["P"];
    //    energy_AC[2] = jsonDoc_recv["E"];
    //    frequency_AC[2] = jsonDoc_recv["F"];
    //    powerfactor_AC[2] = jsonDoc_recv["PF"];
    //    alarm_AC[2] = jsonDoc_recv["A"];

    // Print values.
    //serializeJson(jsonDoc_recv, Serial);
    //Serial.println();


    // clear the string:
    inputString0 = "";
    stringComplete0 = false;
  }
  if (stringComplete3) {//JSON parser package from RPI
    uint8_t n1, n2;
    String n1s, n2s;

    wdt_reset();
    if (inputString3.length() == 6) {
      n1s = inputString3.substring(1, 3);
      n2s = inputString3.substring(3, 5);
      n1 = n1s.toInt();
      n2 = n2s.toInt();

      if (inputString3.charAt(0) == 'D') {
        result = node_DC[n1].writeSingleRegister(2, n2); //set device id dc
        if (result == 0) {
          Serial_Bluetooth.println(F("Ok"));
        } else {
          Serial_Bluetooth.println(F("Fail"));
        }
      }

      if (inputString3.charAt(0) == 'A') {
        result = node_AC[n1].writeSingleRegister(2, n2); //set device id ac
        if (result == 0) {
          Serial_Bluetooth.println(F("Ok"));
        } else {
          Serial_Bluetooth.println(F("Fail"));
        }
      }


    } else if (inputString3 == "checkId#") {
      checkDevice();
    }

    // clear the string:
    inputString3 = "";
    stringComplete3 = false;
  }

  wdt_reset();
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString0:
    inputString0 += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete0 = true;
    }
  }
}
//void serialEvent1() {
//  while (Serial1.available()) {
//    // get the new byte:
//    char inChar = (char)Serial1.read();
//    // add it to the inputString1:
//    inputString1 += inChar;
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n') {
//      stringComplete1 = true;
//    }
//  }
//}
//void serialEvent2() {
//  while (Serial2.available()) {
//    // get the new byte:
//    char inChar = (char)Serial2.read();
//    // add it to the inputString2:
//    inputString2 += inChar;
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n') {
//      stringComplete2 = true;
//    }
//  }
//}
void serialEvent3() {
  while (Serial3.available()) {
    // get the new byte:
    char inChar = (char)Serial3.read();
    // add it to the inputString3:
    inputString3 += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '#') {
      stringComplete3 = true;
    }
  }
}
void checkDevice(void) {
  uint8_t j, result;
  // communicate with Modbus slaves ID  over Serial
  for (j = 1; j < n_AcMax; j++)
  {
    wdt_reset();
    node_AC[j].begin(j, Serial_AC);
    node_AC[j].preTransmission(preTransmission_AC);
    node_AC[j].postTransmission(postTransmission_AC);

    result = node_AC[j].readInputRegisters(0, 10);
    checkResultAC(result, j);
  }

  // communicate with Modbus slaves ID  over Serial
  for (j = 1; j < n_DcMax; j++)
  {
    wdt_reset();
    node_DC[j].begin(j, Serial_DC);
    node_DC[j].preTransmission(preTransmission_DC);
    node_DC[j].postTransmission(postTransmission_DC);

    result = node_DC[j].readInputRegisters(0, 8);
    checkResultDC(result, j);
  }
}
void checkResultAC(uint8_t result, uint8_t j) {
  Serial.print(F("AC"));
  Serial.print(j);

  if (result == node_AC[j].ku8MBSuccess)
  {
    Serial.println(F(" Success"));
    Serial_Bluetooth.print(F("AC"));
    Serial_Bluetooth.print(j);
    Serial_Bluetooth.println(F(" Success"));
  } else if (result == node_AC[j].ku8MBInvalidSlaveID) {
    Serial.println(F(" InvalidSlaveID"));
  } else if (result == node_AC[j].ku8MBInvalidFunction) {
    Serial.println(F(" InvalidFunction"));
  } else if (result == node_AC[j].ku8MBResponseTimedOut) {
    Serial.println(F(" ResponseTimedOut"));
  } else if (result == node_AC[j].ku8MBInvalidCRC) {
    Serial.println(F(" InvalidCRC"));
  } else if (result == node_AC[j].ku8MBIllegalFunction) {
    Serial.println(F(" IllegalFunction"));
  } else if (result == node_AC[j].ku8MBIllegalDataAddress) {
    Serial.println(F(" IllegalDataAddress"));
  } else if (result == node_AC[j].ku8MBIllegalDataValue) {
    Serial.println(F(" IllegalDataValue"));
  } else if (result == node_AC[j].ku8MBSlaveDeviceFailure) {
    Serial.println(F(" SlaveDeviceFailure"));
  } else {
    Serial.println(F(" unknown"));
  }
}
void checkResultDC(uint8_t result, uint8_t j) {
  Serial.print(F("DC"));
  Serial.print(j);

  if (result == node_DC[j].ku8MBSuccess)
  {
    Serial.println(F(" Success"));
    Serial_Bluetooth.print(F("DC"));
    Serial_Bluetooth.print(j);
    Serial_Bluetooth.println(F(" Success"));
  } else if (result == node_DC[j].ku8MBInvalidSlaveID) {
    Serial.println(F(" InvalidSlaveID"));
  } else if (result == node_DC[j].ku8MBInvalidFunction) {
    Serial.println(F(" InvalidFunction"));
  } else if (result == node_DC[j].ku8MBResponseTimedOut) {
    Serial.println(F(" ResponseTimedOut"));
  } else if (result == node_DC[j].ku8MBInvalidCRC) {
    Serial.println(F(" InvalidCRC"));
  } else if (result == node_DC[j].ku8MBIllegalFunction) {
    Serial.println(F(" IllegalFunction"));
  } else if (result == node_DC[j].ku8MBIllegalDataAddress) {
    Serial.println(F(" IllegalDataAddress"));
  } else if (result == node_DC[j].ku8MBIllegalDataValue) {
    Serial.println(F(" IllegalDataValue"));
  } else if (result == node_DC[j].ku8MBSlaveDeviceFailure) {
    Serial.println(F(" SlaveDeviceFailure"));
  } else {
    Serial.println(F(" unknown"));
  }
}
void statusDisplay(void) {
  Serial_Bluetooth.println("[]");
}

void reportMeasuredData(void) {
  uint8_t j;
  StaticJsonDocument<900> jsonDoc_send;
  JsonObject root = jsonDoc_send.to<JsonObject>();

  wdt_reset();
  for (j = 1; j < n_AcMax; j++)
  {
    JsonObject mData = root.createNestedObject(acStr[j]);
    mData["V"] = voltage_AC[j];
    mData["I"] = current_AC[j];
    mData["P"] = power_AC[j];
    mData["E"] = energy_AC[j];
    mData["F"] = frequency_AC[j];
    mData["PF"] = powerfactor_AC[j];
    mData["A"] = alarm_AC[j];
    wdt_reset();
  }

  for (j = 1; j < n_DcMax; j++)
  {
    JsonObject mData = root.createNestedObject(dcStr[j]);
    mData["V"] = voltage_DC[j];
    mData["I"] = current_DC[j];
    mData["P"] = power_DC[j];
    mData["E"] = energy_DC[j];
    mData["F"] = frequency_DC[j];
    mData["PF"] = powerfactor_DC[j];
    mData["A"] = alarm_DC[j];
    wdt_reset();
  }


  serializeJson(jsonDoc_send, Serial);
  Serial.println();
  wdt_reset();
  serializeJson(jsonDoc_send, Serial_Bluetooth);
  Serial_Bluetooth.println();
  wdt_reset();
}
