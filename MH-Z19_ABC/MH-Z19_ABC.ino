#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#define pwmPin 11
#define PIN_NOT_WAKE 5
#define CCS811_ADDR 0x5A //Alternate I2C Address

//Stuff for Button Reading using the Bounce Library
#include <Bounce.h>
const int buttonPin = 8;
Bounce pushbutton = Bounce(buttonPin, 10);  // 10 ms debounce

byte previousState = HIGH;         // what state was the button last time
unsigned int count = 0;            // how many times has it changed to low
unsigned long countAt = 0;         // when count changed
unsigned int countPrinted = 0;     // last count printed


//Global sensor objects
CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;

SoftwareSerial mySerial(9, 10); // RX, TX

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
byte setABC[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86}; //disable

byte mhzCmdMeasurementRange1000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x03,0xE8,0x7B};
byte mhzCmdMeasurementRange2000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x07,0xD0,0x8F};
byte mhzCmdMeasurementRange3000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x0B,0xB8,0xA3};
byte mhzCmdMeasurementRange5000[9] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB};
byte mhzCmdReset[9] = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72};
byte mhzCmdCalibrateZero[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
byte mhzCmdABCEnable[9] = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6}; //enable
//unsigned char response[9]; 
byte response[9];

unsigned long u = 0;
unsigned long th, tl,ppmMHZ19, s, oldppmMHZ19, tvoc, tpwm, p1, p2, ppm2, ppm3 = 0;

unsigned long ppmCCS881 = 999;
unsigned long oldppmCCS881 = 999;

float BMEtempC = 99;
float BMEhumid = 99;
float BMEpressure = 99;
unsigned long measTime = 0;
int ledPin = 13;
signed int temperatureCorrected;
int value;
unsigned long measCount = 0;
int displayFactor = 5000/48;
unsigned long thresholdPPM = 3000;

 void setup() {
  Serial.begin(9600); 
  delay(100);
  pinMode(ledPin, OUTPUT); 
  pinMode(buttonPin, INPUT_PULLUP);

  CCS811Core::status returnCode = myCCS811.begin();
  Serial.print("CCS811 begin exited with: ");
  //Pass the error code to a function to print the results
  printDriverError( returnCode );
  Serial.println();
  
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;

  //Initialize BME280
  //For I2C, enable the following and disable the SPI section
  //myBME280.settings.commInterface = I2C_MODE;
  //myBME280.settings.I2CAddress = 0x76;
  
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  //Calling .begin() causes the settings to be loaded
  delay(100);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  myBME280.begin();
  delay(500);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,25);
  display.println("AirQuality");
  display.setTextSize(1);
  display.setCursor(0,46);
  display.println("BME280 Temp/Hum");
  display.setCursor(0,56);
  display.println("CCS811 CO2 Sensor");
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("ABC");
  display.display();

  delay(1000);

  mySerial.begin(9600); 

  //Serial.println("Reseting MH-Z19");
  //mySerial.write(mhzCmdReset,9);
  delay(1000);
  Serial.println("Setting MH-Z19 to 5000");

  mySerial.write(mhzCmdMeasurementRange5000,9);
  delay(200);
  
  mySerial.readBytes(response, 9);
  unsigned int responseV1 = (unsigned int) response[0];
  unsigned int responseV2 = (unsigned int) response[1];
  unsigned int responseHigh = (unsigned int) response[2];
  unsigned int responseLow = (unsigned int) response[3];
  unsigned int temperatureRaw = (unsigned int) response[4];
  unsigned int mhzRespS = (unsigned int) response[5];
  unsigned int mhzRespUHigh = (unsigned int) response[6];
  unsigned int mhzRespULow = (unsigned int) response[7];
  unsigned int responseV9 = (unsigned int) response[7];

  Serial.println("Booting MH-Z19");
  
while (count == 0){
  //OLEDshowGraph();

  mySerial.write(cmd,9);
  delay(200);
  mySerial.readBytes(response, 9);
  unsigned int responseV1 = (unsigned int) response[0];
  unsigned int responseV2 = (unsigned int) response[1];
  unsigned int responseHigh = (unsigned int) response[2];
  unsigned int responseLow = (unsigned int) response[3];
  unsigned int temperatureRaw = (unsigned int) response[4];
  unsigned int mhzRespS = (unsigned int) response[5];
  unsigned int mhzRespUHigh = (unsigned int) response[6];
  unsigned int mhzRespULow = (unsigned int) response[7];
  unsigned int responseV9 = (unsigned int) response[7];

  temperatureRaw = (unsigned int) response[4];
  temperatureCorrected = temperatureRaw - 40;
  s = mhzRespS;
  ppmMHZ19 = (256*responseHigh)+responseLow;
  u = (256*mhzRespUHigh) + mhzRespULow;
/*
  Serial.print(responseV1, HEX);
  Serial.print(" ");
  Serial.print(responseV2, HEX);
  Serial.print(" ");
  Serial.print(responseHigh, HEX);
  Serial.print(" ");
  Serial.print(responseLow, HEX);
  Serial.print(" ");
  Serial.print(temperatureRaw, HEX);
  Serial.print(" ");
  Serial.print(mhzRespS, HEX);
  Serial.print(" ");
  Serial.print(mhzRespUHigh, HEX);
  Serial.print(" ");
  Serial.print(mhzRespULow, HEX);
  Serial.print(" ");
  Serial.print(responseV9, HEX);
  Serial.print(" -> ");
  Serial.print(ppmMHZ19);
  Serial.print(" ppm ");
  Serial.print(temperatureCorrected);
  Serial.print(" T Celsius ");
  Serial.print(u);
  Serial.print(" u Value ");
  Serial.println();
   */ 
  delay(200);

  preHeating();
}

Serial.println("disable ABC");

mySerial.write(setABC,9);
  delay(200);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("booting system");
  display.display();

  
  pinMode(pwmPin, INPUT);

    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
  
  th = pulseIn(pwmPin, HIGH, 3000000); // use microseconds
  tl = pulseIn(pwmPin, LOW, 3000000);
  tpwm = th + tl; // actual pulse width
  p1 = tpwm/502; // start pulse width
  p2 = tpwm/251; // start and end pulse width combined

OLEDdrawBackground();
}

void loop() {
  oldppmCCS881 = ppmCCS881;
  oldppmMHZ19 = ppmMHZ19;
  mySerial.write(cmd,9);
  delay(200);
  mySerial.readBytes(response, 9);
  unsigned int responseHigh = (unsigned int) response[2];
  unsigned int responseLow = (unsigned int) response[3];
  unsigned int temperatureRaw = (unsigned int) response[4];
  unsigned int mhzRespUHigh = (unsigned int) response[6];
  unsigned int mhzRespULow = (unsigned int) response[7];
  
  temperatureRaw = (unsigned int) response[4];
  temperatureCorrected = temperatureRaw - 40;
  ppmMHZ19 = (256*responseHigh)+responseLow;
  u = (256*mhzRespUHigh) + mhzRespULow;
  if (u==15000)Serial.println("usiech");
  delay(200);

  int readAnalValue = analogRead(A2) * (3300 / 1023.0);

  //Check to see if data is available
  if (myCCS811.dataAvailable())
  {
    //Calling this function updates the global tVOC and eCO2 variables
    myCCS811.readAlgorithmResults();
    ppmCCS881 = myCCS811.getCO2();
    tvoc = myCCS811.getTVOC();
    BMEtempC = myBME280.readTempC();
    BMEpressure = myBME280.readFloatPressure();
    BMEhumid = myBME280.readFloatHumidity();

    //This sends the temperature data to the CCS811
    //myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
  }
  else if (myCCS811.checkForStatusError())
  {
    //If the CCS811 found an internal error, print it.
    printSensorError();
    //uint8_t error = myCCS811.getErrorRegister();
  }
  measTime = millis() / 1000;

  //OLEDshowValues();
  OLEDshowGraph();
  
  Serial.print(measTime);
  //Serial.print(" ");
  //Serial.print(temperatureCorrected);
  //Serial.print(" ");
  //Serial.print(BMEpressure);
  //Serial.print(" ");
  //Serial.print(ppmCCS881);
  Serial.print(" ");
  Serial.println(ppmMHZ19);
  
  digitalWrite(ledPin, LOW);
  delay(2000);
  digitalWrite(ledPin, HIGH);
  measCount++;
  
  if (measCount == 128){
    measCount = 0;
    OLEDdrawBackground();
    }

}

void OLEDshowGraph()
{       display.setTextSize(2);
        display.fillRect(0, 0, 128, 34, BLACK);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.print(ppmMHZ19);
        display.println(" ppm");
        display.setCursor(0,20);
        display.print(ppmCCS881);
        display.println(" CCS");
        display.setTextSize(1);
        display.setCursor(100,0);
        display.print(temperatureCorrected);
        display.println("T");
        display.setCursor(100,9);
        display.print(measTime);
        display.println("s");
        display.drawLine(measCount-1, 64-(oldppmMHZ19/displayFactor),measCount, 64-(ppmMHZ19/displayFactor),WHITE);

        if ( ppmMHZ19 > thresholdPPM){
          if ( oldppmMHZ19 < thresholdPPM){
          display.setTextSize(1);
          display.setCursor(measCount+3,thresholdPPM/displayFactor+10);
          display.print(measTime);
          display.println("s");
           }
        }
        
        display.display();
        delay(300); 
}

void OLEDdrawBackground()
{
display.clearDisplay();
for (int i=0; i <= 128; i=i+3){
  display.drawPixel(i, 64-(400/displayFactor), WHITE);
  }
for (int i=0; i <= 128; i=i+5){
  display.drawPixel(i, 64-(1000/displayFactor), WHITE);
  }
for (int i=0; i <= 128; i=i+7){
  display.drawPixel(i, 64-(3000/displayFactor), WHITE);
  }
  display.drawRect(0, 16, 128, 48, 1); //Border of the bar chart

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(100,29);
  display.println("3000");
  display.setCursor(100,44);
  display.println("1000");
  display.setCursor(105,54);
  display.println("400");
  
  display.display();
}

//old version using drawPixel
void OLEDshowGraph2()
{       display.setTextSize(2);
        display.fillRect(0, 0, 128, 16, BLACK);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.print(ppmCCS881);
        display.println(" ppm");
        display.setTextSize(1);
        display.setCursor(9,80);
        display.print(measTime);
        display.println(" s");
        display.drawPixel(measCount, 64-(ppmCCS881>>5), WHITE);
        display.display();
        delay(300); 
}

//printDriverError decodes the CCS811Core::status type and prints the
//type of error to the serial terminal.
//
//Save the return value of any function of type CCS811Core::status, then pass
//to this function to see what the output was.
void printDriverError( CCS811Core::status errorCode )
{
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      Serial.print("SUCCESS");
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      Serial.print("ID_ERROR");
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      Serial.print("I2C_ERROR");
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      Serial.print("INTERNAL_ERROR");
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      Serial.print("GENERIC_ERROR");
      break;
    default:
      Serial.print("Unspecified error.");
  }
}

//printSensorError gets, clears, then prints the errors
//saved within the error register.
void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    Serial.println("Failed to get ERROR_ID register.");
  }
  else
  {
    Serial.print("Error: ");
    if (error & 1 << 5) Serial.print("HeaterSupply");
    if (error & 1 << 4) Serial.print("HeaterFault");
    if (error & 1 << 3) Serial.print("MaxResistance");
    if (error & 1 << 2) Serial.print("MeasModeInvalid");
    if (error & 1 << 1) Serial.print("ReadRegInvalid");
    if (error & 1 << 0) Serial.print("MsgInvalid");
    Serial.println();
  }
}

// PreHeating display screen and wait for button
void preHeating()
{
  if (millis() >150000) {
    count = count + 1;
  }
  if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      count = count + 1;
    }
  } else {
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("preheating");  
        display.setTextSize(1);
        display.setCursor(0,40);
        display.print(millis()/1000);
        display.println(" sec");
  display.display();
  delay(250);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("preheating.");
        display.setTextSize(1);
        display.setCursor(0,40);
        display.print(millis()/1000);
        display.println(" sec");
  display.display();
  delay(250);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("preheating..");
        display.setTextSize(1);
        display.setCursor(0,40);
        display.print(millis()/1000);
        display.println(" sec");
  display.display();
  delay(250);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("preheating...");
        display.setTextSize(1);
        display.setCursor(0,40);
        display.print(millis()/1000);
        display.println(" sec");
  display.display();
  delay(250);
  }
}


void OLEDshowValues()
{
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0,0);
        display.print(ppmMHZ19);
        display.println(" ppm");
        display.setTextSize(1);
        display.setCursor(0,16);
        display.print("measuring.");
        display.setTextSize(1);
        display.setCursor(64,16);
        display.print(measTime);
        display.println(" sec");
        display.setCursor(1,26);
        display.print(temperatureCorrected);
        display.println(" T Celsius");
        display.setTextSize(1);
        display.setCursor(1,36);
        display.print(ppmCCS881);
        display.println(" ppm CO2 CCS881");
        display.setCursor(1,46);
        display.print(tvoc);
        display.println(" ppb tvoc");
        display.setCursor(1,56);
        display.print(BMEtempC);
        display.println(" T C");
        display.setCursor(64,56);
        display.print(BMEhumid);
        display.println(" %Hum");
        display.display();
}
        

