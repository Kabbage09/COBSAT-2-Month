#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SerialPIO.h>
#include "SdFat.h"
#include <SD.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <utility/imumaths.h>


#define SEALEVELPRESSURE_HPA (1013.25)
#define PIN_CS 17
#define SD_SPI_SPEED 50

SdFat sd;
SdFile file;
SFE_UBLOX_GNSS myGNSS;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // I2C1 (GPIO 12 and GPIO 13)
Adafruit_BMP3XX bmp; //BMP390 used

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
const int chipSelect = 10;
int buzzerPin = 15;

String packet;

void setup() {

//LED setup
  //initialize digital pin LED_BUILTIN as an output.
  pinMode(15, OUTPUT);


//Buzzer setup
  //Set the buzzer pin as an output
  pinMode(buzzerPin, OUTPUT);


//XBEE Setup
  Serial.begin(9600);
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(9600);

  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.setSDA(4); // SDA on GPIO 4
  Wire.setSCL(5); // SCL on GPIO 5
  Wire.begin();


//ZOE-M8Q Setup
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR


  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);


  if (!sd.begin(PIN_CS, SD_SCK_MHZ(SD_SPI_SPEED)))
  {
    Serial.println("SD fail to initialize.");
    Serial.println("Try increasing?");
    while (1) {}
  }


//microSD setup
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt");

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }


//Gyro Setup
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}


  if (!file.open("Example1.TXT", O_WRITE | O_CREAT))
  {
    Serial.println("Write failed");
    while (1) {}
  }


//BNO055 Gyro Embedded Setup
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  //Initialise the sensor
  if (!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  
}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);

  packet = "1,";
  packet += String(event.orientation.x) + ",";
  packet += String(event.orientation.y) + ",";
  packet += String(event.orientation.z) + ",";
  long latitude = myGNSS.getLatitude();lP

  long longitude = myGNSS.getLongitude();
  packet += String(latitude) + ",";
  packet += String(longitude) + ",";
  packet += String(bmp.temperature) + ",";
  packet += String(bmp.pressure / 100.0) + ",";
  packet += String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + ",";

//ZOE-M8Q loop
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }

  Serial.println(packet);
  Serial1.println(packet);
  file.print(packet);
  delay(1000);


//XBEE loop
  Serial.println("Hello");
  //Serial1.println("XBEE");
  delay(1000);
  //while(Serial1.available()){
  //   Serial.print(Serial1.read());
  //}
  
}


