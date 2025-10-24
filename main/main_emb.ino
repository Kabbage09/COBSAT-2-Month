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

Servo myservo;  // create Servo object to control a serv
int pos = 0;    // variable to store the servo position

String packet;

void setup() {

//LED setup
  //initialize digital pin LED_BUILTIN as an output.
  pinMode(15, OUTPUT);


//Buzzer setup
  //Set the buzzer pin as an output
  pinMode(buzzerPin, OUTPUT);


//Servo setup
myservo.attach(15);  // attaches the servo on pin 9 to the Servo object


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
  

  if (!file.open("Example1.TXT", O_WRITE | O_CREAT))
  {
    Serial.println("Write failed");
    while (1) {}
  }
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


//Gyro Loop
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);


//Servo loop
  for (pos = 0; pos <= 240; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 240; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }


//LED loop
  digitalWrite(15, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(15, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second


//Buzzer loop
  // Turn the buzzer on
  digitalWrite(buzzerPin, HIGH);
  delay(500); // Wait for 500 milliseconds (0.5 seconds)

  // Turn the buzzer off
  digitalWrite(buzzerPin, LOW);
  delay(500); // Wait for 500 milliseconds (0.5 seconds)
}


//Gyro Print
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}