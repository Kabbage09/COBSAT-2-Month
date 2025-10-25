#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Servo.h>

#define XBEE_RX 13
#define XBEE_TX 12
#define SERVO_PIN 28
#define SENSOR_SDA 0
#define SENSOR_SCL 1


//Initialize Variables
int TEAM_ID = 1;
unsigned long MISSION_TIME;
int PACKET_COUNT = 0;
String SW_STATE = "Launch";
char PAYLOAD_STATE = 'N';
float ALTITUDE = 0;
float TEMP = 0;
float VOLTAGE = 0;
long int GPS_LATITUDE = 0;
long int GPS_LONGITUDE = 0;
float GYRO_R = 0;
float GYRO_P = 0;
float GYRO_Y = 0;

//Define classes
SFE_UBLOX_GNSS ZOE_GPS;
Servo servo1;
Adafruit_BMP3XX bmp;

float reference_press;

void setup() {
  //Servo Setup
  Serial.begin(9600);
  servo1.attach(SERVO_PIN);


  //XBEE Setup
  Serial1.setRX(XBEE_RX);
  Serial1.setTX(XBEE_TX);
  Serial1.begin(9600);


  //Wire Setup
  Wire.setSDA(SENSOR_SDA);
  Wire.setSCL(SENSOR_SCL);
  Wire.begin();


  //Initialize BMP Sensors
  if(!bmp.begin_I2C()){
    Serial.println("Could not find a valid BMP sensor. Check Wiring!");
  }

  //set up oversampling & filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  bmp.performReading();
  float refYay = bmp.pressure/100.0;
  float reference_press = (refYay);


  //ZOE GPS Setup
  if(!ZOE_GPS.begin(Wire)){
    Serial.println("GPS not found!");
  }
}

void getData(){
  GPS_LATITUDE = ZOE_GPS.getLatitude();
  GPS_LONGITUDE = ZOE_GPS.getLongitude();
  //BMP Data
  TEMP = bmp.readTemperature();
  ALTITUDE = bmp.readAltitude(reference_press);

  //Gyro
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  GYRO_R = event->orientation.x;
  GYRO_P = event->orientation.y;
  GYRO_Y = event->orientation.z;
}

//void flyStates(){

//}

void loop() {
  //Servo Release
  getData();
  if(ALTITUDE >= 530){
    for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo1.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
    // for (int pos = 240; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    //   servo1.write(pos);              // tell servo to go to position in variable 'pos'
    //   delay(15);                       // waits 15 ms for the servo to reach the position
    }

    //XBEE Packets
    Serial1.print(TEAM_ID);
    Serial1.print(", ");
    Serial1.print(MISSION_TIME);
    Serial1.print(", ");
    Serial1.print(PACKET_COUNT);
    Serial1.print(", ");
    Serial1.print(SW_STATE);
    Serial1.print(", ");
    Serial1.print(PAYLOAD_STATE);
    Serial1.print(", ");
    Serial1.print(ALTITUDE);
    Serial1.print(", ");
    Serial1.print(TEMP);
    Serial1.print(", ");
    Serial1.print(VOLTAGE);
    Serial1.print(", ");
    Serial1.print(GPS_LATITUDE);
    Serial1.print(", ");
    Serial1.print(GPS_LONGITUDE);
    Serial1.print(", ");
    Serial1.print(GYRO_R);
    Serial1.print(", ");
    Serial1.print(GYRO_P);
    Serial1.print(", ");
    Serial1.println(GYRO_Y);
     delay(300);
    while(Serial1.available()){
      Serial.print(Serial1.read());
    }

    //if(state == LANDING)
    //digitalWrite(15, HIGH);

}
