#include <PID_v1.h>
#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <Servo.h>
#include <LSM303.h>

#define USE_I2C

Servo myservo_Roll;
Servo myservo_Pitch;
//Servo myservo_Yaw;

Adafruit_L3GD20 gyro;
LSM303 compass;

float alphaAccel = 0.8; //range of 0 to 1, higher will increase response and noise
float alphaMagnet = 0.4;
unsigned int xOffset = 0;
unsigned int yOffset = 0;
unsigned int zOffset = 0;
float Pitch = 0;        float Roll = 0;          float Yaw = 0;
int xRaw = 0;           int yRaw = 0;            int zRaw = 0;
float xFiltered = 0;    float yFiltered = 0;     float zFiltered = 0;
float xFilteredOld = 0; float yFilteredOld = 0;  float zFilteredOld = 0;
float xAccel = 0;       float yAccel = 0;        float zAccel = 0;

const int relaypin = 2;
const int autopilot = 5;
//const int rudder=7;
const int chipSelect = 10;
int roll = 0;
int pitch = 0;
int AutoPilot = 0;
//int Rudder=0;
//int RudderPulse=0;

int Servo_Roll;
int Servo_Pitch;
double gyro_Roll;
double gyro_Pitch;
float Kp = 0;
float Ki = 100;
float Kd = 0;
float kill_factor = 10;
double Setpoint, Input_Roll, Output_Roll, Input_Pitch, Output_Pitch, Inputr1, Inputr2;
PID myPID_Roll(&Input_Roll, &Output_Roll, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_Pitch(&Input_Pitch, &Output_Pitch, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 1;
const long serialPing = 500;
unsigned long now = 0;
unsigned long lastMessage = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  myservo_Roll.attach(9);
  myservo_Pitch.attach(8);
  //myservo_Yaw.attach(6);

  pinMode(autopilot, INPUT);
  //pinMode(rudder, INPUT);

  analogReference(EXTERNAL);
  compass.init();
  compass.enableDefault();
  getAccelOffset();
  gyro.begin(gyro.L3DS20_RANGE_2000DPS);

  Setpoint = 0;

  myPID_Roll.SetMode(AUTOMATIC);
  myPID_Pitch.SetMode(AUTOMATIC);
  myPID_Roll.SetSampleTime(sampleRate);
  myPID_Pitch.SetSampleTime(sampleRate);
  myPID_Roll.SetOutputLimits(-50, 50);
  myPID_Pitch.SetOutputLimits(-50, 50);
  Serial.println("Begin");

  lastMessage = millis();
}


void FilterAD() {
  xRaw = compass.a.x - xOffset;
  yRaw = compass.a.y - yOffset;
  zRaw = compass.a.z - zOffset;
  xFiltered = xFilteredOld + alphaAccel * (xRaw - xFilteredOld);
  yFiltered = yFilteredOld + alphaAccel * (yRaw - yFilteredOld);
  zFiltered = zFilteredOld + alphaAccel * (zRaw - zFilteredOld);
  xFilteredOld = xFiltered;
  yFilteredOld = yFiltered;
  zFilteredOld = zFiltered;
}


void AD2Degree() {
  compass.read();
  xAccel = xFiltered * 3.3 / (1023.0 * 0.8);
  yAccel = yFiltered * 3.3 / (1023.0 * 0.8);
  zAccel = zFiltered * 3.3 / (1023.0 * 0.8) + 1.0;
  Pitch   = atan2(  yAccel ,  sqrt(sq(xAccel) + sq(zAccel))) * 180 / PI;
  Roll  = atan2(  xAccel ,   sqrt(sq(yAccel) + sq(zAccel))) * 180 / PI;
}

void getAccelOffset() {
  compass.read();
  for (int i = 1; i <= 60; i++) {
    xOffset += compass.a.x;
    yOffset += compass.a.y;
    zOffset += compass.a.z;
  }
  xOffset /= 60;
  yOffset /= 60;
  zOffset /= 60;
  Serial.print("xOffset: "); Serial.print(xOffset);
  Serial.print("   yOffset: "); Serial.print(yOffset);
  Serial.print("   zOffset: "); Serial.println(zOffset);
}



void loop() {
  AutoPilot = pulseIn (autopilot, HIGH);
  //RudderPulse = pulseIn (rudder, HIGH);
  //Serial.print(" RudderPulse = "); Serial.print(RudderPulse);
  Serial.print(" AutoPilot = "); Serial.print(AutoPilot);

  compass.read();
  FilterAD();
  AD2Degree();

  if ((Roll > -1) && (Roll < 1)) {
    Input_Roll = kill_factor * Output_Roll;
    myPID_Roll.Compute();
  }
  else {
    gyro.read();
    gyro_Roll = gyro.data.x;
    Input_Roll = map(gyro_Roll, 0, 900, 0, 255);
    myPID_Roll.Compute();
  }


  if ((Pitch > -1) && (Pitch < 1)) {
    Input_Pitch = kill_factor * Output_Pitch;
    myPID_Pitch.Compute();
  }
  else {
    gyro.read();
    gyro_Pitch = gyro.data.y;
    Input_Pitch = map(gyro_Pitch, 0, 900, 0, 255);
    myPID_Pitch.Compute();
  }

  Servo_Roll = map(Output_Roll, -50, 50, 45, 135);
  myservo_Roll.write(Servo_Roll);
  Servo_Pitch = map(Output_Pitch, -50, 50, 45, 135);
  myservo_Pitch.write(Servo_Pitch);

  if (AutoPilot < 1100) {
    digitalWrite(relaypin, 0);
    Serial.print(" , Off ");
  }
  else {
    digitalWrite(relaypin, 1);
    Serial.print(" , On ");
  }

  /*if(RudderPulse > 1300){
    Rudder = map(RudderPulse, 1300, 1650, 91, 179);
  }
  else if(RudderPulse < 1270){
    Rudder = map(RudderPulse, 950, 1270, 0, 90);
  }
  else{
    Rudder = 90;
    //set to heading
  }

  myservo_Yaw.write(Rudder);*/

  now = millis();
  if (now - lastMessage > serialPing) {
    Serial.print(" Roll Input = "); Serial.print(Input_Roll);
    Serial.print(" Roll Output = "); Serial.print(Output_Roll);
    Serial.print(" Pitch Input = "); Serial.print(Input_Pitch);
    Serial.print(" Pitch Output = "); Serial.print(Output_Pitch);
    Serial.print(" Roll Servo = "); Serial.print(Servo_Roll);
    Serial.print(" Pitch Servo = "); Serial.print(Servo_Pitch);
    Serial.print(" Roll: "); Serial.print(Roll);
    Serial.print(" Pitch: "); Serial.print(Pitch);
    //Serial.print(" Rudder: "); Serial.println(Rudder);
    Serial.print("\n");
  }
}








