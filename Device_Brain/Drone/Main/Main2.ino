//---------------------------------------------------------------LIBARIES---------------------------------------------------------------------------------
#include <SoftwareSerial.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//----------------------------------------------------------FUNCTION DECLARATIONS-------------------------------------------------------------------------
int commandCheck();
void executeCommand();
void instructMotors();
void setupMpu();
void readMpu();
void dmpDataReady();
int PID(float reference, float value, float diff_value, int pidGroup, float &errorIntegral);                                                                                                               
String UIinput(String message);
//-------------------------------------------------------------GLOBAL VARIABLES----------------------------------------------------------------------------
Servo esc[4];                                                                                                                    // back right, front right, back left, front left
int rawData, valueInt, btInts[4], cmd, space = 0, refAltitude, altitude, yawDataOld, yawRefOld;                                  // int variable to store: BT data, BT number, BT items, desired altitude, actual altitude
int baseThrottle, baseThrottleMap, throttle[4], throttleMap[4], yprAdjust[3];                                                    // throttle variables, controller 1d throttle adjustments         
float yprRef[3], yprIntegral[3], altIntegral;                                                                                    // ypr ypr reference values used for movement, ypr integral of error, PID gains[kyaw, kpitch, kroll, kheight](prop, derr, int)               
String btData[4] = {"","","",""}, bluetoothValue;                                                                                // string variables to store BT data
//const float k[4][3] =  {{0,0,0},{1,0,0},{1,0,0},{0,0,0}};                                                                // yaw 1 0 0, pitch,roll,z
float k[4][3] =   {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};  

                                                                 // mpu GVs 
MPU6050 mpu;                                                                                                                     // accel&gyro asigned to variable mpu
Quaternion q;                                                                                                                    // [w, x, y, z]                    quaternion container
VectorInt16 aa;                                                                                                                  // [x, y, z]                       accel sensor measurements
VectorInt16 aaReal;                                                                                                              // [x, y, z]                       gravity-free accel sensor measurements
//VectorInt16 aaWorld;                                                                                                           // [x, y, z]                       world-frame accel sensor measurements
VectorInt16 gyro;                                                                                                                // [yawDot, pitchDot, rollDot]     angular velocities
VectorFloat gravity;                                                                                                             // [x, y, z]                       gravity vector
//float euler[3];                                                                                                                // [psi, theta, phi]               Euler angle container
float yprData[3];                                                                                                                // [yaw, pitch, roll]              yaw/pitch/roll container and gravity vector
  //MOVE TO within library
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from 
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//---------------------------------------------------------------SETUP------------------------------------------------------------------------------------
void setup() {
                                                              // BT setup
  Serial1.begin(9600);                                                                                                           // setup bluetooth coms
  Serial.begin(9600);                                                                                                            // setup USB coms
                                                            // motor setup
  esc[0].attach(13);   esc[1].attach(6);   esc[2].attach(5);    esc[3].attach(9);                                                // asign escs' to correct pins: back right, front right, back left, front left
  throttle[0] = 0;    throttle[1] = 0;    throttle[2] = 0;      throttle[3]=0;    baseThrottle = 0;                              // set all throttle variables initally to zero
  baseThrottleMap = map(baseThrottle, 0, 1023, 0, 179);                                                                          // map throttle value to value sero will understand
  esc[0].write(baseThrottleMap); esc[1].write(baseThrottleMap); esc[2].write(baseThrottleMap); esc[3].write(baseThrottleMap);    // turn all motors off
  Serial1.println("Please wait while motors initalise...");                                                                      // inform user to wait while motors initialise
  delay(3000);                                                                                                                   // time to allow motor set up
                                                             // mpu setup  
  setupMpu();
                                                             // PID setup
  yprRef[0]=0;   yprRef[1]=0;    yprRef[2]=0;    yprIntegral[0]=0;    yprIntegral[1]=0;    yprIntegral[2]=0;    altIntegral=0;   // set direction of motion to hover, initialise all integrals to zero
                                                            // altitude setup
 refAltitude = 0;
 Serial1.println("Setup Complete");
  float pkp = atof(UIinput("Type in pitch kp value:").c_str());                                                                         // inorder to calibrate gains 
  float pkd = atof(UIinput("Type in pitch kd value:").c_str());
  float pki = atof(UIinput("Type in pitch ki value:").c_str());
  float rkp = atof(UIinput("Type in roll kp value:").c_str());                                                                         // inorder to calibrate gains 
  float rkd = atof(UIinput("Type in roll kd value:").c_str());
  float rki = atof(UIinput("Type in roll ki value:").c_str());
  float ykp = atof(UIinput("Type in yaw kp value:").c_str());                                                                         // inorder to calibrate gains 
  float ykd = atof(UIinput("Type in yaw kd value:").c_str());
  float yki = atof(UIinput("Type in yaw ki value:").c_str());
    
  k[1][0]=pkp; k[1][1]=pkd; k[1][2]=pki;  k[2][0]=rkp; k[2][1]=rkd; k[2][2]=rki;  k[0][0]=ykp; k[0][1]=ykd; k[0][2]=yki;              // yaw 1 0 0, pitch,roll,z
}
//---------------------------------------------------------------LOOP-------------------------------------------------------------------------------------
void loop() {
                                                            // BT command 
  cmd = commandCheck();                                                                                                          // check for new bluetooth message
  if(cmd == 1) {                                                                                                                 // if one exists
    executeCommand();                                                                                                            // execute it
 //   Serial.print("Height: "),Serial.print(btData[0]),Serial.print("   Pitch: "),Serial.print(btData[1]),Serial.print("    Yaw: "),Serial.print(btData[2]),Serial.print("    Roll: "),Serial.println(btData[3]);
  }
                                                    // main altitude control programme 
  altitude = analogRead(A0);                                                                                                     // record altitude
  readMpu();                                                                                                                     // record all gyro&accel data
//  Serial.print("yawRef: ");Serial.print(yprRef[0]);Serial.print("   pitchRef: "); Serial.print(yprRef[1]);Serial.print("    rollRef: "); Serial.print(yprRef[2]); Serial.print("    Altref: "); Serial.println(refAltitude);
//  Serial.print("yaw: ");Serial.print(yprData[0]);Serial.print("   pitch: "); Serial.print(yprData[1]);Serial.print("    roll: "); Serial.print(yprData[2]); Serial.print("    Alt: "); Serial.println(altitude);
  baseThrottle = PID(float(refAltitude), float(altitude), aaReal.z, 3, altIntegral);                                            // calculate uniform adjustmeant to be made to maintain height
  baseThrottle = refAltitude;                                                                                                    // temporary
  yprAdjust[1] = PID(yprRef[1],yprData[1]*180/M_PI,gyro.y, 1, yprIntegral[1]);                                                  // calculate adjustmeants to be made to eliminate pitch error
  yprAdjust[2] = PID(yprRef[2],yprData[2]*180/M_PI,gyro.x, 2, yprIntegral[2]);                                                  // roll
    float yawRef, ans = yprRef[0]-yprData[0]*180/M_PI;                                                                          // handle yaw discontinuity at 180/-180 degrees. Note that once error in position angle passes abs(180 degrees), motor switch to other direction as that now is the cloest route
    if(ans < -180.0) {yawRef = yprRef[0] + 360.0;}                                                                              // if ref is clockwise of discontinuty with respect to actual position
    else if(ans > 180.0) {yawRef = yprRef[0] - 360.0;}                                                                          // if ref is anti-clockwise of discontinuty with respect to actual position
    else {yawRef = yprRef[0];}                                                                                                  // if ref and postion are on same side of discontinuity
  yprAdjust[0] = PID(yawRef,yprData[0]*180/M_PI,((yprData[0]*180/M_PI)-yawRef)-(yawDataOld-yawRefOld), 0, yprIntegral[0]);                                                     // yaw   
  yawDataOld = yprData[0]*180/M_PI; yawRefOld = yawRef;
  throttle[3] = - yprAdjust[1] - yprAdjust[2] - yprAdjust[0];     throttle[1] = - yprAdjust[1] + yprAdjust[2] + yprAdjust[0];   // linearly combine to give new throttle values that will remove error
  throttle[2] = + yprAdjust[1] - yprAdjust[2] + yprAdjust[0];     throttle[0] = + yprAdjust[1] + yprAdjust[2] - yprAdjust[0];
//  Serial.print(baseThrottle); Serial.print("  throt1: ");Serial.print(throttle[0]);Serial.print("   throt2: "); Serial.print(throttle[1]);Serial.print("    throt3: "); Serial.print(throttle[2]);Serial.print("    throt4: "); Serial.println(throttle[3]);
  instructMotors();                                                                                                              // turn motors
}

//-----------------------------------------------------------OTHER FUNCTIONS-------------------------------------------------------------------------------

//----------------------------------------------------------BT COMMAND CHECK-------------------------------------------------------------------------------
int commandCheck() {
  btData[0] = "";  btData[1] = "";  btData[2] = "";  btData[3] = "";                                                             // reset relavent variables
  int item = -1;                                                                                                                 // handle initial comma
  if(Serial1.available()<8) {return 0;}                                                                                          // return 0 if no bluetooth data of minimum length
  else {
    while(item < 4) {                                                                                                            // continue until all items have been recieved
      while(Serial1.available()>0){                                                                                              // while there is data for the given item
        rawData=Serial1.read();                                                                                                  // read bluetooth data                                                                                                          // if no space has occurred yet
        if(rawData == 44){                                                                                                       // if that data is a comma
          item += 1;                                                                                                             // move onto next item
          }                                                                                              
        else if(item >= 0) {                                                                                                     // else 
          btData[item] = btData[item] + (char) rawData;                                                                          // append data to current item
          }
      }
    }
    return 1;                                                                                                                    // return 1 and bluetooth data
  }
}
//-----------------------------------------------------------EXECUTE BT COMMAND----------------------------------------------------------------------------
void executeCommand() {
  for (int i=0;i<=4;i++){                                                                                                        // height,forward,rotate,left
    btInts[i] = atoi(btData[i].c_str());                                                                                         // convert value from string to int
    } 
  refAltitude = btInts[0];                                                                                                       // set altitude reference               
  yprRef[0] = btInts[2];      yprRef[1] = btInts[1];      yprRef[2] = btInts[3];                                                 // set yaw, pitch, roll referenes
}
//-----------------------------------------------------------INSTRUCT MOTORS-------------------------------------------------------------------------------
void instructMotors() {
    throttleMap[0] = map(baseThrottle + throttle[0], 0, 1023, 0, 179);                                                           // map all throttle values to values the servo understands
    throttleMap[1] = map(baseThrottle + throttle[1], 0, 1023, 0, 179);
    throttleMap[2] = map(baseThrottle + throttle[2], 0, 1023, 0, 179);
    throttleMap[3] = map(baseThrottle + throttle[3], 0, 1023, 0, 179);
//    Serial.print("  throt1: ");Serial.print(throttleMap[0]);Serial.print("   throt2: "); Serial.print(throttleMap[1]);Serial.print("    throt3: "); Serial.print(throttleMap[2]);Serial.print("    throt4: "); Serial.println(throttleMap[3]);
    esc[0].write(throttleMap[0]); esc[1].write(throttleMap[1]); esc[2].write(throttleMap[2]); esc[3].write(throttleMap[3]);      // turn motors by appropiate amounts
}
//-----------------------------------------------------------PID CONTROLLER--------------------------------------------------------------------------------
int PID(float reference, float value, float diff_value, int pidGroup, float &errorIntegral) {
    if(!isnan(value)){                                                                                                           // sort recurring error on first call
      float result;
      errorIntegral += (reference*0.01 - value)*0.1;                                                                             // calculate the error in integral
      result = k[pidGroup][0]*(reference - value) - k[pidGroup][1]*diff_value + k[pidGroup][2]*errorIntegral;                                                  // calculate error in position and velocity, then sum all errors up times by an appropiate constant
      return (int) result;                                                                                                       // return result as a integer
    }
    else{return (int) 0;}
}
//------------------------------------------------------------GET USER INPUT-------------------------------------------------------------------------------
String UIinput(String message){
  Serial1.println(message);                                                                                                      // inform user a message
  while(Serial1.available()==0) {}                                                                                               // wait until user enters data
  bluetoothValue = "";                                                                                                           // reset data container
  while(Serial1.available()>0){                                                                                                  // until data has been interated through
      rawData=Serial1.read();
      bluetoothValue = bluetoothValue + (char) rawData;                                                                          // append data to Bluetoothvalue
      delay(10);                                                                                                                 // allow time for next character to arrive before exiting
  }
  return bluetoothValue;                                                                                                         // return bluetooth data
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------END------------------------------------------------------------------------------------


// MOVE TO LIBRARY
void setupMpu(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial1.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(96);
  mpu.setYGyroOffset(-19);
  mpu.setZGyroOffset(26);
  mpu.setXAccelOffset(-1482);
  mpu.setYAccelOffset(499);
  mpu.setZAccelOffset(939);

  if (devStatus == 0) {
    Serial1.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial1.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(7, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial1.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    Serial1.println("ERROR");
  }
}
// SAME HERE
void readMpu() {
  if (!dmpReady) return;   // if programming failed, don't try to do anything
  mpuInterrupt = true;
  while (!mpuInterrupt && fifoCount < packetSize) {} // wait for MPU interrupt or extra packet(s) available
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount(); // get current FIFO count
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check for overflow (this should never happen unless our code is too inefficient)
     mpu.resetFIFO();         // reset so we can continue cleanly
     Serial1.println(F("FIFO overflow!"));
     } 
  else if (mpuIntStatus & 0x02) {     // otherwise, check for DMP data ready interrupt (this should happen frequently)
     while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();         // wait for correct available data length, should be a VERY short wait
     mpu.getFIFOBytes(fifoBuffer, packetSize);         // read a packet from FIFO
     fifoCount -= packetSize;         // track FIFO count here in case there is > 1 packet available         // (this lets us immediately read more without waiting for an interrupt)
    }  
    
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);         
  mpu.dmpGetYawPitchRoll(yprData, &q, &gravity);            // yaw, pitch, roll
  mpu.dmpGetAccel(&aa, fifoBuffer);                   // aceleration of object in cartesian coords
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);        // aceleration of object in cartesian coords minus gravity
//  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);  // literally the acceleration in x,y,z of the world. x=y=0, z=-9.81
  mpu.dmpGetGyro(&gyro, fifoBuffer);                    // angular velocities
}
// AND HERE
void dmpDataReady() {
    mpuInterrupt = true;
}

