//---------------------------------------------------------------LIBARIES---------------------------------------------------------------------------------
#include <SoftwareSerial.h>
#include <Servo.h>
#include <I2Cdev.h>
//----------------------------------------------------------FUNCTION DECLARATIONS-------------------------------------------------------------------------
void instructMotors();
//-------------------------------------------------------------GLOBAL VARIABLES----------------------------------------------------------------------------                                                                                             // Setup RX & TX ports for bluetooth communication
Servo esc[4];                                                                                                                    // back right, front right, back left, front left
int rawData, valueInt, cmd, space = 0, refAltitude, altitude, ledpin=13;                                                         // int variable to store: BT data, BT number, BT command, BT space detector, desired altitude, actual altitude, onboard led
int baseThrottle, baseThrottleMap, throttle[4], throttleMap[4], yprAdjust[3];                                                    // throttle variables, controller 1d throttle adjustments         
float yprRef[3], yprIntegral[3], k[4][3];                                                                                        // ypr ypr reference values used for movement, ypr integral of error, PID gains[kyaw, kpitch, kroll, kheight](prop, derr, int)               
String bluetoothCommand, bluetoothValue;                                                                                         // string variables to store BT data
//---------------------------------------------------------------SETUP------------------------------------------------------------------------------------
void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
                                                                                                       // setup bluetooth
  Serial.begin(9600);                                                                                                     // setup onboard led
                                                            // motor setup
  esc[0].attach(6);   esc[1].attach(5);   esc[2].attach(13);    esc[3].attach(9);
  altitude = analogRead(A0); 
  baseThrottleMap = map(altitude, 0, 1023, 0, 179);                                                                          // map throttle value to value sero will understand
  esc[3].write(altitude); esc[1].write(0); esc[2].write(0); esc[0].write(0);    // turn all motors off                                                                  // inform user to wait while motors initialise                                                                                                              // time to allow motor set up                                                            
}
//---------------------------------------------------------------LOOP-------------------------------------------------------------------------------------
void loop() {                            // main altitude control programme 
  altitude = analogRead(A0);                                                                                                     // record altitude
  Serial.println(altitude);                                                                                                                 // record all gyro&accel data                                                                                              // temporary
  baseThrottleMap = map(altitude, 0, 1023, 0, 179);   
  esc[3].write(baseThrottleMap);// turn motors
}
