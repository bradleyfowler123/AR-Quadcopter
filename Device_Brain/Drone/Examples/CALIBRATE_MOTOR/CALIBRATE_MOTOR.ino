#include <Servo.h>
#include <Wire.h>
 
Servo esc1, esc2, esc3, esc4;
int value = 0;
int throttle = 0; int zerothrottle = 0; int maxthrottle = 0;
 
void setup()
{
  
Wire.begin();
Serial.begin(9600);    // start serial at 9600 baud
delay(5000);
Serial.println("start");
esc1.attach(6);
esc2.attach(5);
esc3.attach(13);
esc4.attach(9);
zerothrottle = map(0, 0, 1023, 0, 179);
maxthrottle = map(1023, 0, 1023, 0, 179);
esc1.write(maxthrottle); esc2.write(maxthrottle);esc3.write(maxthrottle);esc4.write(maxthrottle);
Serial.println("send somthing");
while(!Serial.available()) {};
Serial.println("passed");
}
 
void loop()
{
//  esc1.writeMicroseconds(value);
  
  if(Serial.available()) { 
    value = Serial.parseInt();    // Parse an Integer from Serial
    throttle = map(value, 0, 1023, 0, 179);
    esc1.write(throttle);esc2.write(throttle);esc3.write(throttle);esc4.write(throttle);
    Serial.println("sent");
  }
}

