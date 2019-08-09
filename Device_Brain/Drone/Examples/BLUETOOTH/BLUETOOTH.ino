#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(10, 11);                          // RX, TX for arduino
Servo esc;
int ledpin=13;
String BluetoothData;
int rawData;

void setup() {
  bluetoothSerial.begin(9600);
  bluetoothSerial.println("Bluetooth On please press 1 or 0 blink LED ..");
  pinMode(ledpin,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,INPUT);
  pinMode(11,OUTPUT);
  esc.attach(9);
  esc.write(0);
}

void loop() {
   if (bluetoothSerial.available()){
      BluetoothData = "";
      while(bluetoothSerial.available()>0){                                                                                              // until data has been interated through
        rawData=bluetoothSerial.read();
        BluetoothData = BluetoothData + (char) rawData;                                                                         // append data to Bluetoothvalue
    }
    
  //    int ans = atoi(BluetoothData.c_str());
  //    esc.write(ans);
    bluetoothSerial.println(BluetoothData);
      if(BluetoothData=="1"){
          digitalWrite(ledpin,1);
          bluetoothSerial.println("LED  On D13 ON ! ");
          }
      if (BluetoothData=="0"){
          digitalWrite(ledpin,0);
           bluetoothSerial.println("LED  On D13 Off ! ");
          }
      }
  delay(100);
}
