#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

const int chipSelect = 10;
static const int RXPin = 8, TXPin = 7;
static const uint32_t GPSBaud = 9600;
static const double TARGET_LAT = 51.508131, TARGET_LON = -0.128002; //set to London

Adafruit_BMP085 bmp;
File logfile;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  bmp.begin(9600);
  Serial.print(F("Initializing SD card..."));
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    //using return will stop function here
    //slashing return will continue to print to serial without sd card present
    return;
  }
  Serial.println(F("card initialized."));
  char filename[] = "GPSLOG00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (! logfile) {
    Serial.println(F("couldnt create file"));
  }
  Serial.print(F("Logging to: ")); Serial.println(filename);
  logfile.print(F("TARGET_LAT:,")); logfile.print(TARGET_LAT, 6); logfile.print(F(",")); logfile.print(F("TARGET_LON:,")); logfile.println(TARGET_LON, 6);
  logfile.println(F("Latitude, Longitude, Date, Time, Heading, Speed, Distance to Target, Heading to Target, Temperature, Pressure, Altitude, Calculated Pressure at Sea Level, Real Altitude"));
  logfile.println(F("(deg), (deg), (DD/MM/YYYY), (HH:MM:SS), (deg), (mps), (m), (deg), (degC), (Pa), (m), (Pa), (m)"));
  logfile.flush();
  Serial.println(F("Latitude    Longitude     Date         Time    Heading  Speed    Distance Heading to Target  Temperature Pressure Altitude Calculated Pressure at Sea Level Real Altitude"));
  Serial.println(F(" (deg)        (deg)    (DD/MM/YYYY) (HH:MM:SS)  (deg)   (mps)      (m)         (deg)            (degC)     (Pa)     (m)                     (Pa)                 (m)"));
}

void loop() {
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printDateTime(gps.date, gps.time);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  //x = gps.course.deg();
  printFloat(gps.speed.mps(), gps.speed.isValid(), 6, 2);
  unsigned long distancemToTARGET =
    (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);
  printFloat(distancemToTARGET, gps.location.isValid(), 9, 2);
  double courseToTARGET =
    TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);
  printFloat(courseToTARGET, gps.location.isValid(), 7, 2);
  //y = courseToTARGET;
  logfile.print(bmp.readTemperature()); logfile.print(", ");
  logfile.print(bmp.readPressure()); logfile.print(", ");
  // Calculate altitude assuming 'standard' barometric, 1013.25 millibar = 101325 Pascal
  logfile.print(bmp.readAltitude()); logfile.print(", ");
  logfile.print(bmp.readSealevelPressure()); logfile.print(", ");
  // insert calculated sea level pressure into readAltitude function to zero the sensor
  logfile.print(bmp.readAltitude(101325)); logfile.print(", ");
  logfile.flush();
  logfile.println();
  Serial.print(bmp.readTemperature()); Serial.print(", ");
  Serial.print(bmp.readPressure()); Serial.print(", ");
  Serial.print(bmp.readAltitude()); Serial.print(", ");
  Serial.print(bmp.readSealevelPressure()); Serial.print(", ");
  Serial.print(bmp.readAltitude(101325)); Serial.print(", ");
  Serial.print(x); Serial.print(",");
  Serial.print(y, 2); Serial.print(",");
  Serial.println(x - y);
  smartDelay(1000);
}


// This custom version of delay() ensures that the gps object is being "fed"
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      logfile.print('*'); logfile.print(", ");
    Serial.print('*'); Serial.print(", ");
  }
  else {
    logfile.print(val, prec);
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      logfile.print(' '); logfile.print(", ");
    Serial.print(' '); Serial.print(", ");
  }
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    logfile.print("**********"); logfile.print(", ");
    Serial.print("**********"); Serial.print(", ");
  }
  else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.day(), d.month(), d.year());
    logfile.print(sz); logfile.print(", ");
    Serial.print(sz); Serial.print(", ");
  }
  if (!t.isValid()) {
    logfile.print("********"); logfile.print(", ");
    Serial.print("********"); Serial.print(", ");
  }
  else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour()+1, t.minute(), t.second()); //DST from march to october
    logfile.print(sz); logfile.print(", ");
    Serial.print(sz); Serial.print(", ");
  }
  smartDelay(0);
}

