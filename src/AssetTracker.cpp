#include "AssetTracker.h"
#include "math.h"

//----------------- Tracker ----------------//

Adafruit_GPS gps = Adafruit_GPS(&Serial1);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2);

uint8_t internalANT[]={0xB5,0x62,0x06,0x13,0x04,0x00,0x00,0x00,0xF0,0x7D,0x8A,0x2A};
uint8_t externalANT[]={0xB5,0x62,0x06,0x13,0x04,0x00,0x01,0x00,0xF0,0x7D,0x8B,0x2E};

AssetTracker::AssetTracker() {

}

void AssetTracker::begin() {
    accel.begin(LIS3DH_DEFAULT_ADDRESS);

    // Default to 5kHz low-power sampling
    accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

    // Default to 4 gravities range
    accel.setRange(LIS3DH_RANGE_4_G);

    // Turn on the GPS module
    // gpsOn();
}

float AssetTracker::readLat() {
    return gps.latitude;
}

float AssetTracker::readLon() {
    return gps.longitude;
}

float AssetTracker::readLatDeg() {
    return gps.latitudeDegrees;
}

float AssetTracker::readLonDeg() {
    return gps.longitudeDegrees;
}

float AssetTracker::readHDOP() {
    return gps.HDOP;
}

float AssetTracker::getGpsAccuracy() {
  // 1.8 taken from specs at https://learn.adafruit.com/adafruit-ultimate-gps/
  return 1.8 * readHDOP();
}

uint32_t AssetTracker::getGpsTimestamp() {
  // Return timestamp in milliseconds, from last GPS reading
  // 0 if no reading has been done
  // (This returns the milliseconds of current day)
  return gps.hour * 60 * 60 * 1000 + gps.minute * 60 * 1000 + gps.seconds * 1000 + gps.milliseconds;
}

uint8_t AssetTracker::getHour(){
  return gps.hour;
}

uint8_t AssetTracker::getMinute(){
  return gps.minute;
}

uint8_t AssetTracker::getSeconds(){
  return gps.seconds;
}

uint16_t AssetTracker::getMilliseconds(){
  return gps.milliseconds;
}

uint8_t AssetTracker::getYear(){
  return gps.year;
}

uint8_t AssetTracker::getMonth(){
  return gps.month;
}

uint8_t AssetTracker::getDay(){
  return gps.day;
}

uint8_t AssetTracker::getSatellites(){
  return gps.satellites;
}

uint8_t AssetTracker::getFixQuality(){
  return gps.fixquality;
}

float AssetTracker::getSpeed(){
  return gps.speed;
}

float AssetTracker::getGeoIdHeight(){
  return gps.geoidheight;
}

float AssetTracker::getAltitude(){
  return gps.altitude;
}

String AssetTracker::readLatLon() {
    String latLon = String::format("%f,%f",gps.latitudeDegrees,gps.longitudeDegrees);
    return latLon;
}

void AssetTracker::gpsOn() {
    // Power to the GPS is controlled by a FET connected to D6
    pinMode(D6,OUTPUT);
    digitalWrite(D6,LOW);
    gps.begin(9600);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(500);
    // Default is 1 Hz update rate
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    delay(500);
    gps.sendCommand(PGCMD_NOANTENNA);
    delay(500);
    //for ublox maxm8 gps only
    //internal antenna selected as default
    for(uint8_t i=0;i<12;i++)
    {
        Serial1.write(internalANT[i]);
    }
    delay(500);
}

void AssetTracker::gpsOff() {
    digitalWrite(D6,HIGH);
}

char* AssetTracker::preNMEA() {
    return gps.lastNMEA();
}

bool AssetTracker::gpsFix() {
    if ((gps.latitude == 0.0) || (gps.longitude == 0.0)){
        return false;
    } else {
        return true;
    }
    //return gps.fix;
}

bool AssetTracker::antennaInternal(){

  for(uint8_t i=0;i<12;i++)
    {
        Serial1.write(internalANT[i]); //send the command to gps modul
    }
  return true;

}

bool AssetTracker::antennaExternal(){

  for(uint8_t i=0;i<12;i++)
    {
        Serial1.write(externalANT[i]); //send the command to gps modul
    }
  return true;

}

// char AssetTracker::checkGPS(){
//     char c = gps.read();
//     return c;
// }

void AssetTracker::updateGPS() {
  //char c = gps.read();
  // // if a sentence is received, we can check the checksum, parse it...
  // if (gps.newNMEAreceived()) {
  //   // a tricky thing here is if we print the NMEA sentence, or data
  //   // we end up not listening and catching other sentences!
  //   // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
  //   //Serial.println(gps.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  //
  //   if (!gps.parse(gps.lastNMEA())) {
  //     // this also sets the newNMEAreceived() flag to false
  //     return;  // we can fail to parse a sentence in which case we should just wait for another
  //   }
  // }
  //read the complete sentence
  while (Serial1.available()) {
        char c = gps.read();
        if (gps.newNMEAreceived()) {
            gps.parse(gps.lastNMEA());
        }
    }
}

int AssetTracker::readX() {
    accel.read();
    return accel.x;
}

int AssetTracker::readY() {
    accel.read();
    return accel.y;
}

int AssetTracker::readZ() {
    accel.read();
    return accel.z;
}

int AssetTracker::readXYZmagnitude() {
    accel.read();
    int magnitude = sqrt((accel.x*accel.x)+(accel.y*accel.y)+(accel.z*accel.z));
    return magnitude;
}

bool AssetTracker::setupLowPowerWakeMode(uint8_t movementThreshold) {
    return accel.setupLowPowerWakeMode(movementThreshold);
}

uint8_t AssetTracker::clearAccelInterrupt() {
    return accel.clearInterrupt();
}
