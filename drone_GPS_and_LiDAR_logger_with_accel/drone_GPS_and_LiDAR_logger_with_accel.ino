// Program to talk to GlobalSat EM-506 GPS and lightware SF-11 laser rangefinder
// and log to SparkFun SDcard board.
// HB 14 June 2016     (added IMU code 19 May 2017)
// Update by Jordan Lerma @jordkl
// 12 May 2021

//Uncomment all of the definitions of the NMEA_PARSE_XXX in NMEAGPS.cfg.h in NMEA GPS Library

#include <SPI.h>
#include <Wire.h>    // Wire.h provides I2C bus
#include <NMEAGPS.h>
#include <LSM6.h>
#include "SD.h"


// Define pins and sensors
#define I2C_ADR  0x55
#define SPI_CS   10
#define RX_LED   17
LSM6 imu;

// The Objects
NMEAGPS gps;
gps_fix fix;
File logfile;
#define NUMSAMPLES 100 

//Write out to the SD Card every 60 seconds
unsigned long prev_millisec = 0;
long interval = 60000;

void setup(void) {
  //Start the Machines
  Serial.begin(9600);
  delay(1000);
  Serial1.begin(9600);
  delay(500);
  Wire.begin();
  delay(500);
  SD.begin(SPI_CS);
  delay(500);

  if (!imu.init()) {
    Serial.println("IMU Error");
  }
  imu.enableDefault();
  delay(500);
  fix = gps.read();

  // Create a new file
  char filename[] = "LOG_0000.CSV";
  for (uint16_t i = 0; i < 10000; i++) {
      filename[4] = i / 1000 + '0';
      filename[5] = (i % 1000) / 100 + '0';
      filename[6] = (i % 100) / 10 + '0';
      filename[7] = i % 10 + '0';
      if (! SD.exists(filename)) {
          logfile = SD.open(filename, FILE_WRITE);
          break;  // leave the loop!
      }
  }
  delay(1000);
  // Initialize the File 
  logfile.println(F("# GPS and Laser Rangefinder logging with Pro Micro Arduino 3.3v"));
  logfile.println(F("# units:  accel=1g  gyro=deg/sec"));
  logfile.print(F("#gmt_date\tgmt_time\tnum_sats\tlongitude\tlatitude\t"));
  logfile.print(F("gps_altitude_m\tSOG_kt\tCOG\tHDOP\tlaser_altitude_cm\t"));
  logfile.println(F("tilt_deg\taccel_x\taccel_y\taccel_z\tgyro_x\tgyro_y\tgyro_z"));
  logfile.flush();
  
  // Tell the boys we ready to fly
  Serial.println("Setup Complete");
}

// Add some functions

// IMU Object
struct imu_data {
    float Grav_x;
    float Grav_y;
    float Grav_z;
    float Gyro_x;
    float Gyro_y;
    float Gyro_z;
    float tilt_deg;
};


// Functions
void write_gps_date() {
  if (fix.valid.date) {
    logfile.print(fix.dateTime.year);
    logfile.print(F("/"));
    if (fix.dateTime.month < 10)
      logfile.print(F("0"));
    logfile.print(fix.dateTime.month);
    logfile.print(F("/"));
    if (fix.dateTime.date < 10)
      logfile.print(F("0"));
    logfile.print(fix.dateTime.date);
  }
  else {
    logfile.print(F("INVALID"));
  }
}

void write_gps_time() {
  if (fix.valid.time) {
    if (fix.dateTime.hours < 10)
      logfile.print(F("0"));
    logfile.print(fix.dateTime.hours);
    logfile.print(F(":"));
    if (fix.dateTime.minutes < 10)
      logfile.print(F("0"));
    logfile.print(fix.dateTime.minutes);
    logfile.print(F(":"));
    if (fix.dateTime.seconds < 10)
      logfile.print(F("0"));
    logfile.print(fix.dateTime.seconds);
  }
  else {
    logfile.print(F("INVALID"));
  }
}

void write_lidar_alt() {
  int distance_cm;
  int byteH, byteL; 

  ////// get laser value (centimeters) //////
  // get 2 bytes from the SF-11 range finder
  Wire.requestFrom(I2C_ADR, 2);
  if (Wire.available() >= 2) {
    byteH = Wire.read();
    byteL = Wire.read();
    // combine in big endian order
    distance_cm = byteH * 256 + byteL;
    //Serial.print("\tlidar (cm): ");
    logfile.print(distance_cm);
  }
  else
    logfile.print(F("NaN"));
}

void write_imu_data(struct imu_data imu_results) {
  logfile.print(imu_results.Grav_x, 4);
  logfile.print(F("\t"));
  logfile.print(imu_results.Grav_y, 4);
  logfile.print(F("\t"));
  logfile.print(imu_results.Grav_z, 4);

  logfile.print(F("\t"));

  logfile.print(imu_results.Gyro_x, 3);
  logfile.print(F("\t"));
  logfile.print(imu_results.Gyro_y, 3);
  logfile.print(F("\t"));
  logfile.print(imu_results.Gyro_z, 3);
}

void lock_and_blink() {
  while (1) {   //lock it up, blinking forever
    digitalWrite(RX_LED, HIGH);
    TXLED0;
    delay(500);
    digitalWrite(RX_LED, LOW);
    TXLED1;
    delay(500);
  }
}

struct imu_data get_IMU_readings() {
  struct imu_data results;

  int i;
  long sample_sum_xAc, sample_sum_yAc, sample_sum_zAc;
  long sample_sum_xGy, sample_sum_yGy, sample_sum_zGy;
  float reading_xAc, reading_yAc, reading_zAc;
  float reading_xGy, reading_yGy, reading_zGy;
  float Grav_x, Grav_y, Grav_z, Gyro_x, Gyro_y, Gyro_z;
  float horiz_mag;
  //unsigned long millisec1;

  sample_sum_xAc = 0;
  sample_sum_yAc = 0;
  sample_sum_zAc = 0;
  sample_sum_xGy = 0;
  sample_sum_yGy = 0;
  sample_sum_zGy = 0;

  digitalWrite(RX_LED, LOW);   // set the Rx LED on
  for (i = 0; i < NUMSAMPLES; i++) {
    imu.read();
    sample_sum_xAc += imu.a.x;
    sample_sum_yAc += imu.a.y;
    sample_sum_zAc += imu.a.z;
    sample_sum_xGy += imu.g.x;
    sample_sum_yGy += imu.g.y;
    sample_sum_zGy += imu.g.z;
  }
  digitalWrite(RX_LED, HIGH);   // set the Rx LED off

  reading_xAc = (float)sample_sum_xAc / NUMSAMPLES;
  reading_yAc = (float)sample_sum_yAc / NUMSAMPLES;
  reading_zAc = (float)sample_sum_zAc / NUMSAMPLES;
  reading_xGy = (float)sample_sum_xGy / NUMSAMPLES;
  reading_yGy = (float)sample_sum_yGy / NUMSAMPLES;
  reading_zGy = (float)sample_sum_zGy / NUMSAMPLES;

  results.Grav_x = reading_xAc * 0.061 / 1000.0;
  results.Grav_y = reading_yAc * 0.061 / 1000.0;
  results.Grav_z = reading_zAc * 0.061 / 1000.0;

  horiz_mag = sqrt(results.Grav_x*results.Grav_x + results.Grav_y*results.Grav_y);
  results.tilt_deg = abs(atan(horiz_mag / results.Grav_z) * 180./M_PI);
  results.Gyro_x = reading_xGy * 8.75 / 1000.0;
  results.Gyro_y = reading_yGy * 8.75 / 1000.0;
  results.Gyro_z = reading_zGy * 8.75 / 1000.0;

  return results;
}


void loop(void)
{
  while (gps.available( Serial1)) {
        fix = gps.read();
        
        // Get IMU data
        struct imu_data imu_results;
        imu_results = get_IMU_readings();
       
      write_gps_date();
      logfile.print(F("\t"));
      write_gps_time(); 
      logfile.print(F("\t"));
      logfile.print(fix.satellites);
      logfile.print(F("\t"));
          
    if (fix.valid.location) {
        logfile.print(fix.longitude());
        logfile.print(F("\t"));
        logfile.print(fix.latitude());
    }
    else {
        logfile.print(F("NaN\tNaN"));
    }
    logfile.print(F("\t"));

    if (fix.valid.altitude) {
        logfile.print(fix.alt.whole);
    }
    else {
        logfile.print(F("NaN"));
    }
    logfile.print(F("\t"));
    if (fix.valid.speed) {
        logfile.print(fix.speed_kph());
    }
    else {
        logfile.print(F("NaN"));
    }
    logfile.print(F("\t"));
    if (fix.valid.heading) {
        logfile.print(fix.heading_cd());
    }
    else {
        logfile.print(F("NaN"));
    }
    logfile.print(F("\t"));
    if (fix.valid.hdop) {
        logfile.print(fix.hdop);
    }
    else {
      logfile.print(F("NaN"));
    }
    logfile.print(F("\t"));

    // lidar
    write_lidar_alt();        
    logfile.print(F("\t"));
    
    
    // IMU
    logfile.print(imu_results.tilt_deg, 2);
    logfile.print(F("\t"));
    write_imu_data(imu_results);  // grav x,y,z then gyro x,y,z
    logfile.println();

     TXLED0;    // Tx LED off
    
    unsigned long current_millisec;
    current_millisec = millis();
    if(current_millisec - prev_millisec > interval) {
        prev_millisec = current_millisec;
        
        digitalWrite(RX_LED, LOW);   // set the Rx LED on
        logfile.flush();
        digitalWrite(RX_LED, HIGH);   // set the Rx LED off
        Serial.println("writing to card");
    }
  }
}