
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
  int byteH, byteL;  // low and high bytes (reading is 16bit int)

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

static void wakeful_sleep(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      fix = gps.read();
  } while (millis() - start < ms);
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

  //millisec1 = millis();

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
  //Serial.println(results.tilt_deg);

  // at gain level of +-245 deg/sec 1 bit is 8.75 mdps/LSB
  //  see same datasheet as gravity above page 15
  results.Gyro_x = reading_xGy * 8.75 / 1000.0;
  results.Gyro_y = reading_yGy * 8.75 / 1000.0;
  results.Gyro_z = reading_zGy * 8.75 / 1000.0;

  return results;
}