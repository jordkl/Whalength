// Program to talk to GlobalSat EM-506 GPS and lightware SF-11 laser rangefinder
// and log to SparkFun SDcard board.
// HB 14 June 2016     (added IMU code 19 May 2017)
// Update by Jordan Lerma @jordkl
// 12 May 2021

/*
GPS hardware:
 Hardware serial Rx,Tx are on pins 0,1 on the ProMicro, but 'Serial1' already
 knows that.  The serial Rx LED is pin 17 and can be used independently, the
 Tx pin needs to be treated differently- see the ProMicro hookup guide example.

 The GPS doesn't seem to have a long memory and resets its programming after
 being off for a couple days. So it seems that we have to reprogram it at
 startup. I'd like to have it at 19200 baud instead of the default of 4800
 but rerunning Serial1.begin() after reprogramming the baud rate and then
 sleeping for 1 sec doesn't seem to work well and is probably too risky to
 trust anyway.
    Serial1.print("$PSRF100,1,19200,8,1,0*38\r\n");   // set baud rate to 19200, N81
    Serial1.print("$PSRF100,1,9600,8,1,0*0D\r\n");    // set baud rate to 9600, N81
    Serial1.print("$PSRF100,1,4800,8,1,0*0E\r\n");    // set baud rate to 4800, N81

    Serial1.begin(19200);
    delay(1000);


 State machine:  (incompatible with always listening to the GPS stream, see below)
    unsigned long prev_millisec = 0;
    long interval = 1000;
    ...
    unsigned long current_millisec;
    // wait a minimum of 1 second between iterations
    current_millisec = millis();
    if(current_millisec - prev_millisec < interval)
        return;    //  (->low power mode?)

    prev_millisec = current_millisec;
    Serial.println(".");


GPS library:
  TinyGPS++ (LGPL>=2.1)  http://arduiniana.org/libraries/tinygpsplus/
  Because the GPS is always coming in and the input buffer on the UART is
  only 64 bytes(?), we can't just delay() or go to sleep between outputs,
  we constantly have to read() & digest the NMEA sentences coming in. The
  TinyGPS++ examples have a smartDelay() function to aid with this.

  The "isValid" result is refering to the last checksum **NOT** the GPS fix.
  The "isUpdated" result is true if a new valid (both checksum and fix) NMEA
  message has come through and the memory field has been touched since the
  last time you QUERIED it. So the date fields will be "updated" once per
  second by $GPRMC, not once per day when the date changes.
  Use the "age()" result to query for the last good fix (1500 ms is suggested).

  TinyGPS++ mainly works on GGA and RMC, so we turn off the other NMEA sentences.

  Lat/Lon floating point values are restricted to 32bit precision; so a total
  of 7 or 8 significant digits including those before the decimal point.
  1e-5 * 1852 * 60 = 1.11 meters.  The GPS unit outputs decimal minutes to
  0.0001 precision, (0.0001 / 60) * 1852 * 60 =  0.18520 meters. The TinyGPS++
  library gives us "raw" billionths of a degree output we can reassemble later.
  [ Perhaps use the NeoGPS library for better precision?
     https://github.com/SlashDevin/NeoGPS ]


LiDAR:
 I2C SDA,CLK on the ProMicro are pins 2 and 3. The unit is preprogrammed at
 the 7-bit address 0x55. It returns two bytes of data to make the integer
 number of centimeters distance. The result is ordered big-endian.
    ////// get laser value //////
    // get 2 bytes from the SF-11 range finder
    Wire.requestFrom(I2C_ADR, 2);
    if (Wire.available() >= 2) {
        byteH = Wire.read();
        byteL = Wire.read();
        // combine in big endian order
        distance_cm = byteH * 256 + byteL;
        Serial.print("\tlidar (cm): ");
        Serial.println(distance_cm);
    }

SDcard:
 The SD card should be formatted using the SD card foundation's official formatting
 tool. Note the SDcard reader must be fed 3.3v for VCC, as 5v will damage it.


IMU:
  Pololu MinIMU-9 v5  www.pololu.com
  """ LIS3MDL mag sensor: (unused) """
  """LSM6DS33 gyro and accel sensor:
  The sensor outputs provided by the library are the raw
  16-bit values obtained by concatenating the 8-bit high and
  low accelerometer and gyro data registers. They can be
  converted to units of g and dps (degrees per second) using
  the conversion factors specified in the datasheet for your
  particular device and full scale setting (gain).

  Example: An LSM6DS33 gives an accelerometer Z axis reading
  of 16276 with its default full scale setting of +/- 2 g. The
  LA_So specification in the LSM6DS33 datasheet (page 11)
  states a conversion factor of 0.061 mg/LSB (least
  significant bit) at this FS setting, so the raw reading of
  16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
  """
*/