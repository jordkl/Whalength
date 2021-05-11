// Program to talk to GlobalSat EM-506 GPS and lightware SF-11 laser rangefinder
// and log to SparkFun SDcard board.
// HB 14 June 2016     (added IMU code 19 May 2017)



#include <SPI.h>
#include <Wire.h>    // Wire.h provides I2C bus
#include <TinyGPS++.h>
#include <LSM6.h>
#include "SD.h"


#define USB_CONNECTED


// LiDAR I2C pins
//#define I2C_SDA  2
//#define I2C_CLK  3
#define I2C_ADR  0x55

// SDcard SPI pins
#define SPI_CS   10
//#define SPI_CLK  15
//#define SPI_MISO 14
//#define SPI_MOSI 16
#define RX_LED   17


// The TinyGPS++ object
TinyGPSPlus gps;


#define NUMSAMPLES 100    // for 400 it takes approx 0.852sec/cycle; 100 samples take 0.212 sec.

// IMU sensor
LSM6 imu;


// write out to sd card at least every 60 seconds
unsigned long prev_millisec = 0;
long interval = 60000;


// the logging file
File logfile;



////////////////////////////////////////////////////////////

int freeRam () {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

////////////////////////////////////////////////////////////


void setup(void)
{

#ifdef USB_CONNECTED
    //// Start the USB-serial, for debugging
    Serial.begin(9600);
    /** Careful with this next line, if computer isn't attatched it will hang **/
    while(!Serial) {   // loop while ProMicro takes a moment to get itself together
        if (millis() > 6000)   // give up waiting for USB cable plugin after 6 sec
            break;
    }

    Serial.print(F("Free RAM: "));
    Serial.println(freeRam());
#endif

    //// GPS is on the ProMicro's UART (Serial1)
    Serial1.begin(4800);
    while(!Serial1);   // wait for it   (** NOTE this will hang the logger if GPS is not connected **)
    // Turn off messages we don't want
    Serial1.print(F("$PSRF103,02,00,00,01*26\r\n"));      // GSA off
    delay(20);
    Serial1.print(F("$PSRF103,03,00,00,01*27\r\n"));      // GSV off
    delay(20);

    //// startup I2C bus for the LiDAR
    Wire.begin();


    //// init LSM6 portion of IMU board
    if (!imu.init()) {
#ifdef USB_CONNECTED
        Serial.println("Failed to detect and initialize IMU!");
#endif
        lock_and_blink();  // lock it up
    }
    imu.enableDefault();


    /////////////////////////////////////////
#ifdef USB_CONNECTED
    Serial.print(F("Initializing SD card..."));
#endif
    // see if the card is present and can be initialized:
    if (!SD.begin(SPI_CS)) {
#ifdef USB_CONNECTED
        Serial.println(F("ERROR: SD card failed, or not present. Halting."));
#endif
        lock_and_blink();  // lock it up
        //or,  return;
    }
#ifdef USB_CONNECTED
    Serial.println(F("SD card initialized."));
#endif


/* TODO: set once we have GPS lock
    // set SD file date time callback function
    SdFile::dateTimeCallback(FileDateTime);
*/

    // create a new file
    char filename[] = "LOG_0000.CSV";
    //char filename[] = "LOGB0000.CSV";
    for (uint16_t i = 0; i < 10000; i++) {
        filename[4] = i / 1000 + '0';
        filename[5] = (i % 1000) / 100 + '0';
        filename[6] = (i % 100) / 10 + '0';
        filename[7] = i % 10 + '0';
        //Serial.print("Trying ");
        //Serial.println(filename);
        if (! SD.exists(filename)) {
            // only open a new file if it doesn't exist
            logfile = SD.open(filename, FILE_WRITE); 
            break;  // leave the loop!
        }
        //else {   // tidy up
        //   SD.remove(filename);
        //}
    }

    delay(500); // give it a chance to catch up before testing if it's ok.
    if (! logfile) {
#ifdef USB_CONNECTED
        Serial.println(F("ERROR: couldn't create log file. Halting"));
#endif
        lock_and_blink();  // lock it up
    }

#ifdef USB_CONNECTED
    Serial.print(F("Logging to: "));
    Serial.println(filename);
    Serial.println();

    ////////////////////////////////////

    Serial.println(F("# GPS and Laser Rangefinder logging with Pro Micro Arduino 3.3v"));
    Serial.println(F("# units:  accel=1g  gyro=deg/sec"));

    Serial.print(F("#gmt_date\tgmt_time\tnum_sats\tlongitude\tlatitude\t"));
    Serial.print(F("gps_altitude_m\tSOG_kt\tCOG\tHDOP\tlaser_altitude_cm\t"));
    Serial.println(F("tilt_deg\taccel_x\taccel_y\taccel_z\tgyro_x\tgyro_y\tgyro_z"));
#endif
    logfile.println(F("# GPS and Laser Rangefinder logging with Pro Micro Arduino 3.3v"));
    //logfile.println(F("#  Datalogger unit 'B'"));
    logfile.println(F("# units:  accel=1g  gyro=deg/sec"));

    logfile.print(F("#gmt_date\tgmt_time\tnum_sats\tlongitude\tlatitude\t"));
    logfile.print(F("gps_altitude_m\tSOG_kt\tCOG\tHDOP\tlaser_altitude_cm\t"));
    logfile.println(F("tilt_deg\taccel_x\taccel_y\taccel_z\tgyro_x\tgyro_y\tgyro_z"));
    logfile.flush();

    // should we wait a while for GPS to get a fix?
    wakeful_sleep(2000);

    // blink 10 times, we're good to go
    for (int i = 0; i < 10; i++) {
        digitalWrite(RX_LED, LOW);
        TXLED1;
        wakeful_sleep(100);
        digitalWrite(RX_LED, HIGH);
        TXLED0;
        wakeful_sleep(100);
    }
}


///////////////////////////////////////////////////

struct imu_data {
    float Grav_x;
    float Grav_y;
    float Grav_z;
    float Gyro_x;
    float Gyro_y;
    float Gyro_z;
    float tilt_deg;
};


void loop(void)
{

#ifdef DEBUG_NMEA
    // Echo GPS to USB-serial port for debugging:
    char c = 'x';    //gps
    while(Serial1.available() > 0) {
        c = Serial1.read();
        if(c == '\r')
            continue;
        else if(c == '\n')
            Serial.println();
        else
            Serial.print(c);
    }
#endif

    ////// get GPS string //////
    while(Serial1.available() > 0)
        gps.encode(Serial1.read());

    // perhaps output all NaNs every 10 sec if no fix, just to show something is happening?
    if(!gps.date.isUpdated() || gps.location.age() > 1750)
        return;
        //{delay(2000); Serial.println("NO FIX!");}
//    /* Debug */
//    if(!gps.date.isUpdated())
//        return;


    ////// IMU ///////////////////////
    struct imu_data imu_results;
    imu_results = get_IMU_readings();

    //// update gps data avail scan again to clear the decks
    while(Serial1.available() > 0)
        gps.encode(Serial1.read());


    //// Printout to USB-serial
#ifdef USB_CONNECTED
    print_gps_date();
    Serial.print(F("\t"));
    print_gps_time(); 
    Serial.print(F("\t"));

    Serial.print(gps.satellites.value());
    Serial.print(F("\t"));
    if (gps.location.isValid()) {
	Serial.print(gps.location.lng(), 6);
	Serial.print(F("\t"));
	Serial.print(gps.location.lat(), 6);
    }
    else {
	Serial.print(F("NaN\tNaN"));
    }
    Serial.print(F("\t"));

    if (gps.altitude.isValid())
	Serial.print(gps.altitude.meters());
    else
	Serial.print(F("NaN"));
    Serial.print(F("\t"));

    if (gps.speed.isValid())
	Serial.print(gps.speed.knots());
    else
	Serial.print(F("NaN"));
    Serial.print(F("\t"));

    if (gps.course.isValid())
	Serial.print(gps.course.deg());
    else
	Serial.print(F("NaN"));
    Serial.print(F("\t"));
    if (gps.hdop.isValid())
	Serial.print(gps.hdop.value());
    else
	Serial.print(F("NaN"));
    Serial.print(F("\t"));

    // lidar
    print_lidar_alt();	      
    Serial.print(F("\t"));

    // IMU
    Serial.print(imu_results.tilt_deg, 2);
    Serial.print(F("\t"));
    print_imu_data(imu_results);  // grav x,y,z then gyro x,y,z
    Serial.println();

    // clear the pipes
    wakeful_sleep(0);
#endif

    ///// write to SD card /////
    TXLED1;   // The Tx LED is not tied to a normally controlled pin so we use this macro

    write_gps_date();
    logfile.print(F("\t"));
    write_gps_time(); 
    logfile.print(F("\t"));

    logfile.print(gps.satellites.value());
    logfile.print(F("\t"));
    if (gps.location.isValid()) {
        logfile.print(gps.location.lng(), 6);
        logfile.print(F("\t"));
        logfile.print(gps.location.lat(), 6);
    }
    else {
        logfile.print(F("NaN\tNaN"));
    }
    logfile.print(F("\t"));

    if (gps.altitude.isValid())
        logfile.print(gps.altitude.meters());
    else
        logfile.print(F("NaN"));
    logfile.print(F("\t"));

    if (gps.speed.isValid())
        logfile.print(gps.speed.knots());
    else
        logfile.print(F("NaN"));
    logfile.print(F("\t"));

    if (gps.course.isValid())
        logfile.print(gps.course.deg());
    else
        logfile.print(F("NaN"));
    logfile.print(F("\t"));
    if (gps.hdop.isValid())
        logfile.print(gps.hdop.value());
    else
        logfile.print(F("NaN"));
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


    // The following line will 'save' the file to the SD card after every
    // 60 seconds of data - this will use more power but it's safer!
    // If you want to speed up the system, remove the call to flush() and it
    // will save the file only every 512 bytes - every time a sector on the 
    // SD card is filled with data. (about every 6 seconds)
    // The flush() may still be needed to update the FAT?
    unsigned long current_millisec;
    current_millisec = millis();
    if(current_millisec - prev_millisec > interval) {
        prev_millisec = current_millisec;
#ifdef USB_CONNECTED
        Serial.println("flushing to SDcard.");
#endif
        digitalWrite(RX_LED, LOW);   // set the Rx LED on
        logfile.flush();
        digitalWrite(RX_LED, HIGH);   // set the Rx LED off
        //wakeful_sleep(20);
    }

    // checking to see if the 1Hz $GPRMC date is updated seems to be enough for a delay.
}


