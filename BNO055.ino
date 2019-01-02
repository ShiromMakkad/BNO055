#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

   /*
    * Orientation:
    *  
    *       front
    *   --------------
    *   |   BNO55    |
    *   |            |
    *   | Vin        |
    *   | 3Vo    PS0 | <-- This side facing up
    *   | GND    PS1 |
    *   | SCL    ADW |
    *   | DST        |
    *   |            |
    *   |   9-Axis   |
    *   --------------
    *        back
    * 
    */
int zeroTime = 50;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

double degToRad = 57.295779513;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void bunnyPrintOrientation(double x, double y, double z) {
  Serial.print(F("Orientation: "));
  Serial.print((float)x);
  Serial.print(F(" "));
  Serial.print((float)y);
  Serial.print(F(" "));
  Serial.print((float)z);
  Serial.println(F(""));
}

void bunnyPrintCalibration() {
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.print(mag, DEC);
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void setup(void)
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    if(calibrate) {
      int eeAddress = 0;
      long bnoID;
      bool foundCalib = false;
  
      EEPROM.get(eeAddress, bnoID);
  
      adafruit_bno055_offsets_t calibrationData;
      sensor_t sensor;
  
      /*
      *  Look for the sensor's unique ID at the beginning oF EEPROM.
      *  This isn't foolproof, but it's better than nothing.
      */
      bno.getSensor(&sensor);
      if (bnoID != sensor.sensor_id)
      {
          Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
          delay(500);
      }
      else
      {
          Serial.println("\nFound Calibration for this sensor in EEPROM.");
          eeAddress += sizeof(long);
          EEPROM.get(eeAddress, calibrationData);
  
          displaySensorOffsets(calibrationData);
  
          Serial.println("\n\nRestoring Calibration data to the BNO055...");
          bno.setSensorOffsets(calibrationData);
  
          Serial.println("\n\nCalibration data loaded into BNO055");
          foundCalib = true;
      }
  
      delay(1000);
  
      /* Display some basic information on this sensor */
      displaySensorDetails();
  
      /* Optional: Display current status */
      displaySensorStatus();
  
     //Crystal must be configured AFTER loading calibration data into BNO055.
      bno.setExtCrystalUse(true);
  
      sensors_event_t event;
      bno.getEvent(&event);
      if (foundCalib){
          Serial.println("Move sensor slightly to calibrate magnetometers");
          while (!bno.isFullyCalibrated())
          {
              bno.getEvent(&event);
              delay(BNO055_SAMPLERATE_DELAY_MS);
          }
      }
      else
      {
          Serial.println("Please Calibrate Sensor: ");
          while (!bno.isFullyCalibrated())
          {
              bno.getEvent(&event);
  
              imu::Vector<3> euler = bno.getQuat().toEuler();
              
              double x = euler.y() * degToRad;
              double y = euler.z() * degToRad;
              double z = euler.x() * degToRad;
              
              Serial.print("X: ");
              Serial.print(x, 4);
              Serial.print(" Y: ");
              Serial.print(y, 4);
              Serial.print(" Z: ");
              Serial.print(z, 4);
              Serial.print("\t\t");
  
              /* Optional: Display calibration status */
              displayCalStatus();
  
              /* New line for the next sample */
              Serial.println("");
  
              /* Wait the specified delay before requesting new data */
              delay(BNO055_SAMPLERATE_DELAY_MS);
          }
      }
  
      Serial.println("\nFully calibrated!");
      Serial.println("--------------------------------");
      Serial.println("Calibration Results: ");
      adafruit_bno055_offsets_t newCalib;
      bno.getSensorOffsets(newCalib);
      displaySensorOffsets(newCalib);
  
      Serial.println("\n\nStoring calibration data to EEPROM...");
  
      eeAddress = 0;
      bno.getSensor(&sensor);
      bnoID = sensor.sensor_id;
  
      EEPROM.put(eeAddress, bnoID);
  
      eeAddress += sizeof(long);
      EEPROM.put(eeAddress, newCalib);
      Serial.println("Data stored to EEPROM.");
    }
    else {
      bno.setExtCrystalUse(true);
    }
    
    if(zero) {
      Serial.println("Zeroing... Please do not move the device");
      delay(1000);
    }
    
    bno.setMode(0X0C);

    delay(500);
}

int i = 0;

double totEulerX = 0;
double totEulerY = 0;
double totEulerZ = 0;

double subEulerX = 0;
double subEulerY = 0;
double subEulerZ = 0;

void loop() {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getQuat().toEuler();

    if(zero == false) {
      i = zeroTime + 1;
    }
    if(i < zeroTime) {
      totEulerX += euler.x();
      totEulerY += euler.y();
      totEulerZ += euler.z();
    }
    else if(i == zeroTime) {
      subEulerX = totEulerX / 100;
      subEulerY = totEulerY / 100;
      subEulerZ = totEulerZ / 100;
    }
    else {
      //Display the orientation data
      double x = (euler.y() - subEulerY) * degToRad;
      double y = (euler.z() - subEulerZ) * degToRad;
      double z = (euler.x() - subEulerX) * degToRad;

      /*
      Serial.print("X: ");
      Serial.print(x, 4);
      Serial.print(" Y: ");
      Serial.print(y, 4);
      Serial.print(" Z: ");
      Serial.print(z, 4);
      Serial.print("\t\t");
  
      //Display the gyro data
      Serial.print("X: ");
      Serial.print(gyro.x());
      Serial.print(" Y: ");
      Serial.print(gyro.y());
      Serial.print(" Z: ");
      Serial.print(gyro.z());
      Serial.print("\t\t");
      */
      
      //Optional: Display calibration status
      //displayCalStatus();
  
      //Optional: Display sensor status (debug only)
      //displaySensorStatus();

      bunnyPrintOrientation(-(z - 90), x, -y);
      bunnyPrintCalibration();
      
      /* New line for the next sample */
      Serial.println("");
    }

    i += 1;
    /* Wait the specified delay before requesting new data */
    delay(BNO055_SAMPLERATE_DELAY_MS);
}

static int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
