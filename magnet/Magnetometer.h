/***************************************
  ,        .       .           .     ,--,
  |        |       |           |       /
  |    ,-: |-. ,-. |-. ,-. ,-. |-     `.
  |    | | | | `-. | | |-' |-' |        )
  `--' `-` `-' `-' ' ' `-' `-' `-'   `-'
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3


class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings
    // from the magnetometer
    float readings[ MAX_AXIS ];

    //added later
    float minimum[MAX_AXIS];
    float maximum[MAX_AXIS];
    float Offset[MAX_AXIS];
    float Scaling[MAX_AXIS];
    float calibrated[MAX_AXIS];//added end
    float magnitude;
    float magnet_angle;


    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor


    bool initialise() {

      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if (!mag.init()) {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1)
          ;
      }
      mag.enableDefault();
      return true;   // FIX: declared bool but previously returned nothing (UB)
    }

    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()

    //added later
    void calcCalibratedMag() {
      getReadings();
      for (int n = 0; n < MAX_AXIS; n++) {
        calibrated[n] = (readings[n] - Offset[n]) * Scaling[n];
      }//end for
//      Serial.println("calibrated_values:");
//      Serial.println(calibrated[MAX_AXIS]);
      magnitude = sqrt(sq(calibrated[0]) + sq(calibrated[1]) + sq(calibrated[2]));
//      Serial.print("Magnetic field magnitude: ");
//      Serial.println(magnitude);
      magnet_angle = atan2(calibrated[0], calibrated[1]);
//      Serial.print("magnet_angle:");
//      Serial.println(magnet_angle);

      //polar to cartesian transform
      //      float x_coordinate = magnitude* cos(magnet_angle);
      //      float y_coordinate = magnitude*sin(magnet_angle);
      //      Serial.println("x_cordinate", x_coordinate);
      //      Se rial.println("y_cordinate", y_coordinate);
    }

    bool isOnMag() {
      calcCalibratedMag();
      if (magnitude > 2) {
        return true;
      }
      else {
        return false;
      }
    }
}; // End of Magnetometer_c class definition
#endif
