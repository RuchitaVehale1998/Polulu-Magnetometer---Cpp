#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define NUM_SENSORS 2

const int sensor_pins[NUM_SENSORS] = { A6, 5 };

#define EMIT_PIN 11

class LineSensors_c {

public:

  float readings[NUM_SENSORS];

  float minimum[NUM_SENSORS];
  float maximum[NUM_SENSORS];
  float scaling[NUM_SENSORS];

  float calibrated[NUM_SENSORS];

  // Constructor, must exist.
  LineSensors_c() {   // FIX: was LineSensor_c (missing 's') — not actually a constructor
    // leave this empty
  }

  void initialiseForADC() {
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      pinMode(sensor_pins[sensor], INPUT_PULLUP);
    }
  }

  void readSensorsADC() {

    initialiseForADC();
    readings[0] = analogRead(sensor_pins[0]);
    readings[1] = analogRead(sensor_pins[1]);
  }

  void calcCalibratedADC() {
    readSensorsADC();

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      calibrated[sensor] = (readings[sensor] - minimum[sensor]) / scaling[sensor];
    }
  }
};


#endif
