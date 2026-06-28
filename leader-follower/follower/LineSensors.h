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
    pinMode(sensor_pins[0], INPUT_PULLUP);
    pinMode(sensor_pins[1], INPUT);
  }

  void readSensorsADC() {

    initialiseForADC();
    for (int sensors = 0; sensors < NUM_SENSORS; sensors++) {
      readings[sensors] = analogRead(sensor_pins[sensors]);
    }
  }

  void calcCalibratedADC() {
    readSensorsADC();
    for (int sensors = 0; sensors < NUM_SENSORS; sensors++) {
      calibrated[sensors] = (readings[sensors] - minimum[sensors]) / scaling[sensors];
    }
  }
};


#endif
