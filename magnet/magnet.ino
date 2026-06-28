#include "final_motor_2.h"   // Labsheet 1
//#include "PID.h"             // Labsheet 1 - Advanced
#include "LineSensors.h"     // Labsheet 2
#include "Magnetometer.h"    // Labsheet 3
#include "Kinematics.h"      // Labsheet 4
#include "Encoders.h"        // For encoder counts
# include <PololuOLED.h>

//PololuOLED display;

LineSensors_c line_sensors;

Magnetometer_c magnetometer;

//Buzzer function
#define BUZZER_PIN 6
unsigned long beep_stop_time;
void setBeep( unsigned long duration_ms ) {
  analogWrite( BUZZER_PIN, 120); // on
  beep_stop_time = millis() + duration_ms;
}

//kinematics//
bool is_turning;
bool is_moving;
Kinematics_c pose;
unsigned long pose_ms;   // FIX: millis() is unsigned long; 16-bit unsigned overflows ~65 s
unsigned long pose_ts;
float target_angle;
bool detect_magnet;
float mag_x;
float mag_y;
float dist;
float mag_theta;
bool return_home;
float target_dist;

float home_x;

void checkBeep() {
  unsigned long current_millis = millis();
  if (current_millis > beep_stop_time) {
    analogWrite( BUZZER_PIN, 0 ); // off
  }
}

void beep_sequence() {
  analogWrite( BUZZER_PIN, 200);
  delay(150);
  analogWrite( BUZZER_PIN, 150);
  delay(150);
  analogWrite( BUZZER_PIN, 100);
  delay(150);
  analogWrite( BUZZER_PIN, 0);
}
//Motors calibration and setturn function
Motors_c motors;
float left;
float right;
unsigned long int stop_time;
# define TURN_THRESHOLD 0.1
# define DIST_THRESHOLD 0.1

//wheel speed calculation
long last_e0;
long last_e1;
float speed_e0;
float speed_e1;
float filtered_speed_e0;
float filtered_speed_e1;
float alpha = 0;

unsigned long stopSearch;
unsigned long stop_cal;
bool calibrating = false;



// Search time.........
#define searchTime 120000
// Ends................


unsigned long speed_est_ts; // timestamp for speed estimation
# define speed_est_ms 10


void wheel_speed() {
  unsigned long current_time = millis() ;
  unsigned long elapsed_time = current_time - speed_est_ts;
  float fs0 = filtered_speed_e0;
  float fs1 = filtered_speed_e1;
  if ( elapsed_time >= speed_est_ms ) {

    // Work out the difference in encoder counts
    long distance = last_e0 - count_e0;
    long distance2 = last_e1 - count_e1;
    last_e0 = count_e0;
    last_e1 = count_e1;


    speed_e0 = float(distance * 1000) / float(elapsed_time);
    speed_e1 = float(distance2 * 1000) / float(elapsed_time);
    speed_e0 /= 3600;
    speed_e1 /= 3600;
    speed_est_ts = current_time;
    filtered_speed_e0 = alpha * speed_e0 + (1 - alpha) * fs0;
    filtered_speed_e1 = alpha * speed_e1 + (1 - alpha) * fs1;

    // Report the result, so that it can be viewed
    // on the Serial Plotter.
  }
}

/********Turn function*********/
void setTurn(float a, float b, float angle) {
  if (is_turning == false) {
    pose.update();
    motors.setPWM(a, b);
    target_angle = pose.theta + angle;
    is_turning = true;
  }
}

void checkTurn() {
  pose.update();
  float angle_diff = target_angle - pose.theta;
  if (fabs(angle_diff) < TURN_THRESHOLD) {
    motors.setPWM(0, 0);
    is_turning = false;
    target_angle = 0;
  } else {
    // Do nothing
  }
}

void setForward(float a, float b, float distance) {
  pose.update();
  motors.setPWM(a, b);
  target_dist = pose.x + distance;
  is_moving = true;
}

bool checkForward() {
  pose.update();
  float target_diff = target_dist - pose.x;
  if (fabs(target_diff) < DIST_THRESHOLD) {
    //motors.setPWM(0, 0);
    motors.setPWM(0, 0);
    is_moving = false;
    return false;
  } else {
    // do nothing
    return true;
    is_moving = true;
  }
}
void set_calib_rout(float angle) {
  pose.update();
  motors.setPWM(-30, 33);
  target_angle = pose.theta + angle;
  calibrating = true;
}

void check_calib_rout() {
  pose.update();
  float angle_diff = target_angle - pose.theta;
  if (fabs(angle_diff) < TURN_THRESHOLD) {
    motors.setPWM(0, 0);
    calibrating = false;
  } else {
    // do nothing
  }
}

int Max_Sensor = 5;
void sensor_mag_calibration() {
  set_calib_rout(12.56);

  //n=sensor number
  for (int n = 0; n < Max_Sensor; n++) {
    line_sensors.minimum[n] = 1023;
    line_sensors.maximum[n] = 0;
  }

  //float Range_x, Range_y, Range_z;
  float Range[MAX_AXIS];
  for (int n = 0; n < MAX_AXIS; n++) {
    // FIX: seed max LOW and min HIGH so the running max/min can actually update.
    // (Was inverted, which froze the range at +/-9999.9 and broke magnet detection.)
    magnetometer.maximum[n] = -9999.9;
    magnetometer.minimum[n] = 9999.9;
  }
  line_sensors.calcCalibratedADC();
  while (calibrating == true) {
    line_sensors.readSensorsADC();
    magnetometer.getReadings();
    for ( int n = 0; n < Max_Sensor; n++) {
      if (line_sensors.readings[n] > line_sensors.maximum[n]) {
        line_sensors.maximum[n] = line_sensors.readings[n];
      }
      if (line_sensors.readings[n] < line_sensors.minimum[n]) {
        line_sensors.minimum[n] = line_sensors.readings[n];
      }
    }

    for (int n = 0; n < MAX_AXIS; n++) {
      if (magnetometer.readings[n] > magnetometer.maximum[n]) {
        magnetometer.maximum[n] = magnetometer.readings[n];
      }
      else if (magnetometer.readings[n] < magnetometer.minimum[n]) {
        magnetometer.minimum[n] = magnetometer.readings[n];
      }//end if
    }//end for
    delay(10);
    check_calib_rout();
  }//end while sensor_calibration

  for (int n = 0; n < Max_Sensor; n++) {
    line_sensors.scaling[n] = line_sensors.maximum[n] - line_sensors.minimum[n];

  }

  for (int n = 0; n < MAX_AXIS; n++) {
    Range[n] = magnetometer.maximum[n] - magnetometer.minimum[n];
    magnetometer.Offset[n] = magnetometer.minimum[n] + (Range[n] / 2.0);
    magnetometer.Scaling[n] = 1.0 / (Range[n] / 2.0);
  } //end for loop
}//end sensor and magnet calibration loop

float normal;

void setReturn(float x, float y) {
  pose.update();
  float target_angle = atan2((y - pose.y), (x - pose.x));
  float normal = atan2(sin(target_angle - pose.theta), cos(target_angle - pose.theta));
  if (normal > 0) {
    setTurn(-40, 42, normal);
  } else if (normal < 0) {
    setTurn(40, -42, normal);
  }
  return_home = true;
}
bool checkReturn(float x) {
  pose.update();
  while (is_turning == true) {
    checkTurn();
  }
  home_x = pose.x;
  if (home_x < x) {
    motors.setPWM(-46, -50);
    if (0 - pose.x > 1) {
      motors.setPWM(0, 0);
      return_home = false;
      return false;
    }
  } else if (home_x > x) {
    motors.setPWM(46, 50);
    if (0 - pose.x > 1) {
      motors.setPWM(0, 0);
      return_home = false;
      return false;
    }
  }
  return_home = true;
  return true;
}


void setup() {

  Serial.begin(9600);
  Serial.println("\n\n * READY *");
  pinMode(BUZZER_PIN, OUTPUT);
  setBeep(250);
  beep_sequence();

  motors.initialise();

  line_sensors.initialiseForADC();

  pose.initialise(0, 0, 0);

  pose_ms = 20;
  pose_ts = millis();

  magnetometer.initialise();

  setupEncoder0();
  setupEncoder1();

  last_e0 = count_e0;
  last_e1 = count_e1;

  speed_e0 = 0.0;
  speed_e1 = 0.0;

  //  speed_est_ms = 10;
  speed_est_ts = millis();
  wheel_speed();
  pose.update();
  sensor_mag_calibration();
  // buzzer beeps sequence
}
bool mag;
bool ended;
void setSearch() {
  line_sensors.calcCalibratedADC();
  magnetometer.calcCalibratedMag();
  while (magnetometer.isOnMag() == false) {
    pose.update();
    if ( line_sensors.isOnLine(2) == true || line_sensors.isOnLine(3) == true || line_sensors.isOnLine(4) == true || line_sensors.isOnLine(2) == true && line_sensors.isOnLine(3) == true || line_sensors.isOnLine(3) == true && line_sensors.isOnLine(4) == true || line_sensors.isOnLine(2) == true && line_sensors.isOnLine(4) == true || line_sensors.isOnLine(2) == true && line_sensors.isOnLine(3) == true && line_sensors.isOnLine(4) == true) {
      setTurn(-56, 60, 0.79);
      while (is_turning == true) {
        checkTurn();
      }
    }
    else if ( line_sensors.isOnLine(0) == true || line_sensors.isOnLine(1) == true || line_sensors.isOnLine(0) == true && line_sensors.isOnLine(1) == true || line_sensors.isOnLine(1) == true && line_sensors.isOnLine(2) == true || line_sensors.isOnLine(0) == true && line_sensors.isOnLine(2) == true || line_sensors.isOnLine(0) == true && line_sensors.isOnLine(1) == true && line_sensors.isOnLine(2) == true) {
      setTurn(56, -60, -0.79);
      while (is_turning == true) {
        checkTurn();
      }
    }
    else if (pose.y > -240) {
      if (pose.x >= 40) {
        motors.setPWM(0, 34);
        motors.setPWM(30, 34);
      }
      else if (pose.x <= 39) {
        motors.setPWM(34, 0);
        motors.setPWM(30, 34);
      }
    }
    else {
      motors.setPWM(30, 34);
    }
    while (magnetometer.isOnMag() == true) {
      motors.setPWM(0, 0);
      analogWrite( BUZZER_PIN, 100);
      delay(150);
      analogWrite( BUZZER_PIN, 0);
      mag = true;
      break;
    }
    if (millis() > searchTime && mag == false) {
      ended = true;
      setReturn(0, 0);
      while (return_home == true) {
        checkReturn(0);
        while (!checkReturn(0)) {
          motors.setPWM(0, 0);
        }
      }
    }
    else if (ended == false && mag == true) {
      setReturn(0, 0);
      while (return_home == true) {
        checkReturn(0);
        while (!checkReturn(0)) {
          motors.setPWM(0, 0);
        }
      }
    }
  }
}

void loop() {
  if (millis() - pose_ts > pose_ms) {
    pose_ts = millis();
    pose.update();
  }
  setTurn(30, -34, -0.79);
  while (is_turning == true) {
    checkTurn();
  }
  setForward(30, 34, 250);
  while (is_moving == true) {
    checkForward();
  }
  setSearch();

}
