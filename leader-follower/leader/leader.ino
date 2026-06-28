#include "Motors.h"      // For Motor Control
#include "PID.h"         // For PID spoeed control
#include "Kinematics.h"  // For advanced movement and bot position tracking
#include "Encoders.h"    // For encoder counts
#include "LineSensors.h"
#include <Pushbutton.h>  // For Push Buttons
#include <PololuOLED.h>  // For Display

#define BUZZER_PIN 6

#define BUTTON_PIN 30

Pushbutton button(BUTTON_PIN);

PololuSH1106 display(1, 30, 0, 17, 13);

uint8_t graphics[1024];

void clearGraphics() {
  memset(graphics, 0, sizeof(graphics));
}

void setPixel(uint8_t x, uint8_t y, bool value) {
  if (x >= 128 || y >= 64) { return; }
  if (value) {
    graphics[x + (y >> 3) * 128] |= 1 << (y & 7);
  } else {
    graphics[x + (y >> 3) * 128] &= ~(1 << (y & 7));
  }
}

Motors_c motors;

Kinematics_c pose_leader;

LineSensors_c lsc;


unsigned long stopFwd;
unsigned long stopCal;
unsigned long stopTurn;
unsigned long stopBeep;

bool calibrating = false;

bool turn;
bool fwd;
bool return_home;

unsigned long ts_task;
unsigned long ms_task;

// encoder values starts here
unsigned long speed_est_ts;  // timestamp for speed estimation
unsigned long speed_est_ms;  // how frequently the speed will be calculated in ms

long last_e0;
long last_e1;

float speed_e0;
float speed_e1;

float smoothL;
float stL;
float aL = 0.08;

float smoothR;
float stR;
float aR = 0.08;
// encoder values ends here

// Kinematics value starts here
float target_angle;
float target_dist;

float x_home;
float y_home;
float theta_home;

#define TURN_THRESHOLD 0.05
#define DIST_THRESHOLD 0.1
// Kinematice vales ends here

// PID values starts here
PID_c left_pid;   // To control the left motor
PID_c right_pid;  // To control the right motor
// PID values ends here

// PID task scheduling
unsigned long pid_ts;  // _ts, for "timestamp"
unsigned long pid_ms;  // _ms, how frequently, in milliseconds
// ends here
// .
// .
// .
// .
// .
// Kinematics task scheduling
unsigned long pose_leader_ts;
unsigned long pose_leader_ms;

#define aPin A6
#define dPin 5
#define emit 11

#define bit_duration 10

#define NUM_SENSORS 2

// Non-blocking Forward moving function
void set_fwd(float a, float b, float distance) {
  pose_leader.update();
  pid_n_enc(a, b);
  target_dist = pose_leader.x + distance;
  fwd = true;
}

bool check_fwd() {
  pose_leader.update();
  float target_diff = target_dist - pose_leader.x;
  if (fabs(target_diff) < DIST_THRESHOLD) {
    //motors.setPWM(0, 0);
    pid_n_enc(0.0, 0.0);
    fwd = false;
    return false;
  } else {
    // do nothing
    return true;
    fwd = true;
  }
}
// ends here
// .
// .
// .
// .
// .
// Non-blocking turn function
void set_turn(float a, float b, float angle) {
  if (turn == false) {
    pose_leader.update();
    pid_n_enc(a, b);
    target_angle = pose_leader.theta + angle;
    turn = true;
  }
}

void check_turn() {
  pose_leader.update();
  float angle_diff = target_angle - pose_leader.theta;
  if (fabs(angle_diff) < TURN_THRESHOLD) {
    pid_n_enc(0, 0);
    turn = false;
    target_angle = 0;
  } else {
    // Do nothing
  }
}


void set_turn_pwm(float a, float b, float angle) {
  if (turn == false) {
    pose_leader.update();
    motors.setPWM(a, b);
    target_angle = pose_leader.theta + angle;
    turn = true;
  }
}

void check_turn_pwm() {
  pose_leader.update();
  float angle_diff = target_angle - pose_leader.theta;
  if (fabs(angle_diff) < TURN_THRESHOLD) {
    motors.setPWM(0, 0);
    turn = false;
    target_angle = 0;
  } else {
    // Do nothing
  }
}
// ends here
// .
// .
// .
// .
// .
// delay Beep in sequence function
void beep_sequence() {
  analogWrite(BUZZER_PIN, 230);
  delay(125);
  analogWrite(BUZZER_PIN, 165);
  delay(125);
  analogWrite(BUZZER_PIN, 100);
  delay(125);
  analogWrite(BUZZER_PIN, 35);
  delay(125);
  analogWrite(BUZZER_PIN, 0);
}
// ends here
// .
// .
// .
// .
// .
// Non-blocking beep function
void set_beep(unsigned long duration_ms) {
  stopBeep = duration_ms;
  analogWrite(BUZZER_PIN, 200);
  delay(stopBeep);
  analogWrite(BUZZER_PIN, 0);
  delay(stopBeep);
}
// ends here
// .
// .
// .
// .
// .
// Line sensor calibration routine starts here
void doCalibration() {
  pose_leader.update();
  set_calib_rout(12.56);

  // Apply calibration values, store in calibrated[]
  for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
    lsc.maximum[sensor] = 0;
    lsc.minimum[sensor] = 1023;
  }

  while (calibrating == true) {
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      lsc.calcCalibratedADC();
      if (lsc.readings[sensor] > lsc.maximum[sensor]) {
        lsc.maximum[sensor] = lsc.readings[sensor];
      }
      if (lsc.readings[sensor] < lsc.minimum[sensor]) {
        lsc.minimum[sensor] = lsc.readings[sensor];
      }
    }
    delay(10);
    check_calib_rout();
  }

  for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
    lsc.scaling[sensor] = lsc.maximum[sensor] - lsc.minimum[sensor];
  }
}
// ends here
// .
// .
// .
// .
// .
// Non-blocking calibration routine motors rotation function starts here
void set_calib_rout(float angle) {
  pose_leader.update();
  pid_n_enc(0.40, -0.40);
  target_angle = pose_leader.theta + angle;
  calibrating = true;
}

void check_calib_rout() {
  pose_leader.update();
  float angle_diff = target_angle - pose_leader.theta;
  if (fabs(angle_diff) < TURN_THRESHOLD) {
    pid_n_enc(0, 0);
    calibrating = false;
  } else {
    // do nothing
  }
}
// ends here
// .
// .
// .
// .
// .
void setReturn(float x, float y) {
  pose_leader.update();
  float target_angle = atan2((y - pose_leader.y), (x - pose_leader.x));
  float normal = atan2(sin(target_angle - pose_leader.theta), cos(target_angle - pose_leader.theta));
  if (normal > 0) {
    set_turn_pwm(-30, 30, normal);
  } else if (normal < 0) {
    set_turn_pwm(30, -30, normal);
  }
  return_home = true;
}

bool checkReturn(float x, float y) {
  pose_leader.update();
  while (turn == true) {
    check_turn_pwm();
  }
  x_home = pose_leader.x;
  y_home = pose_leader.y;
  theta_home = pose_leader.theta;

  if (x_home < x && y_home > y) {
    motors.setPWM(-50, -50);
    if (x - pose_leader.x > 0.5 && y - pose_leader.y < 0.5) {
      //motors.setPWM(0, 0);
      return_home = false;
      return false;
    }
  } else if (x_home > x && y_home < y) {
    motors.setPWM(50, 50);
    if (x - pose_leader.x > 0.5 && pose_leader.y - y > 0.5) {
      //motors.setPWM(0, 0);
      return_home = false;
      return false;
    }
  }
  return_home = true;
  return true;
}
// Ends here
// .
// .
// .
// .
// .
// Low Pass filter functions
void lowPassL(float xtL) {
  smoothL = (aL * xtL) + ((1.0 - aL) * stL);
  stL = smoothL;
}
void lowPassR(float xtR) {
  smoothR = (aR * xtR) + ((1.0 - aR) * stR);
  stR = smoothR;
}
// Ends here
// .
// .
// .
// .
// .
// PID and wheel speed calculation function
void pid_n_enc(float l_demand, float r_demand) {

  float timeDiff = millis() - speed_est_ts;

  if (timeDiff > speed_est_ms) {

    // Work out the difference in encoder counts
    long count_difference = last_e0 - count_e0;
    long count_difference1 = last_e1 - count_e1;

    last_e0 = count_e0;
    last_e1 = count_e1;

    speed_e0 = float(count_difference) / float(timeDiff);
    speed_e1 = float(count_difference1) / float(timeDiff);

    lowPassL(speed_e1);
    lowPassR(speed_e0);

    speed_est_ts = millis();
  }

  if (millis() - pid_ts > pid_ms) {

    float l_pwm = left_pid.update(l_demand, smoothL);
    float r_pwm = right_pid.update(r_demand, smoothR);

    motors.setPWM(l_pwm, r_pwm);
  }

  if (millis() - pose_leader_ts > pose_leader_ms) {
    pose_leader.update();
  }

  // clearGraphics();
  // display.noAutoDisplay();
  // display.clear();
  // display.print("X = ");
  // display.gotoXY(4, 0);
  // display.print(pose_leader.x);
  // display.gotoXY(0, 1);
  // display.print("Y = ");
  // display.gotoXY(4, 1);
  // display.print(pose_leader.y);
  // display.gotoXY(0, 2);
  // display.print("T = ");
  // display.gotoXY(4, 2);
  // display.print(pose_leader.theta);
  // display.display();
}
// Ends here
// .
// .
// .
// .
// .
// setup starts here
void setup() {

  Serial.begin(9600);
  Serial.println("\n\n *** READY ***");

  pinMode(aPin, INPUT_PULLUP);
  pinMode(dPin, INPUT_PULLUP);
  pinMode(emit, OUTPUT);

  display.setLayout11x4WithGraphics(graphics);

  clearGraphics();
  display.noAutoDisplay();
  display.clear();
  display.print(F("** READY **"));
  display.display();

  pinMode(BUZZER_PIN, OUTPUT);

  motors.initialise();

  pose_leader.initialise(0, 0, 0);

  lsc.initialiseForADC();

  pose_leader_ms = 20;
  pose_leader_ts = millis();

  ts_task = millis();
  ms_task = 10;

  setupEncoder0();
  setupEncoder1();

  last_e0 = count_e0;
  last_e1 = count_e1;

  speed_e0 = 0.0;
  speed_e1 = 0.0;

  speed_est_ms = 10;
  speed_est_ts = millis();


  // PID gains (Kp, Ki, Kd). NOTE: with the PID.h D-term bug fixed, Kd is now LIVE.
  // The system was demonstrated/tuned with the D term effectively disabled, so Kd is
  // set to 0 here to reproduce that known-good (PI) behaviour. Re-tune on hardware
  // before raising Kd (the old value of 200 was never actually applied).
  left_pid.initialise(55, 0.03, 0);
  right_pid.initialise(60, 0.03, 0);

  pid_ms = 50;
  pid_ts = millis();

  left_pid.reset();
  right_pid.reset();

  // buzzer beeps sequence
  beep_sequence();

  delay(4000);
  motors.setPWM(0, 0);
}
// void setup ends here
// .
// .
// .
// .
// .
// void loop starts here
void loop() {
  if (millis() - pose_leader_ts > pose_leader_ms) {
    pose_leader.update();
    pose_leader_ts = millis();
  }
  if (millis() - ts_task > ms_task) {
    digitalWrite(emit, LOW);
    delay(bit_duration);
    digitalWrite(emit, HIGH);
    delay(bit_duration);
    ts_task = millis();
  }
  pid_n_enc(-0.12, -0.12);
  delay(6000);
  motors.setPWM(0, 0);
  delay(1000);
  pid_n_enc(0.12, 0.12);
  delay(6000);
  motors.setPWM(0, 0);
  delay(1000);
}
// void loop ends here