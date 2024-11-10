#include "structs_and_functions.h"
#define bitSet(value, bit) ((value) |= (1UL << (bit)))

#define yellow_pin 14   // 14-16 are motor phase pins 
#define red_pin 15
#define blue_pin 16
#define direction_pin 17
#define throttle_pin 18

#define pid_Kp 3.5
#define pid_Ki 5.7
#define pid_Kd 0
#define debug_pid 0

#define KF_Fk 1
#define KF_Bk 1/24
#define KF_Qk 1       
#define KF_Hk 1
#define KF_Rk 0.7




byte state = 0;
byte prev_state = 0b001;
bool y = 0;
bool g = 0;
bool b = 0;
// const int readTH = 800; // Analog signal Threshold.
const int steps = 240; // Hall sensor number of pulses for one revolution.
int count = 0;
int direction = 1; // Engine spining direction.
const int durationEncoder = 50; // All durations are in (ms)
const int durationPub = 100; // Estimate speed publish rate 10(Hz)
const int durationKF = 10;
const int durationPID = 200;

unsigned long prevTimeEncoder = 0; // time in milliseconds.
unsigned long prevTimePub = 0; // time in milliseconds. Used for write engine vel_cmd.
unsigned long prevTimeKF = 0; // time in milliseconds.
unsigned long prevTimePID = 0; // time in milliseconds.

const float wheel_radius = 0.24; // (m)
volatile float speed = 0; // wheel gruond contect speed (m/s)

float target_speed = 0;
int command = 40;

PID velPid;
KalmanFilter velKF;

char serial_input[32]; //0x3FC00000
float input = 0;

// the setup routine runs once when you press reset:
void setup() {                
    Serial.begin(57600);
  // initialize pins
  pinMode(yellow_pin, INPUT);
  pinMode(red_pin, INPUT);
  pinMode(blue_pin, INPUT);
  pinMode(direction_pin, OUTPUT);
  pinMode(throttle_pin, OUTPUT);
  digitalWrite(direction_pin, LOW);
  analogWrite(throttle_pin, command);
  attachInterrupt(digitalPinToInterrupt(yellow_pin), hall_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(red_pin), hall_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(blue_pin), hall_callback, CHANGE);
  
  // define pid{float Kp, float Ki, float Kd, float Kaw, float T_C, float max, float min, float max_change}
  velPid = pidCreate(pid_Kp, pid_Ki, pid_Kd, // kp, ki, kd
            0, 0, 250, // Kaw, T_c, max
            0, 50);   // min, max_change

  velKF = KF_Create(KF_Fk, KF_Bk, KF_Qk, KF_Hk, KF_Rk); 
  analogWrite(throttle_pin, 0);  
  Serial.println("Setup complete");
}

void loop() {

  if ((millis() - prevTimeEncoder) > durationEncoder){
      speed = wheel_radius * TWO_PI * count * 1000.0 / (steps * (millis() - prevTimeEncoder)); 
      count = 0;
      prevTimeEncoder = millis();
      KF_update(&velKF, speed);
      printToSerial(speed);
      }

    if ((millis() - prevTimeKF) > durationKF){
      KF_predict(&velKF, velPid.delta_cmd);
      // Serial.print("\nvelKF.X_k: ");
      // Serial.print(velKF.X_k);
      // Serial.print(" target_speed: ");
      // Serial.println(target_speed);
      
      prevTimeKF = millis();
    }
    if ((millis() - prevTimePub) > durationPub){
      printToSerial(velKF.X_k);
      prevTimePub = millis();
    }
    
    if ((millis() - prevTimePID) > durationPID){
      command = (int) PID_Step(&velPid, velKF.X_k, target_speed, (millis() - prevTimePID)/1000.0);
      if (debug_pid){
        Serial.print("\nestimate X_k: ");
        Serial.print(velKF.X_k);
        Serial.print(" Enc_speed: ");
        Serial.print(speed);
        Serial.print(" target_speed: ");
        Serial.print(target_speed);
        Serial.print(" command: ");
        Serial.println(command);
        Serial.print(" p: ");
        Serial.print(velPid.err_prev * velPid.Kp);
        Serial.print(" i: ");
        Serial.print(velPid.integral);
        Serial.print(" d: ");
        Serial.println(velPid.deriv_prev * velPid.Kd);
      }

      analogWrite(throttle_pin, command);
      prevTimePID = millis();
    }

}

void serialEvent() {
  Serial.readBytesUntil('\n',serial_input, 32);
  target_speed = atof(serial_input);
}

void printToSerial(float msg){
    
    Serial.write("<");
    Serial.print(msg);
    Serial.write(">");
}

void serialFlush(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}

int8_t get_direction(){
  
  byte xorValue = state ^ prev_state;
  if (!xorValue || state == 0b111 || state == 0b000) return 0; // True if there isn't change in the state
  if (xorValue & prev_state){ // True if the CHANGED signal is LOW
    if ((xorValue << 1) % 7 == state) return -1;
    else if (xorValue >> 1 == state) return 1;
    else if (prev_state == 0b101 && state == 0b100) return 1;
    return 0;
  }
  if (((prev_state<<1) % 7 | prev_state)   == state) return -1;
  else if (((prev_state >> 1) | prev_state) == state) return 1;
  else if (prev_state == 0b001 && state == 0b101) return 1; 
  return 0; 
}

void hall_callback(){
  y = digitalRead(yellow_pin);
  b = digitalRead(blue_pin);
  g = digitalRead(red_pin);
  
  state = (y<<2) + (g<<1) + b;

  if (state==prev_state){
    return;
  } 
  direction = get_direction();
  
  if (direction == 0) return;
  count += direction;
  prev_state = state;  
}