#include <Servo.h>

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.1

#define _DUTY_MIN 1450
#define _DUTY_NEU 1600
#define _DUTY_MAX 1700

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _KP 1.2

float timeout;
float dist_min, dist_max, dist_center, dist_raw, dist_prev, dist_ema, alpha;
float filtered_dist;
float dist_target;
float scale;
Servo myservo;
unsigned long last_sampling_time, last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

float error_curr, error_prev, control, control_prev, control_filter, pterm, dterm, iterm;

float duty_chg_per_interval;
int duty_target, duty_curr, duty_filtered;
int a, b;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  dist_center = _DIST_MIN + _DIST_MAX / 2.0;
  timeout = (_INTERVAL_DIST / 2) * 1000.0;
  dist_raw = dist_prev = 0.0;
  scale = 0.001 * 0.5 * SND_VEL;

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
  
  Serial.begin(115200);

  last_sampling_time = last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  a = 70;
  b = 270;
  control_prev = 0;
}

float ir_distance(void){
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered() {
  const float coE[] = {0.0000074, -0.0004957, 0.9819665, 40.6514622};
  float x = ir_distance();
  return coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
}

void loop() {
/////////////////////
// Event generator // [3133] 이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = false; // [3133]
  // get a distance reading from the distance sensor
      dist_raw = ir_distance();
      filtered_dist = ir_distance_filtered();  //[0028] 적외선 센서 필터링 값 저장

  // PID control logic
    error_curr = _DIST_TARGET - filtered_dist; //[0028] 현재 오차 저장
    pterm = error_curr * _KP; //[0028] kp * 오차
    control = constrain(pterm, -110, 110);

    control_filter = (alpha*control) + ((1-alpha)*control_filter);
    control = control_filter;

  // duty_target = f(duty_neutral, control)
    if (control < control_prev && control <= 30) {
      duty_target = map(control, -110, 30, _DUTY_MIN, _DUTY_NEU-30);
    } else if (control > control_prev && control >= -30) {
      duty_target = map(control, -30, 110, _DUTY_NEU+30, _DUTY_MAX);
    } else {
      duty_target = map(control, -30, 30, _DUTY_NEU-5, _DUTY_NEU+5);
    }

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;

    control_prev = control;
  
    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; // [3133]
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    duty_curr = duty_target;
    // update servo position
    duty_filtered = (alpha*duty_curr) + ((1-alpha)*duty_filtered);
    myservo.writeMicroseconds(duty_curr);
    last_sampling_time_servo = millis(); // [3133] 마지막 servo event 처리 시각 기록

  }
  
  if(event_serial) {
    event_serial = false; // [3133]
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print("filtered_dist:");
    Serial.print(filtered_dist);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.print(",duty_filtered:");
    Serial.print(duty_filtered);
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis(); // [3133] 마지막 serial event 처리 시각 기록

  }
}
