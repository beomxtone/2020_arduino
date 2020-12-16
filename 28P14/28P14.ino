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

#define EMA_ALPHA 0.2

#define _DUTY_MIN 1460
#define _DUTY_NEU 1600
#define _DUTY_MAX 1700

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 800

#define _KP 0.8
#define _KD 15
#define _KI 17.0
#define _ITERM_MAX 99.0

#define DELAY_MICROS  1500

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

float samples_num = 3;
float ema_dist = 0;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = EMA_ALPHA;
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
  float x;
  float volt = float(analogRead(PIN_IR));
  x = ((6762.0/(volt-9.0))-4.0) * 10.0;
  const float coE[] = {0.0000074, -0.0004957, 0.9819665, 40.6514622};
  return coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
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
      filtered_dist = filtered_ir_distance();  //[0028] 적외선 센서 필터링 값 저장

  // PID control logic
    error_curr = _DIST_TARGET - filtered_dist; //[0028] 현재 오차 저장
    pterm = error_curr * _KP; //[0028] kp * 오차
    dterm = (error_curr - error_prev) * _KD;
    iterm += error_curr * _KI;
    
    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX;
    
    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;

    control_prev = control;
    error_prev = error_curr;
  
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
    Serial.print("IR:");
    Serial.print(filtered_dist);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    last_sampling_time_serial = millis();
 // [3133] 마지막 serial event 처리 시각 기록

  }
}
