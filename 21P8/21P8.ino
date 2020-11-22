#include <Servo.h>

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define _INTERVAL_DIST    25
#define _INTERVAL_SERVO   20
#define _INTERVAL_SERIAL  20
#define _DIST_MIN 100
#define _DIST_MAX 400
#define _DIST_ALPHA 0.1

#define _DUTY_MIN 1460
#define _DUTY_NEU 1600
#define _DUTY_MAX 1750

float timeout;
float dist_min, dist_max, dist_center, dist_raw, dist_prev, dist_ema, alpha;
float scale;
Servo myservo;
unsigned long last_sampling_time, last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

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
  
  Serial.begin(57600);

  last_sampling_time = last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  a = 68;
  b = 268;
}

float ir_distance(void){
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + _INTERVAL_DIST) return;

  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  dist_ema = (alpha*dist_cali) + ((1-alpha)*dist_ema);
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.println(myservo.read());

  if(dist_ema < 255) myservo.writeMicroseconds(_DUTY_MAX);
  else if(dist_ema == 255) myservo.writeMicroseconds(_DUTY_NEU);
  else myservo.writeMicroseconds(_DUTY_MIN);
  
}
