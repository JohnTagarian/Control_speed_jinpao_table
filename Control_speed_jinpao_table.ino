#define ENCA PA10
#define ENCB PA9
#define PWM_PIN PA0
#define IN1 PA1
#define IN2 PA2


const int PPR = 2048;

// globals
long prevT = 0;
int posPrev = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float eintegral = 0;

float kp = 2.0;
float ki = 19.35;

float target_rpm = 7.5;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  float v2 = velocity2 / PPR * 60.0;
  float e = target_rpm - fabs(v2);
  
  eintegral = eintegral + e * deltaT;
  float u = kp * e + ki * eintegral;

  int pwr = (int)(u);
  if (pwr > 255) {
    pwr = 255;
  }
  if(pwr <0){
    pwr = 0;
  }
  
  setMotor(1, pwr, PWM_PIN, IN1, IN2);
  Serial.print("Target : " + String(target_rpm));
  Serial.print("  Error : " + String(e));
  Serial.print("  RPM : " + String(v2));
  Serial.print("  Control : "+String(pwr));
  Serial.println("SUM : "+String(eintegral));

}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Motor speed
  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  //   Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i)) / 1.0e6;
  velocity_i = increment / deltaT;
  prevT_i = currT;
}
