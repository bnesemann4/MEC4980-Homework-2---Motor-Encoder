#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

Adafruit_ST7789 disp = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const int motor1 = 9;
const int motor2 = 10;
const int encoderPin = A0;
const int d0 = 0;
const int d1 = 1;
const int d2 = 2;
const int pulsePerRev = 4;
int lightMeasure = 0;
int lightBarrier = 2000;
int lightHysteresis = 300;
bool gapState = false;
unsigned long prevPulse = 0;
unsigned long prevDispTime = 0;
unsigned long prevButTime = 0;
const unsigned long debounceTime = 200;
double targetRPM = 60.;
double motorRPM = 0.;
double motorPWM = 0.;
double KP = 0.7;
double KI = 0.3;
double KD = 0.0;

PID motorPID(
  &motorRPM,
  &motorRPM,
  &motorRPM,
  KP,
  KI,
  KD,
  DIRECT
);

bool motorOn = false;
int d0Prev = HIGH;
int d1Prev = LOW;
int d2Prev = LOW;
void encoderRead();
void butCheck();
void runMotor();
void stopMotor();
void dispScreen();

void setup() {
  Serial.begin(115200);

  pinMode(TFT_I2C_POWER,OUTPUT);
  digitalWrite(TFT_I2C_POWER,HIGH);
  
  pinMode(TFT_BACKLITE,OUTPUT);
  digitalWrite(TFT_BACKLITE,HIGH);

  disp.init(135,240);
  disp.setRotation(1);
  disp.fillScreen(ST77XX_WHITE);
  disp.setTextColor(ST77XX_BLUE);

  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  analogWrite(motor1,0);
  analogWrite(motor2,0);

  pinMode(encoderPin,INPUT);

  pinMode(d0,INPUT_PULLUP);
  pinMode(d1,INPUT_PULLDOWN);
  pinMode(d2,INPUT_PULLDOWN);

  motorPID.SetOutputLimits(0,255);
  motorPID.SetSampleTime(200);
  motorPID.SetMode(MANUAL);

  dispScreen();
}

void loop() {
  encoderRead();
  butCheck();

  if (motorOn) {
    motorPID.Compute();
    runMotor();
  } else {
    stopMotor();
  }

  if (millis() - prevDispTime >= 500) {
    prevDispTime = millis();
    dispScreen();
    Serial.print("Target motor speed: ");
    Serial.print(targetRPM);
    Serial.println(" RPM.");
    Serial.print("Current motor speed: ");
    Serial.print(motorRPM);
    Serial.println(" RPM.");
    Serial.print("PWM: ");
    Serial.println(motorPWM);
    Serial.print("Light reading: ");
    Serial.print(lightMeasure);
  }
}

void encoderRead() {
  lightMeasure = analogRead(encoderPin);

  if (!gapState && lightMeasure > lightBarrier + lightHysteresis) {
    gapState = true;
    unsigned long pulseTime = micros();

    if (prevPulse != 0) {
      unsigned long pulseDuration = pulseTime - prevPulse;
      double newRPM = 60000000.0/(pulseDuration*pulsePerRev);
      motorRPM = 0.8*motorRPM+0.2*newRPM;
    }

    prevPulse = pulseTime;
  }

  if (gapState && lightMeasure < lightBarrier - lightHysteresis) {
    gapState = false;
  }

  if (prevPulse != 0 && micros() - prevPulse > 1000000) {
    motorRPM = 0;
  }
}

void butCheck() {
  unsigned long currentTime = millis();
  int d0Cur = digitalRead(d0);
  int d1Cur = digitalRead(d1);
  int d2Cur = digitalRead(d2);

  if (d0Cur == LOW && d0Prev == HIGH && currentTime - prevButTime > debounceTime) {
    prevButTime = currentTime;
    motorOn = !motorOn;

    if (motorOn) {
      motorPWM = 0;
      motorPID.SetMode(AUTOMATIC);
    } else {
      motorPID.SetMode(MANUAL);
      motorPWM = 0;
    }
  }

  if (d1Cur == HIGH && d1Prev == LOW && currentTime - prevButTime > debounceTime) {
    prevButTime = currentTime;
    targetRPM += 10;

    if (targetRPM > 150) {
      targetRPM = 150;
    }
  }

  if (d2Cur == HIGH && d2Prev == LOW && currentTime - prevButTime > debounceTime) {
    prevButTime = currentTime;
    targetRPM -= 10;

    if (targetRPM < 50) {
      targetRPM = 50;
    }
  }

  d0Prev = d0Cur;
  d1Prev = d1Cur;
  d2Prev = d2Cur;
}

void runMotor() {
  int PWMReading = constrain((int)motorPWM,0,255);
  analogWrite(motor1,PWMReading);
  analogWrite(motor2,0);
}

void stopMotor() {
  analogWrite(motor1,0);
  analogWrite(motor2,0);
}

void dispScreen() {
  disp.fillScreen(ST77XX_WHITE);
  disp.setTextColor(ST77XX_BLUE);
  disp.setTextSize(1.5);
  disp.setCursor(10,15);
  disp.println("Motor Options: ");
  disp.setCursor(10,35);
  disp.print("Motor Speed Setting: ");
  disp.print(targetRPM,0);
  disp.setCursor(10,55);
  disp.print("Current Motor Speed: ");
  disp.print(motorRPM,1);
  disp.setCursor(10,75);
  disp.print("PWM: ");
  disp.print(motorPWM,0);
  disp.setTextSize(1);
  disp.setCursor(10,85);
  disp.print("Light Reading: ");
  disp.print(lightMeasure);
  disp.setCursor(10,95);

  if (motorOn) {
    disp.print("Motor currently running.");
  } else {
    disp.print("Motor currently off.");
  }

  disp.setTextSize(0.5);
  disp.setCursor(10,105);
  disp.setTextColor(ST77XX_BLUE);
  disp.print("Press D0 to Start the Motor, D1 to ");
  disp.setCursor(10,115);
  disp.print("Increase Speed, and D2 to Decrease");
  disp.setCursor(10,125);
  disp.print("Speed.");
}