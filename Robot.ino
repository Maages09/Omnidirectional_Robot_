#include <Arduino.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial BT;

//Servo y sensor
Servo myservo;

#define echoPin1 22
#define trigPin1 23

// Define motor pins
#define ENABLE_PIN_1 13
#define MOTOR_PIN_1A 12
#define MOTOR_PIN_1B 14

#define ENABLE_PIN_2 25
#define MOTOR_PIN_2A 26
#define MOTOR_PIN_2B 27

#define ENABLE_PIN_3 2
#define MOTOR_PIN_3A 4
#define MOTOR_PIN_3B 5

#define ENABLE_PIN_4 21
#define MOTOR_PIN_4A 19
#define MOTOR_PIN_4B 18

#define MAX_SPEED 190

#define LEDC_TIMER_FREQ 5000
#define LEDC_TIMER_RESOLUTION 8

// Define motor states
bool motorsRunning = false;

long duration, distance;

void setup() {
  //Sensor and Servo
  Serial.begin(115200);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  myservo.attach(15);

  // Set pins and initialize PWM
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_3, OUTPUT);
  pinMode(ENABLE_PIN_4, OUTPUT);
  pinMode(MOTOR_PIN_1A, OUTPUT);
  pinMode(MOTOR_PIN_1B, OUTPUT);
  pinMode(MOTOR_PIN_2A, OUTPUT);
  pinMode(MOTOR_PIN_2B, OUTPUT);
  pinMode(MOTOR_PIN_3A, OUTPUT);
  pinMode(MOTOR_PIN_3B, OUTPUT);
  pinMode(MOTOR_PIN_4A, OUTPUT);
  pinMode(MOTOR_PIN_4B, OUTPUT);

  // SET PWM
  ledcSetup(0, LEDC_TIMER_FREQ, LEDC_TIMER_RESOLUTION);
  ledcAttachPin(ENABLE_PIN_1, 0);
  ledcAttachPin(ENABLE_PIN_2, 0);
  ledcAttachPin(ENABLE_PIN_3, 0);
  ledcAttachPin(ENABLE_PIN_4, 0);

  // START MOTORS
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);

  BT.begin("ESP32_Motor_Control");
}

void Forward(int SPD) {
  // MOTOR 1
  digitalWrite(MOTOR_PIN_1A, LOW);
  digitalWrite(MOTOR_PIN_1B, HIGH);
  ledcWrite(0, SPD);
  
  // MOTOR 2
  digitalWrite(MOTOR_PIN_2A, LOW);
  digitalWrite(MOTOR_PIN_2B, HIGH);
  ledcWrite(1, SPD);
  
  // MOTOR 3
  digitalWrite(MOTOR_PIN_3A, LOW);
  digitalWrite(MOTOR_PIN_3B, HIGH);
  ledcWrite(2, SPD);
  
  // MOTOR 4
  digitalWrite(MOTOR_PIN_4A, LOW);
  digitalWrite(MOTOR_PIN_4B, HIGH);
  ledcWrite(3, SPD);
}

void Backward(int SPD) {
  // MOTOR 1
  digitalWrite(MOTOR_PIN_1A, HIGH);
  digitalWrite(MOTOR_PIN_1B, LOW);
  ledcWrite(0, SPD);
  
  // MOTOR 2
  digitalWrite(MOTOR_PIN_2A, HIGH);
  digitalWrite(MOTOR_PIN_2B, LOW);
  ledcWrite(1, SPD);
  
  // MOTOR 3
  digitalWrite(MOTOR_PIN_3A, HIGH);
  digitalWrite(MOTOR_PIN_3B, LOW);
  ledcWrite(2, SPD);
  
  // MOTOR 4
  digitalWrite(MOTOR_PIN_4A, HIGH);
  digitalWrite(MOTOR_PIN_4B, LOW);
  ledcWrite(3, SPD);
}

void Right(int SPD) {
  // MOTOR 1
  digitalWrite(MOTOR_PIN_1A, LOW);
  digitalWrite(MOTOR_PIN_1B, HIGH);
  ledcWrite(0, SPD);
  
  // MOTOR 2
  digitalWrite(MOTOR_PIN_2A, HIGH);
  digitalWrite(MOTOR_PIN_2B, LOW);
  ledcWrite(1, SPD);
  
  // MOTOR 3
  digitalWrite(MOTOR_PIN_3A, HIGH);
  digitalWrite(MOTOR_PIN_3B, LOW);
  ledcWrite(2, SPD);
  
  // MOTOR 4
  digitalWrite(MOTOR_PIN_4A, LOW);
  digitalWrite(MOTOR_PIN_4B, HIGH);
  ledcWrite(3, SPD);
}

void Left(int SPD) {
  // MOTOR 1
  digitalWrite(MOTOR_PIN_1A, HIGH);
  digitalWrite(MOTOR_PIN_1B, LOW);
  ledcWrite(0, SPD);
  
  // MOTOR 2
  digitalWrite(MOTOR_PIN_2A, LOW);
  digitalWrite(MOTOR_PIN_2B, HIGH);
  ledcWrite(1, SPD);
  
  // MOTOR 3
  digitalWrite(MOTOR_PIN_3A, LOW);
  digitalWrite(MOTOR_PIN_3B, HIGH);
  ledcWrite(2, SPD);
  
  // MOTOR 4
  digitalWrite(MOTOR_PIN_4A, HIGH);
  digitalWrite(MOTOR_PIN_4B, LOW);
  ledcWrite(3, SPD);
}

void RotateR(int SPD) {
  // MOTOR 1
  digitalWrite(MOTOR_PIN_1A, LOW);
  digitalWrite(MOTOR_PIN_1B, HIGH);
  ledcWrite(0, SPD);
  
  // MOTOR 2
  digitalWrite(MOTOR_PIN_2A, HIGH);
  digitalWrite(MOTOR_PIN_2B, LOW);
  ledcWrite(1, SPD);
  
  // MOTOR 3
  digitalWrite(MOTOR_PIN_3A, LOW);
  digitalWrite(MOTOR_PIN_3B, HIGH);
  ledcWrite(2, SPD);

  // MOTOR 4
  digitalWrite(MOTOR_PIN_4A, HIGH);
  digitalWrite(MOTOR_PIN_4B, LOW);
  ledcWrite(3, SPD);
}
void Stop(){
  ledcWrite(0,0);
  ledcWrite(1,0);
  ledcWrite(2,0);
  ledcWrite(3,0);
}

void RotateL(int SPD) {
  // MOTOR 1
  digitalWrite(MOTOR_PIN_1A, HIGH);
  digitalWrite(MOTOR_PIN_1B, LOW);
  ledcWrite(0, SPD);
  
  // MOTOR 2
  digitalWrite(MOTOR_PIN_2A, LOW);
  digitalWrite(MOTOR_PIN_2B, HIGH);
  ledcWrite(1, SPD);
  
  // MOTOR 3
  digitalWrite(MOTOR_PIN_3A, HIGH);
  digitalWrite(MOTOR_PIN_3B, LOW);
  ledcWrite(2, SPD);
  
  // MOTOR 4
  digitalWrite(MOTOR_PIN_4A, LOW);
  digitalWrite(MOTOR_PIN_4B, HIGH);
  ledcWrite(3, SPD);
}

int pos = 0;
int object = 0;

void loop() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  duration = pulseIn(echoPin1, HIGH);
  distance = duration / 58.2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(100);

  if (distance < 20 && object == 0) {
    Stop();
    object = 1;
  }

  if (BT.available()) {
    char command = BT.read();
    switch (command) {
      case 'w':
        object = 0;
        Stop();
        delay(500);
        Forward(150);  // Cambié el valor de 255 a 50
        break;

      case 's':
        object = 0;
        Stop();
        delay(500);
        Backward(150);  // Cambié el valor de 255 a 50
        break;

      case 'a':
        object = 0;
        Stop();
        delay(500);
        Left(150);  // Cambié el valor de 255 a 50
        break;

      case 'd':
        object = 0;
        Stop();
        delay(500);
        Right(150);  // Cambié el valor de 255 a 50
        break;

      case 'x':
        Stop();
        break;

      case 'l':
        object = 0;
        Stop();
        delay(500);
        RotateL(150);  // Cambié el valor de 200 a 50
        break;

      case 'r':
        object = 0;
        Stop();
        delay(500);
        RotateR(150);  // Cambié el valor de 200 a 50
        break;
    }
  }
}