#include<math.h>
#include<SoftwareSerial.h>
#include "NewPing.h"
//motor connections
#define IN1 8
#define IN2 7
#define IN3 6
#define IN4 4
#define ENA 9
#define ENB 5
//ultrasonic sensors
#define trigPin1 11
#define echoPin1 13
#define trigPin2 3
#define echoPin2 2
//bluetooth module
#define rxPin 12
#define txPin 10

#define MAX_DISTANCE 400
 
NewPing sonar1(trigPin1, echoPin1, MAX_DISTANCE);
NewPing sonar2(trigPin2, echoPin2, MAX_DISTANCE);
SoftwareSerial BTSerial(rxPin, txPin);
 
int xAxis = 125, yAxis = 125;
int motorSpeedA, motorSpeedB;

//Maximum speed at an error of 7 cm
float duration, duration1, distance, distance1;
float integral = 0, error, derivate, prev_error, output;
float integral1 = 0, error1, derivate1, prev_error1, output1;
float  k_p = 25;
float k_i = 7;
float k_d = 0.1;
float setpoint = 7;
float interval=0.001;

void setup() {
  //motor outputs
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  //set motors off 
  motorStop();
  analogWrite(ENA,0);
  analogWrite(ENB,0);

  // Distance sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  //BT Module
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  BTSerial.begin(115200);
}

void loop() {
  int mode = analogRead(A0);
  if(mode<511){
    //Bluetooth Mode
    while(BTSerial.available() >= 2) {
      xAxis = BTSerial.read();
      delay(10);
      yAxis = BTSerial.read();
    }
    //Joystick logic
    if(xAxis >=0 && yAxis >= 0 && xAxis<=250 && yAxis<=250){
      if(yAxis > 150 && yAxis < 251){
          setBackward();
          if(xAxis>135){
            motorSpeedB = map(yAxis, 125 , 250, 0, 255);
            motorSpeedA = motorSpeedB - floor((250 - xAxis)/1.8);
          }else if (xAxis<115){
            motorSpeedA = map(yAxis, 125 , 250, 0, 255);
            motorSpeedB = motorSpeedA - floor((250 - xAxis)/1.8);
          }else{
            motorSpeedA = map(yAxis, 125 , 250, 0, 255);
            motorSpeedB = motorSpeedA;
          }
      }else if(yAxis > -1 && yAxis < 101){
          setForward();
          if(xAxis>135){
            motorSpeedA = map(yAxis, 125, 0, 0, 255);
            motorSpeedB = motorSpeedA - floor((250 - xAxis)/1.8);
          }else if (xAxis<115){
            motorSpeedB = map(yAxis, 125, 0, 0, 255);
            motorSpeedA = motorSpeedB - floor((250 - xAxis)/1.8);
          }else{
            motorSpeedA = map(yAxis, 125, 0, 0, 255);
            motorSpeedB = motorSpeedA;
          }
        }else if(yAxis > 100 && yAxis < 150){
          if(xAxis > 140){
            setLeft();
            motorSpeedA = map(xAxis, 125, 250, 0, 160);
            motorSpeedB = motorSpeedA;
          }else if (xAxis < 115 ){
            setRight();
            motorSpeedA = map(xAxis, 125, 0, 0, 160);
            motorSpeedB = motorSpeedA;
          }else{
            motorStop();
            motorSpeedA = 0;
            motorSpeedB = 0;
          }
        }
        analogWrite(ENB,motorSpeedB);
        analogWrite(ENA,motorSpeedA);
    } 
  }else
  { // Autonomous Mode
    // Determine distance from duration
    duration = sonar1.ping();
    duration1 = sonar2.ping();

    distance = (duration / 2) * 0.0343;
    distance1 = (duration1 / 2) * 0.0343;
    
    if (distance < 40 && distance >2){
      error = distance - setpoint;
      integral = integral + (error * interval);
      derivate = (error - prev_error)/interval;
      output = (error * k_p) + (integral * k_i) + (derivate * k_d);

      if(abs(error)<0.6){
        error = 0;
        integral = 0;
        output = 0; 
        //stop motor
      }

      prev_error = error;

      if(output>0){
        //set motor forward
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        if(abs(output)>255){
          output = 255;
        }
        analogWrite(ENA,floor(output));
        
      }else if(output<0){
        //set motor backward
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        if(abs(output)>255){
          output = 255;
        }
        analogWrite(ENA,floor(abs(output)));
      }
      delay(interval*100);
    }
    if (distance1 < 40 && distance1 >2){
      error1 = distance1 - setpoint;
      integral1 = integral1 + (error1 * interval);
      derivate1 = (error1 - prev_error1)/interval;
      output1 = (error1 * k_p) + (integral1 * k_i) + (derivate1 * k_d);

      if(abs(error1)<0.6){
        error1 = 0;
        integral1 = 0;
        output1 = 0; 
        //stop motor
      }

      prev_error1 = error1;

      if(output1>0){
        //set motor forward
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        if(abs(output1)>255){
          output1 = 255;
        }
        analogWrite(ENB,floor(output1));

      }else if(output1<0){
        //set motor backward
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
        if(abs(output1)>255){
          output1 = 255;
        }
        analogWrite(ENB,floor(abs(output1)));
      }
      delay(interval*100);
    }
  }
}

void setForward(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void setBackward(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void setRight(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void setLeft(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void motorStop(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

