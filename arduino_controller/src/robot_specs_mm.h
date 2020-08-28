#ifndef ROBOT_SPECS_MM_H
#define ROBOT_SPECS_MM_H

#define ENCODER_LEFT_PIN 21  // Encoder Left PIN
#define ENCODER_RIGHT_PIN 20  // Encoder Right PIN
#define ENCODER_PULSE_LEFT 2.5
#define ENCODER_PULSE_RIGH 2.5
#define ENCODER_FILTER 5

#define L 0.42 // distance between axes m
#define R 0.0775 // wheel radius m
#define PI 3.1415926

// Definition of Ultrasound 1
#define ECHOPIN_1 46
#define TRIGPIN_1 47

// Definition of Ultrasound 2
#define ECHOPIN_2 48
#define TRIGPIN_2 49

// Definition of Ultrasound 3
#define ECHOPIN_3 30
#define TRIGPIN_3 31

// Definition of Ultrasound 4
#define ECHOPIN_4 33
#define TRIGPIN_4 32

// Definition of Shield Moster
#define BRAKE 0
#define CW    1
#define CCW   2
#define CS_THRESHOLD 15   // Definition of safety current (Check: "1.3 Monster Shield Example").

//MOTOR 1 (Right)
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

//MOTOR 2 (Left)
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9

#define PWM_MOTOR_LEFT_PIN 5
#define PWM_MOTOR_RIGHT_PIN 6

#define EN_PIN_1 A0
#define EN_PIN_2 A1

#define CURRENT_SEN_1 A2
#define CURRENT_SEN_2 A3

#define MOTOR_LEFT_1 0          // Motor Left PID Controll
#define MOTOR_RIGHT_2 1         // Motor Right PID Controll

#endif

#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

//Function that controls the variables: motor(left(1) ou right(2)), direction (cw ou ccw) e pwm (entra -255 e 255);
void motorGo(uint8_t motor, float pwm)         
{

  int direct;

  if(pwm > 0){
  	direct = CW;	
  }else if(pwm < 0){
  	direct = CCW;
  }else{
  	direct = BRAKE;
  }

  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);

  if(motor == MOTOR_LEFT_1) // 0
  {
    if(direct == CW) //Forward CW
    {
      digitalWrite(MOTOR_A1_PIN, LOW);//7
      digitalWrite(MOTOR_B1_PIN, HIGH);//8
    }
    else if(direct == CCW) //Reverse CCW
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);//7
      digitalWrite(MOTOR_B1_PIN, LOW);//8  
    }
    else //BRAKE
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_LEFT_PIN, abs(pwm)); 
  }

  else if(motor == MOTOR_RIGHT_2) // 1
  {
    if(direct == CW) //Forward CW
    {
      digitalWrite(MOTOR_A2_PIN, LOW); //4
      digitalWrite(MOTOR_B2_PIN, HIGH); //9
    }
    else if(direct == CCW) //Reverse CCW
    {
      digitalWrite(MOTOR_A2_PIN, HIGH); //4
      digitalWrite(MOTOR_B2_PIN, LOW);  //9
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_RIGHT_PIN, abs(pwm));
  }
}