//--GunterBot--
//  Connections to make
//   Arduino USB(Rx)->0   ->  Raspberry USB(Rx)
//   Arduino USB(Tx)->1   ->  Raspberry USB(Tx)
//
//   Arduino PIN 21       ->  Encoder Left
//   Arduino PIN 20       ->  Encoder Right
//
//   Arduino PIN 7        ->  MOTOR_A1_LEFT
//   Arduino PIN 8        ->  MOTOR_B1_LEFT
//   Arduino PIN 4        ->  MOTOR_A2_RIGHT
//   Arduino PIN 9        ->  MOTOR_B2_RIGHT
//
//   Arduino PIN 5        ->  PWM_MOTOR_LEFT
//   Arduino PIN 6        ->  PWM_MOTOR_RIGHT

#define LOOPTIME 200   // PID loop time(ms)

#include "robot_specs_mm.h"
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>

double pid_right = 0;
double pid_left = 0;

char encoder[] = "/encoder";
char sonar_1[] ="/sonar_1";
char sonar_2[] ="/sonar_2";
char sonar_3[] ="/sonar_3";
char sonar_4[] ="/sonar_4";

unsigned long lastMilli = 0;       // loop timing
double vel_req1;
double vel_req2;
double vel_act1;
double vel_act2;
int PWM_val1;
int PWM_val2;

int duration_1;
int duration_2;
int duration_3;
int duration_4;

//Variables of Encoder
double Sum_vel_Left = 0;
int cont_Left = 1;

double Sum_vel_Right = 0;
int cont_Right = 1;

//Left wheel encoder
volatile long encoder0Pos_Left = 0;
long encoder0PosAnt_Left = 0;

//Right wheel encoder
volatile long encoder0Pos_Right = 0;
long encoder0PosAnt_Right = 0;

double diffEncoder_Left = 0;
double diffEncoder_Right = 0;

//ROS Function - Angular and linear Velocity Desired
void handle_cmd(const geometry_msgs::Twist& msg){
  
  //V - linear velocity disired
  double x = msg.linear.x;

  //W - angular velocity disired
  double z = msg.angular.z;

  // Robot Differential Drive Kinematic
  vel_req1 = ( (2*x) - (z*L) )/2;  // Left wheel
  vel_req2 = ( (2*x) + (z*L) )/2;  // Right wheel
}

// *********************************************
ros::NodeHandle  nh;

// ROS msgs
geometry_msgs::Vector3Stamped vel_encoder_msg;
geometry_msgs::Twist vel_pid_msg;
sensor_msgs::Range sonar_msg1;
sensor_msgs::Range sonar_msg2;
sensor_msgs::Range sonar_msg3;
sensor_msgs::Range sonar_msg4;

//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &handle_cmd);

//Publisher
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_msg);

ros::Publisher pub_sonar1( "rangeSonar_1", &sonar_msg1);
ros::Publisher pub_sonar2( "rangeSonar_2", &sonar_msg2);
ros::Publisher pub_sonar3( "rangeSonar_3", &sonar_msg3);
ros::Publisher pub_sonar4( "rangeSonar_4", &sonar_msg4);

ros::Publisher pub_pid("/vel_pid", &vel_pid_msg);

// *********************************************

void setup()
{
  Serial.begin(57600); // MEGA_USB = 0 (RX), 1 (TX) -> RASPBERRY (USB)
  
  delay(1000);

  // PINs of Motor
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_LEFT_PIN, OUTPUT);
  pinMode(PWM_MOTOR_RIGHT_PIN, OUTPUT);

  pinMode(CURRENT_SEN_1, OUTPUT);
  pinMode(CURRENT_SEN_2, OUTPUT);  

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  //PINs of Encoder
  pinMode (ENCODER_LEFT_PIN, INPUT);
  pinMode (ENCODER_RIGHT_PIN, INPUT);

  //PINs of Ultrasound
  pinMode(ECHOPIN_1,INPUT);
  pinMode(TRIGPIN_1,OUTPUT);

  pinMode(ECHOPIN_2,INPUT);
  pinMode(TRIGPIN_2,OUTPUT);

  pinMode(ECHOPIN_3,INPUT);
  pinMode(TRIGPIN_3,OUTPUT);

  pinMode(ECHOPIN_4,INPUT);
  pinMode(TRIGPIN_4,OUTPUT);

  // Settings of Ultrasound 1
  sonar_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg1.header.frame_id =  sonar_1;
  sonar_msg1.field_of_view = (10.0/180.0) * 3.14;
  sonar_msg1.min_range = 0.05;
  sonar_msg1.max_range = 0.35;

  // Settings of Ultrasound 2
  sonar_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg2.header.frame_id =  sonar_2;
  sonar_msg2.field_of_view = (10.0/180.0) * 3.14;
  sonar_msg2.min_range = 0.05;
  sonar_msg2.max_range = 0.35;

  // Settings of Ultrasound 3
  sonar_msg3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg3.header.frame_id =  sonar_3;
  sonar_msg3.field_of_view = (10.0/180.0) * 3.14;
  sonar_msg3.min_range = 0.05;
  sonar_msg3.max_range = 0.35;

  // Settings of Ultrasound 4
  sonar_msg4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg4.header.frame_id =  sonar_4;
  sonar_msg4.field_of_view = (10.0/180.0) * 3.14;
  sonar_msg4.min_range = 0.05;
  sonar_msg4.max_range = 0.35;

  // PINs of Interrupt
  digitalWrite(ENCODER_LEFT_PIN, HIGH);
  digitalWrite(ENCODER_RIGHT_PIN, HIGH);
  attachInterrupt(2, call_encoder_Left, CHANGE); // Int.2 encoder left pin 21 (Interrupction - Arduino Mega)
  attachInterrupt(3, call_encoder_Right, CHANGE); // Int.3 encoder right pin 20 (Interrupction - Arduino Mega)

  encoder0Pos_Left = 0;
  encoder0Pos_Right = 0;
  encoder0PosAnt_Left = 0;
  encoder0PosAnt_Right = 0;

  vel_req1 = 0;
  vel_req2 = 0;
  vel_act1 = 0;
  vel_act2 = 0;
  PWM_val1 = 0;
  PWM_val2 = 0;

  // filter mean
  Sum_vel_Left = 0;
  cont_Left = 1;

  // filter mean
  Sum_vel_Right = 0;
  cont_Right = 1;

  // ROS Initialization with Publishers and Subscribers 
  nh.initNode();
  nh.subscribe(sub_rasp);
  nh.advertise(pub_encoder);
  nh.advertise(pub_sonar1);
  nh.advertise(pub_sonar2);
  nh.advertise(pub_sonar3);
  nh.advertise(pub_sonar4);
  nh.advertise(pub_pid);
}


void loop()
{

  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME){
    getMotorData(time-lastMilli);

    float getDistance_1 = sensor_ultra1();
    float getDistance_2 = sensor_ultra2();
    float getDistance_3 = sensor_ultra3();
    float getDistance_4 = sensor_ultra4();

    // float sinal1 = (vel_req1*255)/0.8;
    // float sinal2 = (vel_req2*255)/0.8;

    // PWM_val1 = constrain(sinal1, -255, 255);
    // PWM_val2 = constrain(sinal2, -255, 255);
    
    // Limitation of front movement of the robot through the ultrasonic sensor 1
    if( (getDistance_1 != 0 && getDistance_1 <= 0.25)){ 
      // front left and right engine limitation
      if(vel_req1 > 0 && vel_req2 > 0){
        PWM_val1 = updatePid(MOTOR_LEFT_1, 0, vel_act1);
        PWM_val2 = updatePid(MOTOR_RIGHT_2, 0, vel_act2);
      }else{
        PWM_val1 = updatePid(MOTOR_LEFT_1, vel_req1, vel_act1);
        PWM_val2 = updatePid(MOTOR_RIGHT_2, vel_req2, vel_act2);
      }
    }else{
      PWM_val1 = updatePid(MOTOR_LEFT_1, vel_req1, vel_act1);
      PWM_val2 = updatePid(MOTOR_RIGHT_2, vel_req2, vel_act2);
    }

    motorGo(MOTOR_LEFT_1, PWM_val1);
    motorGo(MOTOR_RIGHT_2, PWM_val2);

    publishDebugPid(time-lastMilli);
    publishVEL(time-lastMilli);
    lastMilli = time;
  }
}

// Publish Vel encoder- Function
void publishVEL(unsigned long time) {
  vel_encoder_msg.header.stamp = nh.now();
  vel_encoder_msg.header.frame_id = encoder;
  vel_encoder_msg.vector.x = vel_act1;  // encoder left rad/s
  vel_encoder_msg.vector.y = vel_act2;  // encoder right rad/s
  vel_encoder_msg.vector.z = double(time)/1000;  // time 200 millisenconds
  pub_encoder.publish(&vel_encoder_msg);
  nh.spinOnce();
}

// Publish Vel encoder- Function
void publishDebugPid(unsigned long time) {

  vel_pid_msg.linear.x = vel_req1;  // velocity reference left rad/s
  vel_pid_msg.linear.y = vel_act1;  // velocity encoder left rad/s
  vel_pid_msg.linear.z = pid_left;  // pid wheel left pwm

  vel_pid_msg.angular.x = vel_req2;  // velocity reference right rad/s
  vel_pid_msg.angular.y = vel_act2;  // velocity encoder left rad/s
  vel_pid_msg.angular.z = pid_right; // pid wheel right pwm

  pub_pid.publish(&vel_pid_msg);
  nh.spinOnce();
}

float sensor_ultra1(){

  float sensoReading = 0; 
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  digitalWrite(TRIGPIN_1,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_1,HIGH);
  delayMicroseconds(10);
    
  duration_1 = pulseIn(ECHOPIN_1, HIGH,2000);
  sensoReading = duration_1 *0.342/2000;
  //sensoReading = getDistance;
  sonar_msg1.range = sensoReading;
  //Serial.println(sensoReading);
  sonar_msg1.header.stamp = nh.now();
  pub_sonar1.publish(&sonar_msg1);

  return sensoReading;
}

float sensor_ultra2(){

  float sensoReading = 0; 
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  digitalWrite(TRIGPIN_2,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_2,HIGH);
  delayMicroseconds(10);
    
  duration_2 = pulseIn(ECHOPIN_2, HIGH,2000);
  sensoReading = duration_2 *0.342/2000;
  //sensoReading = getDistance;
  sonar_msg2.range = sensoReading;
  //Serial.println(sensoReading);
  sonar_msg2.header.stamp = nh.now();
  pub_sonar2.publish(&sonar_msg2);

  return sensoReading;
}

float sensor_ultra3(){

  float sensoReading = 0; 
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  digitalWrite(TRIGPIN_3,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_3,HIGH);
  delayMicroseconds(10);
    
  duration_3 = pulseIn(ECHOPIN_3, HIGH,2000);
  sensoReading = duration_3 *0.342/2000;
  //sensoReading = getDistance;
  sonar_msg3.range = sensoReading;
  //Serial.println(sensoReading);
  sonar_msg3.header.stamp = nh.now();
  pub_sonar3.publish(&sonar_msg3);

  return sensoReading;
}

float sensor_ultra4(){

  float sensoReading = 0; 
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:

  digitalWrite(TRIGPIN_4,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_4,HIGH);
  delayMicroseconds(10);
    
  duration_4 = pulseIn(ECHOPIN_4, HIGH,2000);
  sensoReading = duration_4 *0.342/2000;
  //sensoReading = getDistance;
  sonar_msg4.range = sensoReading;
  //Serial.println(sensoReading);
  sonar_msg4.header.stamp = nh.now();
  pub_sonar4.publish(&sonar_msg4);

  return sensoReading;
}

// Get the motor velocity with Encoder - Function
void getMotorData(unsigned long time)  {
  double dt = time * 0.001; // time to seg
  double w1 = (ENCODER_PULSE_LEFT * PI / 180);
  double w2 = (ENCODER_PULSE_RIGH * PI / 180);

  diffEncoder_Left = (encoder0Pos_Left-encoder0PosAnt_Left);
  diffEncoder_Right = (encoder0Pos_Right-encoder0PosAnt_Right);

  double vel_left = (double(diffEncoder_Left*w1*R)/double(dt));
  double vel_right = (double(diffEncoder_Right*w2*R)/double(dt));
  encoder0PosAnt_Left = encoder0Pos_Left;
  encoder0PosAnt_Right = encoder0Pos_Right;

  vel_act1 = filterLeft(vel_left);
  vel_act2 = filterRight(vel_right);

  // vel_act1 = vel_left;
  // vel_act2 = vel_right;
}

double filterLeft(double vel_left)  {

  //Mean of velocity in 5(ENCODER_FILTER) interations if encoder good
  cont_Left++;

  Sum_vel_Left = Sum_vel_Left + vel_left;
  double filter = Sum_vel_Left / cont_Left;

  if(cont_Left>ENCODER_FILTER){
    Sum_vel_Left = filter;
    cont_Left = 1;
  }

  if(diffEncoder_Left ==0) filter = 0;

  return filter;
}

double filterRight(double vel_right)  {

  //Mean of velocity in 5(ENCODER_FILTER) interations if encoder good
  cont_Right++;

  Sum_vel_Right = Sum_vel_Right + vel_right;
  double filter = Sum_vel_Right / cont_Right;

  if(cont_Right>ENCODER_FILTER){
    Sum_vel_Right = filter;
    cont_Right = 1;
  }

  if(diffEncoder_Right ==0) filter = 0;

  return filter;
}

// PID correction - Function
int updatePid(int idMotor, double referenceValue, double encoderValue) {
  float Kp = 1.8;  //2.0
  float Kd = 0.1;  //0.1
  float Ki = 0.5;  //0.5
  double pidTerm = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  pid_right = 0;
  pid_left = 0;

  //erro = (kinetmatic - encoder) MetersreferenceSinal_pwm per Second
  double error = (referenceValue - encoderValue);

  if(idMotor == MOTOR_LEFT_1) { //left
    pidTerm = Kp*error + Ki*int_error1 + Kd*(error-last_error1);
    int_error1 += error;
    last_error1 = error;
    pid_left = pidTerm;
  }
  else if(idMotor == MOTOR_RIGHT_2){ //right
    pidTerm = Kp*error + Ki*int_error2 + Kd*(error-last_error2);
    int_error2 += error;
    last_error2 = error;
    pid_right = pidTerm;
  }else{
    pidTerm = 0;
  }

  // Conversion: m/s to PWM
  float referenceSinal_pwm = (pidTerm*255)/0.8;

  // Constrain of Control System
  double constrainMotor = abs(referenceValue)*1.5;

  // Syntax: constrain(sinal, min, max)
  new_pwm = constrain( referenceSinal_pwm, -((constrainMotor*255)/(0.8)), ((constrainMotor*255)/(0.8)) );
  new_cmd = constrain( new_pwm , -255, 255 );


  return int(new_cmd);
}

void call_encoder_Left() {
  if (PWM_val1 > 0) encoder0Pos_Left++;
  else encoder0Pos_Left--;
}
void call_encoder_Right() {
  if (PWM_val2 > 0) encoder0Pos_Right++;
  else encoder0Pos_Right--;
}