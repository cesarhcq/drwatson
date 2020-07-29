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

#include "robot_specs_polulo.h"
#include <DualVNH5019MotorShield.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>

#define CMD_VEL_TIMEOUT 1000 // milliseconds
#define MAX_VEL 0.61 // meter/second
#define MAX_INT 200.0 // limite da integral
#define Kp 1.8
#define Ki 0.8
#define Kd 0.1
#define PWM_RESOLUTION 400
#define LOOPTIME 100   // Tempo do controlador [PID loop time(ms)]
#define MOTOR_LEFT_1 0          // Motor Left PID Control
#define MOTOR_RIGHT_2 1         // Motor Right PID Control
#define time_to_sec 0.001 // tempo em segundos


DualVNH5019MotorShield md;

double pid_right;
double pid_left;

char encoder[] = "/encoder";

unsigned long time = 0;
unsigned long lastMilli = 0;       // loop timing
double vel_req1;
double vel_req2;
double vel_act1;
double vel_act2;
int PWM_val1;
int PWM_val2;

double last_error1 = 0;
double last_error2 = 0;
double int_error1 = 0;
double int_error2 = 0;

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

//Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_rasp("/cmd_vel", &handle_cmd);

//Publisher
ros::Publisher pub_encoder("/vel_encoder", &vel_encoder_msg);
ros::Publisher pub_pid("/vel_pid", &vel_pid_msg);

// *********************************************

void setup()
{
  Serial.begin(57600); // MEGA_USB = 0 (RX), 1 (TX) -> RASPBERRY (USB)
  
  delay(1000);

  //PINs of Encoder
  pinMode (ENCODER_LEFT_PIN, INPUT);
  pinMode (ENCODER_RIGHT_PIN, INPUT);

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
  nh.advertise(pub_pid);
}


void loop()
{

  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME){
    getMotorData(time-lastMilli);

    // float sinal1 = (vel_req1*PWM_RESOLUTION)/MAX_VEL;
    // float sinal2 = (vel_req2*PWM_RESOLUTION)/MAX_VEL;

    // PWM_val1 = constrain(sinal1, -PWM_RESOLUTION, PWM_RESOLUTION);
    // PWM_val2 = constrain(sinal2, -PWM_RESOLUTION, PWM_RESOLUTION);

    PWM_val1 = updatePid(MOTOR_LEFT_1, vel_req1, vel_act1);
    PWM_val2 = updatePid(MOTOR_RIGHT_2, vel_req2, vel_act2);

    md.setM1Speed(PWM_val1);
    md.setM2Speed(PWM_val2);

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

// Get the motor velocity with Encoder - Function
void getMotorData(unsigned long time)  {
  double dt = time * time_to_sec; // time to seg
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

  //Mean of velocity in 5(ENCODER_FILTER) intlast_error1_erations if encoder good
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

  double pidTerm = 0;
  double new_pwm = 0;
  double new_cmd = 0;

  double dt = time - lastMilli;

  //erro = (kinetmatic - encoder) MetersreferenceSinal_pwm per Second
  double error = (referenceValue - encoderValue);

  if(idMotor == MOTOR_LEFT_1) { //left
    pidTerm = Kp*error; + Ki*int_error1; // + Kd*(last_error1-error);
    int_error1 += error*dt;
    last_error1 = error;
    pid_left = pidTerm;
    int_error1 = constrain(int_error1, -MAX_INT, MAX_INT);
  }
  else if(idMotor == MOTOR_RIGHT_2){ //right

    pidTerm = Kp*error; + Ki*int_error2; //+ Kd*(last_error2 - error);
    int_error2 += error*dt;
    last_error2 = error;
    pid_right = pidTerm;
    int_error2 = constrain(int_error2, -MAX_INT, MAX_INT);
  }else{
    pidTerm = 0;
  }

  // Conversion: m/s to PWM
  double referenceSinal_pwm = (pidTerm*PWM_RESOLUTION)/MAX_VEL;

  // Syntax: constrain(sinal, min, max)
  //new_pwm = constrain( referenceSinal_pwm, -((constrainMotor*255)/(0.8)), ((constrainMotor*255)/(0.8)) );
  new_cmd = constrain(referenceSinal_pwm,-PWM_RESOLUTION, PWM_RESOLUTION);


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