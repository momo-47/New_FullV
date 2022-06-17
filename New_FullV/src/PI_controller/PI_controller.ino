#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle  nh;

#define PWM1 9
#define PWM2 3
#define M1CW 11
#define M1CCW 12
#define M2CW 14
#define M2CCW 15


const float  al=0.4;
const float wheelr=0.056;

float wr=0;
float wl=0;
float v=0;
float w=0;

float wr_real=0;
float wl_real=0;
float v_real=0;
float w_real=0;

const float sat1 = 255;
const float sat2 = -255;
int f1l=0;
int f2l=0;
int f1r=0;
int f2r=0;

float el=0;
float er=0;
float esuml=0;
float esumr=0;

long int prevT=0;

float ul=0;
float ur=0;

const float kp=20.0;
const float ki=0;

float pwml=0;
float pwmr=0;

long int CT = 0;


void PWM(const float& wl,const float& wr,const float& wl_real,const float& wr_real){
  el = wl - wl_real;
  er = wr - wr_real;

  CT = micros();

  if( f1l==1 && f2l==1){
   esuml=0;
   ul = (kp * el);
  }else{
   esuml = esuml + (el*((CT-prevT)/1.0e6));
   ul = (kp * el) + (ki * esuml); 
  }

  
  if( f1r==1 && f2r==1){
   esumr=0;
   ur = (kp * er);
  }else{
   esumr = esumr + (er*((CT-prevT)/1.0e6));
   ur = (kp * er) + (ki * esumr); 
  }
  
  if(ul>sat1){
    f1l = 1;
    ul = sat1;
  }else if(ul<sat2){
    f1l = 1;
    ul = sat2;
  }else{
    f1l = 0;
  }

  if((ul<0 && esuml<0) || (ul>0 && esuml>0)){
    f2l = 1;
  }else{
    f2l=0;
  }

    //er = wr - wr_real;

  /*if( f1r==1 && f2r==1){
   esumr=0;
   ur = (kp * er);
  }else{
   esumr = esumr + (er*((micros()-prevT)/1.0e6));
   ul = (kp * er) + (ki * esumr); 
  }*/
  
  if(ur>sat1){
    f1r = 1;
    ur = sat1;
  }else if(ul<sat2){
    f1r = 1;
    ur = sat2;
  }else{
    f1r = 0;
  }

  if((ur<0 && esumr<0) || (ur>0 && esumr>0)){
    f2r = 1;
  }else{
    f2r=0;
  }

  prevT = CT;


  pwml=fabs(ul);
  pwmr=fabs(ur);
  
  if(ul>0){
    digitalWrite(M1CW, HIGH);
    digitalWrite(M1CCW, LOW);
    analogWrite(PWM1, pwml);
  }else{  
    digitalWrite(M1CW, LOW);
    digitalWrite(M1CCW, HIGH);
    analogWrite(PWM1, pwml);   
  }

    if(ur>0){
    digitalWrite(M2CW, HIGH);
    digitalWrite(M2CCW, LOW);
    analogWrite(PWM2, pwmr);
  }else{
    digitalWrite(M2CW, LOW);
    digitalWrite(M2CCW, HIGH);
    analogWrite(PWM2, pwmr);   
  }

}

void odomCb( const nav_msgs::Odometry& odom_msg){
  v_real = odom_msg.twist.twist.linear.x;
  w_real = odom_msg.twist.twist.angular.z;
  wl_real = (v_real-(w_real*(al/2)))/wheelr;
  wr_real = (v_real+(w_real*(al/2)))/wheelr;  
}

void motorCb( const geometry_msgs::Twist& cmd_msg){
  v = cmd_msg.linear.x;
  w = cmd_msg.angular.z;
  wl = (v-(w*(al/2)))/wheelr;
  wr = (v+(w*(al/2)))/wheelr;
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motorCb );

ros::Subscriber<nav_msgs::Odometry> hey("/noisy_odom", &odomCb );

void setup()
{ 
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(M1CW, OUTPUT);
  pinMode(M1CCW, OUTPUT);
  pinMode(M2CW, OUTPUT);
  pinMode(M2CCW, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(hey);
  Serial.begin(9600);
}

void loop()
{  
  PWM(wl,wr,wl_real,wr_real);
  nh.spinOnce();
  delay(1);
}
