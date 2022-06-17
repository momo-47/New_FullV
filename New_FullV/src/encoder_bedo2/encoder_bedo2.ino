#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
int pulse1 = 0;
int pulse2 = 0;
float w1 = 0;
float w2 = 0;
float dist1 = 0;
float dist2 = 0;
float dist = 0;
float last = 0;
int pulse1_dist = 0;
int pulse2_dist = 0;
float v = 0;
float w = 0;
double x = 0;
double y = 0;
double z = 0;
double theeta = 0;
float i;
int j;
float quaternion[36];
float p[3];
float t;

double vx, vy, vz, wx, wy, wz;

ros::NodeHandle nh;

nav_msgs::Odometry odom_msg;
geometry_msgs::Quaternion trans;
geometry_msgs::TransformStamped tr;
tf::TransformBroadcaster broadcaster;

ros::Publisher pub_odom("/noisy_odom", &odom_msg);



void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub_odom);
  broadcaster.init(nh);

  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  pinMode(21, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(21), A_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), B_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), C_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), D_CHANGE, CHANGE);
  Serial.begin(115200);



}

void odoms() {
  cli();
  t = (micros() - last) / 1000000;
  w1 = ((pulse1 / (360.0*2.0*60.0)) * 2.0 * 3.14159) / t;
  w2 = ((pulse2 / (360.0*2.0*60.0)) * 2.0 * 3.14159) / t;
  dist = (0.056 / 2.0) * ((float)dist1 + (float)dist2);


  vx = (0.056 / 2.0) * ((float)w1 + (float)w2);
  vy = 0;
  vz = 0;

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = vz;

  wx = 0;
  wy = 0;
  wz = (0.056 / 0.4) * ((float)w1 - (float)w2);

  odom_msg.twist.twist.angular.x = wx;
  odom_msg.twist.twist.angular.y = wy;
  odom_msg.twist.twist.angular.z = wz;

  //odom_msg.twist.covariance = odom_msg.twist.twist;

  odom_msg.twist.covariance[0] = vx;
  odom_msg.twist.covariance[1] = vy;
  odom_msg.twist.covariance[2] = vz;
  odom_msg.twist.covariance[3] = wx;
  odom_msg.twist.covariance[4] = wy;
  odom_msg.twist.covariance[5] = wz;


  theeta = (0.056 / 0.4) * ((float)dist1 - (float)dist2);
  //i=(0.4/2.0)*((w1+w2)/(w1-w2));
  y = (t * vx * sin(theeta)) + y;
  x = (t * vx * cos(theeta)) + x;
  z = 0;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = z;


  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theeta);
//char base_link[] = "/base_link";
//char odom[] = "/odom";

  /*odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = sin(theeta*0.5);
    odom_msg.pose.pose.orientation.w = cos(theeta*0.5);*/


  odom_msg.pose.covariance[0] = x;
  odom_msg.pose.covariance[1] = y;
  odom_msg.pose.covariance[2] = z;
  odom_msg.pose.covariance[3] = 0;
  odom_msg.pose.covariance[4] = 0;
  odom_msg.pose.covariance[5] = theeta;

  /*tr.header.frame_id = "/odom";
  tr.child_frame_id = "/base_link";
  
  tr.transform.translation.x = x;
  tr.transform.translation.y = y;
  
  tr.transform.rotation = tf::createQuaternionFromYaw(theeta);
  tr.header.stamp = nh.now();*/

  pulse1 = 0;
  pulse2 = 0;
  /*
    Serial.print(vx);
    Serial.println("   m/s");
    Serial.print(wz);
    Serial.println("   rad/s");
    Serial.print(x);
    Serial.println("   m");
    Serial.print(y);
    Serial.println("   m");
    Serial.print(theeta);
    Serial.println("   rad");
  */
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((int)(micros() - last) >= 50) {
    odoms();
    odom_msg.header.stamp = nh.now();
    tr.header.frame_id = "/odom";
      tr.child_frame_id = "/base_link";
      tr.transform.translation.x = x;
      tr.transform.translation.y = y;
      tr.transform.translation.z = 0;
      tr.transform.rotation.x = odom_msg.pose.pose.orientation.x;
      tr.transform.rotation.y = odom_msg.pose.pose.orientation.y;
      tr.transform.rotation.z = odom_msg.pose.pose.orientation.z;
      tr.transform.rotation.w = odom_msg.pose.pose.orientation.w;
      tr.header.stamp = nh.now();
      broadcaster.sendTransform(tr);
    pub_odom.publish(&odom_msg);
    nh.spinOnce();
    //delay(500);
    last = micros();
    delay(10);
  }

}



void A_CHANGE() {
  if (((digitalRead(21) == 1) && (digitalRead(19) == 0)) || ((digitalRead(21) == 0) && (digitalRead(19) == 1))) {
    pulse1++;
    pulse1_dist++;
  } else {
    pulse1--;
    pulse1_dist--;

  }
  dist1 = ((pulse1_dist / (360.0*2.0*60)) * 2.0 * 3.14159);
  //Serial.println(dist1);
}
void B_CHANGE() {
}

void C_CHANGE() {
  if (((digitalRead(22) == 1) && (digitalRead(23) == 0)) || ((digitalRead(22) == 0) && (digitalRead(23) == 1))) {
    pulse2++;
    pulse2_dist++;
  } else {
    pulse2--;
    pulse2_dist--;

  }
  dist2 = ((pulse2_dist / (360.0*2.0*60)) * 2.0 * 3.14159);
  //Serial.println(dist2);
}
void D_CHANGE() {
}
