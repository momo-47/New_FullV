#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

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
  Serial2.begin(115200);
  while (!Serial2);

  Serial2.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial2.println(F("Testing device connections..."));
    Serial2.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial2.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-1126);
    mpu.setYAccelOffset(-281);
    mpu.setZAccelOffset(999);
    mpu.setXGyroOffset(74);
    mpu.setYGyroOffset(2);
    mpu.setZGyroOffset(23);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial2.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial2.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial2.print(F("DMP Initialization failed (code "));
        Serial2.print(devStatus);
        Serial2.println(F(")"));
    }


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

  if (!dmpReady) return;

    dmpDataReady();

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial2.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        // gyro values
        teapotPacket[10] = fifoBuffer[16];
        teapotPacket[11] = fifoBuffer[17];
        teapotPacket[12] = fifoBuffer[20];
        teapotPacket[13] = fifoBuffer[21];
        teapotPacket[14] = fifoBuffer[24];
        teapotPacket[15] = fifoBuffer[25];
        // accelerometer values
        teapotPacket[16] = fifoBuffer[28];
        teapotPacket[17] = fifoBuffer[29];
        teapotPacket[18] = fifoBuffer[32];
        teapotPacket[19] = fifoBuffer[33];
        teapotPacket[20] = fifoBuffer[36];
        teapotPacket[21] = fifoBuffer[37];
        //temperature
        int16_t temperature = mpu.getTemperature();
        teapotPacket[22] = temperature >> 8;
        teapotPacket[23] = temperature & 0xFF;
        Serial2.write(teapotPacket, 28);
        teapotPacket[25]++; // packetCount, loops at 0xFF on purpose

    }
    
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
