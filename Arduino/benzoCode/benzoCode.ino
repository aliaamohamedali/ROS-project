
// ROBOT PHYSCIAL CHARACTERISTICS
#define WHEEL_RADIUS                     0.065           // meter
#define WHEEL_SEPARATION                 0.340           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.177           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

// ROBOT LIMITS
#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 133 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s
#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY

// PINS
#define RIGHT_WHEEL_FORWARD               9
#define RIGHT_WHEEL_BACKWARD              10
#define LEFT_WHEEL_BACKWARD               11
#define LEFT_WHEEL_FORWARD                12
#define LED_PIN                           13

// INDEXING 
#define LINEAR                            0
#define ANGULAR                           1

#define LEFT                              0
#define RIGHT                             1

#define LED                               0
#define CMD_VEL                           1
#define IMU                               2
#define LOOP                              3

// MODFIABLE 
#define BAUD_RATE                         57600
#define LED_FLASH_RATE                    10          // 10 switches/sec (5 flashes/sec)
#define IMU_PUBLISH_RATE                  10           // 5 msg/sec                  
#define PID_SAMPLE_TIME                   10          // msec
#define STABILIZATION_SECONDS             5          // make it 15 when in use !


// TIMERS READABLE SETTINGS
#define TIMER0_MODE           TCCR0A      // PIN  4 & 13
#define TIMER1_MODE           TCCR1A      // PIN 12 & 11
#define TIMER2_MODE           TCCR2A      // PIN 10 &  9
#define TIMER3_MODE           TCCR3A      // PIN  5 &  3 &  2
#define TIMER4_MODE           TCCR4A      // PIN  8 &  7 &  6

// TIMERS PRESCALERS
#define TIMER0_PRESCALER      TCCR0B
#define TIMER1_PRESCALER      TCCR1B
#define TIMER2_PRESCALER      TCCR2B
#define TIMER3_PRESCALER      TCCR3B
#define TIMER4_PRESCALER      TCCR4B

#define FAST_PWM              _BV(COM0A1)|_BV(COM0B1)|_BV(WGM01)|_BV(WGM00)
#define PHASE_CORRECT_PWM     _BV(COM0A1)|_BV(COM0B1)|_BV(WGM00)



/**********************
 *       MPU
 **********************/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

// mpu object
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 acc;        // [x, y, z]            accel sensor measurements
VectorInt16 gyro;       // [x, y, z]            gyro sensor measurments
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aaWorld;    // [w, y, z]
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorFloat velocity;
VectorFloat jerk;
VectorFloat delta;
VectorFloat postn;
double dt;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


/**********************
 *       ROS
 **********************/
ros::NodeHandle nh;

// subscribers
void commandVelocityCallback(const geometry_msgs::Twist&);

geometry_msgs::Twist twistMsg;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

// publishers
sensor_msgs::Imu imuMsg;
ros::Publisher imu_pub("imu", &imuMsg); 

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

geometry_msgs::TransformStamped odom_trans;
geometry_msgs::Quaternion odom_quat;
tf::TransformBroadcaster odom_broadcaster;


ros::Time current_time, last_time;

/**********************
 *      CONTROL
 **********************/
int64_t timer[4] = {0};
double goalVelocity[2] = {0};
double wheelVelocity[2] = {0};

void setup() {

  // set PWM frequency to 62.5 KHz
  TIMER1_MODE = FAST_PWM;
  TIMER2_MODE = FAST_PWM;
  TIMER1_PRESCALER = 0x01;
  TIMER2_PRESCALER = 0x01;
  
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  Serial.begin(BAUD_RATE);
    
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setXAccelOffset(-1647);
  mpu.setYAccelOffset(522);
  mpu.setZAccelOffset(1711); // 1688 factory default for my test chip
  

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection 
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  delay(100);
  initNode();
  last_time = nh.now();

  // the led will flash 15 times (1 flash/sec)
  //Serial.print(F("Stabilizing..."));
  for (int i = 0; i < STABILIZATION_SECONDS; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    //Serial.print(F("."));
    
    digitalWrite(LED_PIN, LOW);
    delay(500);
    //Serial.print(F("."));
  }
  //Serial.println();
  //Serial.flush();
  delay(10);

}


void loop() {
  
 
  if (!dmpReady) return;

  /******************** READ NEW PACKETS ********************/
  if (mpuInterrupt || fifoCount >= packetSize) {

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();

    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&acc, fifoBuffer);
      mpu.dmpGetGyro(&gyro, fifoBuffer);
      //mpu.dmpGetGravity(&gravity, &q);
      //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //mpu.dmpGetAccel(&aa, fifoBuffer);
      //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);


      /******************** UPDATE MESSAGES DATA ********************/

      // calculate time and dt
      current_time = nh.now();
      dt = (millis() - timer[IMU])/1000.0;
      timer[IMU] = millis();

      // prepare reusable quaternion object
      odom_quat.x = q.x;
      odom_quat.y = q.y;
      odom_quat.z = q.z;
      odom_quat.w = q.w;

      
      // setup sensor_msgs::Imu
      imuMsg.header.stamp = current_time;
      imuMsg.header.frame_id = "imu_link";
      
      imuMsg.orientation = odom_quat;
    
      imuMsg.linear_acceleration.x = acc.x / 8192.0 * 9.81;    // m/s2
      imuMsg.linear_acceleration.y = acc.y / 8192.0 * 9.81;    // m/s2
      imuMsg.linear_acceleration.z = acc.z / 8192.0 * 9.81;    // m/s2
      
      imuMsg.angular_velocity.x = gyro.x * 250 / 32768;        // rad/sec
      imuMsg.angular_velocity.y = gyro.x * 250 / 32768;        // rad/sec
      imuMsg.angular_velocity.z = gyro.x * 250 / 32768;        // rad/sec

      // setup transforms
      //jerk.x = (imuMsg.linear_acceleration.x - jerk.x) / dt; 
      //jerk.y = (imuMsg.linear_acceleration.y - jerk.x) / dt; 
    
      velocity.x += imuMsg.linear_acceleration.x * dt + 0.5 * jerk.x * dt * dt; 
      velocity.y += imuMsg.linear_acceleration.y * dt + 0.5 * jerk.y * dt * dt; 
      velocity.z =  imuMsg.angular_velocity.z; // vth
    
      delta.x = (velocity.x * cos(postn.z) - velocity.y * sin(postn.z)) * dt;
      delta.y = (velocity.x * sin(postn.z) + velocity.y * cos(postn.z)) * dt;
      delta.z = velocity.z * dt;
    
      postn.x += delta.x;
      postn.y += delta.y;
      postn.z += delta.z;
      
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
    
      odom_trans.transform.translation.x = postn.x;
      odom_trans.transform.translation.y = postn.y;
      odom_trans.transform.translation.z = dt;
      odom_trans.transform.rotation = odom_quat;
     
      // nav_msgs/Odometry
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

    
      odom.pose.pose.position.x = postn.x;
      odom.pose.pose.position.y = postn.y;
      odom.pose.pose.position.z = postn.z;
    
      odom.pose.pose.orientation = odom_quat;
    
      odom.twist.twist.linear.x = velocity.x;
      odom.twist.twist.linear.y = velocity.y;
      odom.twist.twist.angular.z = velocity.z;
    }
  }
  
  if (millis() - timer[LED] >= 1000 / LED_FLASH_RATE){
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      timer[LED] = millis();
  }

  if (millis() - timer[CMD_VEL] >= 1000){
    twistMsg.linear.x = twistMsg.angular.z = 0; 
  }

  if (millis() - timer[LOOP] >= 0) {
        
    // publish the messages
    odom_broadcaster.sendTransform(odom_trans);
    odom_pub.publish(&odom);
    imu_pub.publish(&imuMsg);
  
    // receive incoming messages and sync with host
    nh.spinOnce();

    timer[LOOP] = millis();
  }

  updateMotors();
    
}

void initNode(){
  
  nh.initNode();
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.setSpinTimeout(10);
  nh.subscribe(cmd_vel_sub);
  
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  odom_broadcaster.init(nh);

  nh.spinOnce();
  delay(100);
}

void updateMotors() {

  
  goalVelocity[LINEAR] = twistMsg.linear.x; // + goalVelocityPID[LINEAR];
  goalVelocity[ANGULAR] = twistMsg.angular.z; // + goalVelocityPID[ANGULAR];

  wheelVelocity[LEFT]   = goalVelocity[LINEAR] - (goalVelocity[ANGULAR] * WHEEL_SEPARATION / 2);
  wheelVelocity[RIGHT]  = goalVelocity[LINEAR] + (goalVelocity[ANGULAR] * WHEEL_SEPARATION / 2);

  int PWM1 = wheelVelocity[RIGHT] * 255 / MAX_LINEAR_VELOCITY;
  analogWrite(RIGHT_WHEEL_FORWARD , abs(max(PWM1, 0)));
  analogWrite(RIGHT_WHEEL_BACKWARD, abs(min(PWM1, 0)));

  int PWM2 = wheelVelocity[LEFT] * 255 / MAX_LINEAR_VELOCITY;
  
  analogWrite(LEFT_WHEEL_FORWARD , abs(max(PWM2, 0)));
  analogWrite(LEFT_WHEEL_BACKWARD, abs(min(PWM2, 0)));

}

void commandVelocityCallback(const geometry_msgs::Twist& msg) {

  twistMsg.linear.x = constrain(msg.linear.x, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  twistMsg.linear.z = constrain(msg.angular.z, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  timer[CMD_VEL] = millis();

}

void printat(){
  
        /*Serial.print("w: ");
      Serial.print(q.w);
      Serial.print(" \t");
      
      Serial.print("x: ");
      Serial.print(q.x);
      Serial.print(" \t");
      
      Serial.print("y: ");
      Serial.print(q.y);
      Serial.print(" \t");
            
      Serial.print("z: ");
      Serial.print(q.z);
      Serial.print(" \t");
      
            
      Serial.print("ax: ");
      Serial.print(acc.x / 8192.0 * 9.81);
      Serial.print(" \t");
      
      Serial.print("ay: ");
      Serial.print(acc.y / 8192.0 * 9.81);
      Serial.print(" \t");

      Serial.print("az: ");
      Serial.print(acc.z / 8192.0 * 9.81);
      Serial.print(" \t");

      Serial.print("gx: ");
      Serial.print(gyro.x / 131.1);
      Serial.print(" \t");

      Serial.print("gy: ");
      Serial.print(gyro.y / 131.1);
      Serial.print(" \t");

      Serial.print("gz: ");
      Serial.print(gyro.z / 131.1);
      Serial.print(" \t");
      
      Serial.println();*/

}
