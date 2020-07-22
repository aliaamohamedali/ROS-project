/* MPU9250_MS5611_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.

  Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
  getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
  allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
  Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

  This sketch is intended specifically for the MPU9250MS5611 Add-on shield for the Teensy 3.1.
  It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
  The MS5611 is a simple but high resolution pressure sensor, which can be used in its high resolution
  mode but with power consumption od 20 microAmp, or in a lower resolution mode with power consumption of
  only 1 microAmp. The choice will depend on the application.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND

  Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
  Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
  We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
  We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
*/
#include <SBWire.h>
#include "MyMPU9250.h"

// Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int ledPin  = 13;
int motorPin1 = 22;
int motorPin2 =  3;
int motorPin3 = 23;
int motorPin4 =  4;
int led1 = 18;
int led2 = 21;
int led3 =  5;
int led4 =  6;

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5611 pressures sensor pressure and temperature
float SelfTest[6];            // holds results of gyro and accelerometer self test

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
uint32_t ledTimer = 0;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
//float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void setup(){
  Wire.begin();
  TWBR = 24;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  //  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(2000);
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  pinMode(motorPin1, OUTPUT);
  analogWrite(motorPin1, 10);
  pinMode(motorPin2, OUTPUT);
  analogWrite(motorPin2, 0);
  pinMode(motorPin3, OUTPUT);
  analogWrite(motorPin3, 0);
  pinMode(motorPin4, OUTPUT);
  analogWrite(motorPin4, 0);
  pinMode(led1, OUTPUT);
  digitalWrite(led1, LOW);
  pinMode(led2, OUTPUT);
  digitalWrite(led2, LOW);
  pinMode(led3, OUTPUT);
  digitalWrite(led3, LOW);
  pinMode(led4, OUTPUT);
  digitalWrite(led4, LOW);


  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x73) { // WHO_AM_I should always be 0x68
    Serial.println("MPU9250 is online...");

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
    delay(1000);

    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    initMPU9250();
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

    if (SerialDebug) {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }
    
    // get sensor resolutions, only need to do this once
    getAres();
    getGres();
    getMres();
    magbias[0] = 0;//+470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = 0;//+120.;  // User environmental y-axis correction in milliGauss
    magbias[2] = 0;//+125.;  // User environmental z-axis correction in milliGauss

  }
  else{
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }
}

void loop(){
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    // if (digitalRead(intPin)) {  // On interrupt, read data
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes - accelBias[1];
    az = (float)accelCount[2] * aRes - accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
  //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

  delt_t = millis() - count;
  if (delt_t > 10) { 
    tempCount = readTempData();  // Read the gyro adc values
    temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade
    //Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    yaw   -= 2.02; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04

    float g[3] = {0, 0, 0};
    
    g[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    g[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    if (SerialDebug) {
      Serial.print("Acc: ");  
      Serial.print((int)1000 * (ax - g[0])); Serial.print("\t");
      Serial.print((int)1000 * (ay - g[1])); Serial.print("\t");
      Serial.print((int)1000 * (az - g[2])); Serial.print("\t"); Serial.print(" mg"); 
/*
      Serial.print(" gx = "); Serial.print( gx, 2);
      Serial.print(" gy = "); Serial.print( gy, 2);
      Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      Serial.print(" mx = "); Serial.print( (int)mx );
      Serial.print(" my = "); Serial.print( (int)my );
      Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
*/
      Serial.print(" | Quat: "); 
      Serial.print(q[0]); Serial.print("\t");
      Serial.print(q[1]); Serial.print("\t");
      Serial.print(q[2]); Serial.print("\t");
      Serial.print(q[3]); Serial.print("\t");
      
      Serial.print(" | Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2); Serial.print("\t");
      Serial.print(pitch, 2); Serial.print("\t");
      Serial.println(roll, 2);

      //Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
    }
    count = millis();
    sumCount = 0;
    sum = 0;
  }

  if (millis() - ledTimer > 200) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    ledTimer = millis();
  }

}
