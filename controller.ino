// reference:http://akiracing.com/2017/12/18/arduino_drone_motordrive/

#include <Servo.h>
#include <SPI.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>   
#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

//The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

// nano CH and setup
#define m1Pin 3
#define m2Pin 5
#define m3Pin 6
#define m4Pin 9
Servo m1PWM, m2PWM, m3PWM, m4PWM;
const int throttle_max = 1923; //maximum width of the PWM
const int throttle_min = 900; //minimum width of the PWM
int level; //value of the motor output (width of the PWM(microsec))

// reciever CH and setup
#define CH1 2
#define CH2 10
#define CH3 11
#define CH4 12
#define CH5 20
#define CH6 21


int ch1Value, ch2Value, ch3Value, ch4Value;

// madgwick
float B_madgwick = 0.04;  //(default 0.04)
float q0 = 1.0f;          //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Controller parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
float i_limit = 20;   //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 18.0;   //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 18.0;  //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 10.0;    //Max yaw rate in deg/sec (default 160.0)
float maxMotor = 0.8;
float Kp_range = 20;
float Kd_range = 5;



float parameter_rate = 1.0;

// float Kp_roll_angle = 15 * parameter_rate;  //Roll P-gain
// float Ki_roll_angle = 0.00 * parameter_rate;  //Roll I-gain
// float Kd_roll_angle = 0.8 * parameter_rate;  //Roll D-gain

// float Kp_pitch_angle = 15 * parameter_rate;  //Pitch P-gain
// float Ki_pitch_angle = 0.00 * parameter_rate;   //Pitch I-gain
// float Kd_pitch_angle = 0.8 * parameter_rate;  //Pitch D-gain

float Kp_roll_angle = 6.06 * parameter_rate;  //Roll P-gain
float Ki_roll_angle = 0.00 * parameter_rate;  //Roll I-gain
float Kd_roll_angle = 0.83 * parameter_rate;  //Roll D-gain

float Kp_pitch_angle = 6.06 * parameter_rate;  //Pitch P-gain
float Ki_pitch_angle = 0.00 * parameter_rate;   //Pitch I-gain
float Kd_pitch_angle = 0.83 * parameter_rate;  //Pitch D-gain

float Kp_yaw = 0;  //Yaw P-gain default 30
float Ki_yaw = 0;  //Yaw I-gain default 5
float Kd_yaw = 0;  //Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)

float stick_dampener = 0.3;

//General stuff for controlling timing of things
float deltaTime = 1;
float invFreq = (1.0 / LOOP_TIMING) * 1000000.0;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;

unsigned long previousMillis = 0;
unsigned long currentMillis;  // Declare currentMillis as a global variable
float frameRate;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

float AccErrorX = 0.03;
float AccErrorY = 0.02;
float AccErrorZ = 0.01;
float GyroErrorX = 0.9;
float GyroErrorY = -0.40;
float GyroErrorZ = 0.0;

float RollError = -8.0;
float PitchError = 4.0;

float rollPIDError = -0.02;
float pitchPIDError = 0.00;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;

float volA_norm, volB_norm;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;

//Radio communication:
unsigned long PWM_throttle, PWM_roll, PWM_Elevation, PWM_Rudd, PWM_ThrottleCutSwitch;
unsigned long PWM_throttle_prev, PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;
unsigned long PWM_throttle_output, PWM_roll_output, PWM_Elevation_output, PWM_Rudd_output;
unsigned long volA, volB;

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Red the channel and return a boolean value
bool redSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup() {
  Serial.begin(115200);
  // nano
  m1PWM.attach(m1Pin);
  m2PWM.attach(m2Pin);
  m3PWM.attach(m3Pin);
  m4PWM.attach(m4Pin);
  PWM_throttle = throttle_min;
  m1PWM.writeMicroseconds(PWM_throttle);
  m2PWM.writeMicroseconds(PWM_throttle);
  m3PWM.writeMicroseconds(PWM_throttle);
  m4PWM.writeMicroseconds(PWM_throttle);
  // reciever
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);

  delay(4000);

  //Initialize IMU communication
  IMUinit();
  MadgwickFilter.begin(50);

  // calibrateESCs();

  m1_command_PWM = 0;  //Will send the default for motor stopped for Simonk firmware
  m2_command_PWM = 0;
  m3_command_PWM = 0;
  m4_command_PWM = 0;
}

void loop() {
  currentMillis = millis();
  
  if (currentMillis - previousMillis > 0) {
    // Calculate the frame rate in frames per second (FPS)
    frameRate = 1000.0 / (currentMillis - previousMillis);

    // Print the frame rate to the serial monitor
    Serial.println(frameRate);

    // Update previousMillis for the next loop
    previousMillis = currentMillis;
  }

  loopDrone();
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  if (!IMU.begin()) {
    while (true)
      ;
  }
  delay(15);
}

void loopDrone() {
  getIMUdata();                                            //Pulls raw gyro andaccelerometer data from IMU and applies LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);  //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle();                           //Convert raw commands to normalized values based on saturated control limits
  PIDControlCalcs();                                       //The PID functions. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer();                                          //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands();                                         //Scales motor commands to 0-1
  commandMotors();                                         //Sends command pulses to each ESC pin to drive the motors
  getRadioSticks();                                        //Gets the PWM from the radio receiver
  // getRadioVolumes();

  printAcc();
  printGyro();
  printRollPitchYaw();
  printPIDoutput();
  printMotorCommands();
  // printReceive();
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU

  //Accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccY = AccY - AccErrorZ;
  } else return;
  if (IMU.gyroscopeAvailable()) {
    //Gyro
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
  }
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az) {
  MadgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);
  roll_IMU = MadgwickFilter.getRoll() - RollError;
  pitch_IMU = -MadgwickFilter.getPitch() - PitchError;
  yaw_IMU  = MadgwickFilter.getYaw();
}

void getDesiredAnglesAndThrottle() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in degrees
   * yaw_des is scaled to be within max yaw in degrees/sec.
   */
  thro_des = (PWM_throttle - 1000.0) / 1000.0;   //Between 0 and 1
  roll_des = (PWM_roll - 1482.0) / 500.0;        //Between -1 and 1
  pitch_des = (PWM_Elevation - 1487.0) / 500.0;  //Between -1 and 1
  yaw_des = (PWM_Rudd - 1485.0) / 500.0;         //Between -1 and 1

  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0) * 0.6;                //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;     //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch;  //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;        //Between -maxYaw and +maxYaw
}

void PIDControlCalcs() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesiredAnglesAndThrottle(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  // if (PWM_throttle < 1300)
  // { //This will keep the motors from spinning with the throttle at zero should the drone be sitting unlevel.
  //   integral_roll_prev = 0;
  //   integral_pitch_prev = 0;
  //   error_yaw_prev = 0;
  //   integral_yaw_prev = 0;
  //   roll_PID=0;
  //   pitch_PID=0;
  //   yaw_PID=0;
  //   return;
  // }

  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * deltaTime;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Limit integrator to prevent saturating
  derivative_roll = GyroX; //(roll_des-roll_IMU-roll_des-previous_IMU)/dt=current angular velocity since last IMU read and therefore GyroX in deg/s
  roll_PID = 0.0001 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll);  //Scaled by .0001 to bring within -1 to 1 range
  roll_PID -= rollPIDError;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * deltaTime;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  derivative_pitch = GyroY;
  pitch_PID = .0001 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch);  //Scaled by .0001 to bring within -1 to 1 range
  pitch_PID -= pitchPIDError;

  //Yaw, stablize on rate from GyroZ versus angle.  In other words, your stick is setting y axis rotation speed - not the angle to get to.
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * deltaTime;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  derivative_yaw = (error_yaw - error_yaw_prev) / deltaTime;
  yaw_PID = .0001 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .0001 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables  
   */

  //Quad mixing. maxMotor is used to keep the motors from being too violent if you have a big battery and concers about that.
  m1_command_scaled = maxMotor * (thro_des) - pitch_PID + roll_PID + yaw_PID;    //Front left
  m2_command_scaled = maxMotor * (thro_des) - pitch_PID - roll_PID - yaw_PID;    //Front right
  m3_command_scaled = maxMotor * (thro_des) + pitch_PID - roll_PID + yaw_PID;    //Back Right
  m4_command_scaled = maxMotor * (thro_des) + pitch_PID + roll_PID - yaw_PID;    //Back Left

  m1_command_scaled = constrain(m1_command_scaled, 0, 1.0);
  m2_command_scaled = constrain(m2_command_scaled, 0, 1.0);
  m3_command_scaled = constrain(m3_command_scaled, 0, 1.0);
  m4_command_scaled = constrain(m4_command_scaled, 0, 1.0);
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * The actual pulse width is set at the servo attach.
   */
  //Scale to Servo PWM 0-180 degrees for stop to full speed.  No need to constrain since mx_command_scaled already is.
  m1_command_PWM = m1_command_scaled * throttle_max;
  m2_command_PWM = m2_command_scaled * throttle_max;
  m3_command_PWM = m3_command_scaled * throttle_max;
  m4_command_PWM = m4_command_scaled * throttle_max;  
}



void getRadioSticks() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */
  // PWM_throttle = pulseIn(CH1, HIGH, 30000);
  // PWM_roll = pulseIn(CH2, HIGH, 30000);
  // PWM_Elevation = pulseIn(CH3, HIGH, 30000);
  // PWM_Rudd = pulseIn(CH4, HIGH, 30000);
  // PWM_throttle = pulseIn(CH4, HIGH, 30000);
  // PWM_roll = pulseIn(CH1, HIGH, 30000);
  // PWM_Elevation = pulseIn(CH2, HIGH, 30000);
  // PWM_Rudd = pulseIn(CH3, HIGH, 30000);
  PWM_throttle = pulseIn(CH2, HIGH, 30000);
  PWM_roll = pulseIn(CH4, HIGH, 30000) * 1.0;
  PWM_Elevation = pulseIn(CH3, HIGH, 30000) * 1.0;
  PWM_Rudd = pulseIn(CH1, HIGH, 30000);

  PWM_throttle_output = PWM_throttle;
  PWM_roll_output = PWM_roll;
  PWM_Elevation_output = PWM_Elevation;
  PWM_Rudd_output = PWM_Rudd;

  //Low-pass the critical commands and update previous values
  if (PWM_throttle - PWM_throttle_prev < 0)
  {
    //Going down  - slow
    PWM_throttle = (.95) * PWM_throttle_prev + 0.05 * PWM_throttle;
  } else 
  { //Going up - fast
    PWM_throttle = (stick_dampener)*PWM_throttle_prev + (1 - stick_dampener) * PWM_throttle;
  }
  PWM_roll = (1.0 - stick_dampener) * PWM_roll_prev + stick_dampener * PWM_roll;
  PWM_Elevation = (1.0 - stick_dampener) * PWM_Elevation_prev + stick_dampener * PWM_Elevation;
  PWM_Rudd = (1.0 - stick_dampener) * PWM_Rudd_prev + stick_dampener * PWM_Rudd;

  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev = PWM_roll;
  PWM_Elevation_prev = PWM_Elevation;
  PWM_Rudd_prev = PWM_Rudd;
}

void getRadioVolumes(){
  volA = pulseIn(CH5, HIGH, 30000);
  volB = pulseIn(CH6,  HIGH, 30000);

  volA_norm = (volA - 1000.0)/ 1000.0;
  volB_norm = (volB - 1000.0) / 1000.0;

  volA_norm = constrain(volA_norm,0.0,1.0);
  volB_norm = constrain(volB_norm,0.0,1.0);

  Kp_roll_angle = Kp_pitch_angle = Kp_range * volA_norm;
  Kd_roll_angle = Kd_pitch_angle = Kd_range * volB_norm;

  Serial.print("volA:"); Serial.print(volA); Serial.print("\t");
  Serial.print("volB:"); Serial.print(volB);
  Serial.println("");
  Serial.print("Kp:"); Serial.print(Kp_roll_angle); Serial.print("\t");
  Serial.print("Kd:"); Serial.print(Kd_roll_angle);
  Serial.println("");
  

}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins
  m1_command_PWM += 1040;
  m2_command_PWM += 980;
  m3_command_PWM += 980;
  m4_command_PWM += 910;
  m1PWM.write(m3_command_PWM);
  m2PWM.write(m4_command_PWM);
  m3PWM.write(m2_command_PWM);
  m4PWM.write(m1_command_PWM);
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero.
   */
  m1PWM.write(throttle_max);
  m2PWM.write(throttle_max);
  m3PWM.write(throttle_max);
  m4PWM.write(throttle_max);
  delay(20000);
  m1PWM.write(throttle_min);
  m2PWM.write(throttle_min);
  m3PWM.write(throttle_min);
  m4PWM.write(throttle_min);
  delay(20000);
}

void printRollPitchYaw() {
  Serial.print(F(" roll_imu: "));
  Serial.print(roll_IMU);
  Serial.print(F(" pitch_imu: "));
  Serial.print(pitch_IMU);
  Serial.print(F(" yaw_imu: "));
  Serial.println(yaw_IMU);
}

void printAcc() {
  Serial.print(F(" AccX: "));
  Serial.print(AccX);
  Serial.print(F(" AccY: "));
  Serial.print(AccY);
  Serial.print(F(" AccZ: "));
  Serial.println(AccZ);
}

void printGyro() {
  Serial.print(F(" GyroX: "));
  Serial.print(GyroX);
  Serial.print(F(" GyroY: "));
  Serial.print(GyroY);
  Serial.print(F(" GyroZ: "));
  Serial.println(GyroZ);
}

void printMotorCommands() {
  Serial.print(F("m1_command: "));
  Serial.print(m1_command_PWM);
  // Serial.print(m1_command_scaled);
  Serial.print(F(" m2_command: "));
  Serial.print(m2_command_PWM);
  // Serial.print(m2_command_scaled);
  Serial.print(F(" m3_command: "));
  Serial.print(m3_command_PWM);
  // Serial.print(m3_command_scaled);
  Serial.print(F(" m4_command: "));
  Serial.println(m4_command_PWM);
  // Serial.print(m4_command_scaled);
}

void printPIDoutput() {
  Serial.print(F("roll_PID: "));
  Serial.print(roll_PID);
  Serial.print(F(" pitch_PID: "));
  Serial.print(pitch_PID);
  Serial.print(F(" yaw_PID: "));
  Serial.println(yaw_PID);
}

void printReceive() {
  Serial.print(F(", \"PWM_throttle\": "));
  Serial.print(PWM_throttle_output);
  Serial.print(F(", \"PWM_roll\": "));
  Serial.print(PWM_roll_output);
  Serial.print(F(", \"PWM_Elevation\": "));
  Serial.print(PWM_Elevation_output);
  Serial.print(F(", \"PWM_Rudd\": "));
  Serial.print(PWM_Rudd_output);
}

float invSqrt(float x) {
  return 1.0 / sqrtf(x);  //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}