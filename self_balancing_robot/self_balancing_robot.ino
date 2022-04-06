#include "I2C.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050


#define leftMotorPWMPin 6
#define leftMotorDirPin 7
#define leftMotorGNDPin 8


#define rightMotorPWMPin 5
#define rightMotorDirPin 4
#define rightMotorGNDPin 3


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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

 

/*********Tune these Values*********/
double setpoint= 180.5; //178.54 set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 72.0 ; //65.0Set this first
double Kd = 1.6 ; //1.6Set this secound
double Ki = 80; // 160 Finally set this 
/******End*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);

  // initialize device
//    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

     // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(4);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(-8);

    mpu.setXAccelOffset(-5528);
    mpu.setYAccelOffset(-753);
    mpu.setZAccelOffset(651); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(2);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus);
//        Serial.println(F(")"));
    }

//Initialise the Motor output pins
    pinMode (leftMotorPWMPin, OUTPUT);
    pinMode (leftMotorDirPin, OUTPUT);
    pinMode (leftMotorGNDPin, OUTPUT);
    pinMode (rightMotorPWMPin, OUTPUT);
    pinMode (rightMotorDirPin,OUTPUT);
    pinMode(rightMotorGNDPin,OUTPUT);
//By default turn off both the motors
    digitalWrite(leftMotorDirPin,LOW);
    digitalWrite(leftMotorGNDPin,LOW);
    digitalWrite(rightMotorDirPin,LOW);
    digitalWrite(rightMotorGNDPin,LOW);
    analogWrite(leftMotorPWMPin,0);
    analogWrite(rightMotorPWMPin,0);
    
}

 

void loop()
{
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   


        Serial.print("input: "); Serial.print(input);  Serial.print("  ");
        Serial.print("output: "); Serial.print(output);  Serial.print("  ");
//        Serial.print("D: "); Serial.print(pid.GetKd()); Serial.print("  ");
        Serial.println("uT");
        
        //Print the value of Input and Output on serial monitor to check how it is working.
//        Serial.println(input); 
//        Serial.print(" =>"); 
//        Serial.println(output);
               
        if (input>150 && input<200)//If the Bot is falling 
        {
          
          if (output>0) //Falling towards front 
          {
          Forward(); //Rotate the wheels forward 
          }
          else if (output<0) //Falling towards back
          {
          Reverse(); //Rotate the wheels backward
          } 
        }
        else //If Bot not falling
        {
        Stop(); //Hold the wheels still
        }
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[1] * 180/M_PI + 180;

   }
}

void Forward() //Code to rotate the wheel forward 
{
    digitalWrite(leftMotorDirPin,LOW);
    digitalWrite(leftMotorGNDPin,HIGH);
    digitalWrite(rightMotorDirPin,LOW);
    digitalWrite(rightMotorGNDPin,HIGH);
    analogWrite(leftMotorPWMPin,output);
    analogWrite(rightMotorPWMPin,output);
//    Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    digitalWrite(leftMotorDirPin,HIGH);
    digitalWrite(leftMotorGNDPin,LOW);
    digitalWrite(rightMotorDirPin,HIGH);
    digitalWrite(rightMotorGNDPin,LOW); 
    analogWrite(leftMotorPWMPin,output*-1);
    analogWrite(rightMotorPWMPin,output*-1);
//    Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
    digitalWrite(leftMotorDirPin,LOW);
    digitalWrite(leftMotorGNDPin,LOW);
    digitalWrite(rightMotorDirPin,LOW);
    digitalWrite(rightMotorGNDPin,LOW); 
    analogWrite(leftMotorPWMPin,0);
    analogWrite(rightMotorPWMPin,0);
//    Serial.print("S");
}
