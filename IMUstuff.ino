///VCC  -  5V
///GND  -  GND
///SDA  -  A4
///SCL  -  A5

#include <Wire.h>     //Include Wire library for I2C communication
#include <Servo.h>    //Include Servo library to drive the servos




//Define servos
//1 and 2 control roll, at opposite sides
//3 and 4 control pitch, at opposite sides
Servo hipServo1;
Servo hipServo2;
Servo hipServo3;
Servo hipServo4;


int hipServoPin1=9;                   //Servo Pin attachments
int hipServoPin2=11;
int hipServoPin3 = 3;
int hipServoPin4 = 5;
double temperature;                   //MPU6050 measures temperature
double acc_x;                         //Accelerometer value x
double acc_y;                         //Accelerometer value y
double acc_z;                         //Accelerometer value z
double timer = 0;                     //Loop timer for integration
float gyro_offset_x;                  //Gyroscope calibration offset x
float gyro_offset_y;                  //Gyroscope calibration offset y
float gyro_offset_z;                  //Gyroscope calibration offset z
double acc_counterx;                  //Accelerometer median filter stuff
double acc_countery;                  //
double acc_counterz;                    //
double acc_angle_x;                   //Angle prediction by the accelerometer
double acc_angle_y;
double acc_angle_z;
float gyro_velocity_x;                //Angular velocities
float gyro_velocity_y;
float gyro_velocity_z;
double gyro_angle_x;                  //Gyroscope angle prediction
double gyro_angle_y;
double gyro_angle_z;
float delta_t;                        //Delta t integrator
double acc_total_vector;              
double angle_x;                       
double angle_y;
double angle_z;
double acc_pitch;                     //Accelerometer's pitch and  roll prediction
double acc_roll;
double pitch;
double roll;
double kp = 0.005;
double kI = 0.005;
float pitch_integral=0;
double hipServo1_signal = 0;
float PI_timer=0;
double previous_position1 = 120;
double previous_position2 = 120;
double previous_position3 = 120;
double previous_position4 = 120;




void set_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


void get_IMU_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_velocity_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_velocity_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_velocity_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

//Find the Gyro Offset
void gyro_calibration(){

  //Read 2000 Values While Holding the IMU Stationary
  for(int i=0; i<2000; i++){
 
    get_IMU_data();
    gyro_offset_x += gyro_velocity_x;
    gyro_offset_y += gyro_velocity_y;
    gyro_offset_z += gyro_velocity_z;    
       
    }
  //Average These 2000 Measurements to Get the Average Offset
  gyro_offset_x/=2000;
  gyro_offset_y/=2000;
  gyro_offset_z/=2000;
  
  } 
void get_angle(){
  
/////ACCELEROMETER MEDIAN FILTER//////////
      for(int i = 0; i<20; i++){
      get_IMU_data();
      acc_counterx+= acc_x;
      acc_countery+=acc_y;
      acc_counterz+=acc_z;       
      }

   //Average these Measurements
      acc_x = acc_counterx/20;
      acc_y = acc_countery/20;
      acc_z = acc_counterz/20;
      acc_counterx = 0;
      acc_countery=0;
      acc_counterz=0;

//////ACCELEROMETER ANGLE PREDICTION//////
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    acc_pitch=  asin(acc_y/acc_total_vector)* -57.296;
    acc_roll = asin(acc_x/acc_total_vector)* 57.296; 
    
    /*acc_angle_z+=90;
    acc_angle_y+=90;
    acc_angle_z+=90; */
  

 ///////////GYROSCOPE ANGLE PREDICTION VIA INTEGRATION////////
    get_IMU_data();
    delta_t = (micros()-timer)/1000000;                      //delta t for integration
    gyro_velocity_x -= gyro_offset_x;
    gyro_velocity_y -= gyro_offset_y;
    gyro_velocity_z -= gyro_offset_z;
    gyro_velocity_x*=0.0427/2.6;
    gyro_velocity_y*=0.0427/2.6;
    gyro_velocity_z*=0.0427/2.6;     
    gyro_angle_x+=gyro_velocity_x*delta_t;
    gyro_angle_y += gyro_velocity_y *delta_t;
    gyro_angle_z += gyro_velocity_z *delta_t;
    timer = micros();     

////////COMPLEMENTARY FILTER/////////////
  pitch = 0.9*acc_pitch+0.1*gyro_angle_x;
  roll = 0.9*acc_roll+0.1*gyro_angle_y;
 

  /*Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("  --  ");
  Serial.print("Roll: ")*/;
   //Serial.println(pitch);
  
  }

void setup() {
  //Initiate the IMU, calibrate the gyro. DO NOT MOVE THE SYSTEM WHILE CALIBRATING
  Wire.begin();
  Serial.begin(9600);
  set_registers();
  gyro_calibration();
  hipServo1.attach(hipServoPin1);
  hipServo2.attach(hipServoPin2);
  hipServo3.attach(hipServoPin3);
  hipServo4.attach(hipServoPin4);
  hipServo1.write(120); 
  hipServo2.write(120);
  hipServo3.write(120);
  hipServo4.write(120);

  delay(3000);
  
  
}

void loop() {
  
  get_angle();

///////Mixing Algorithm///////
/* Pitch ~~ pos(1) - pos(2)
   Roll ~~  pos(3)- pos(4)  
   Height ~~ pos(1) + pos(2) + pos(3) + pos(4) */
   
   //hipServo1 = kp(Height_error + Pitch Error) + kI(Height_error_integral + Pitch_error_integral)
   //hipServo2 = -kp(Height_error + Pitch Error) - kI(Height_error_integral + Pitch_error_integral)
   //hipServo3 = kp(Height_error + Roll Error ) + kI(Height_error_integral + Roll_error_integral)
   //hipServo4 = -kp(Height_error + Roll Error) - kI(Height_error_integral + Roll_error_integral) 
    pitch_integral += pitch * (micros()-PI_timer)/1000000;
    PI_timer = micros();
    hipServo1_signal += kp*pitch; //+ kI*pitch_integral;
  if(abs(pitch) > 5){
      
     // if(previous_position1 >= 90 && previous_position2 >=90 && previous_position3 >=90 && previous_position4 >=90){
    hipServo1.write(previous_position1+hipServo1_signal);
    previous_position1 += hipServo1_signal;
    hipServo2.write(previous_position2-hipServo1_signal);
    previous_position2 -= hipServo1_signal;
   hipServo3.write(previous_position3+hipServo1_signal);
    previous_position3 += hipServo1_signal;
   hipServo4.write(previous_position4+hipServo1_signal);
    previous_position4 += hipServo1_signal;    

      
      //}
  //hipServo2.write(90-(hipServo1_signal));
  
  }
  else{
    
    hipServo1.write(previous_position1);
    hipServo2.write(previous_position2);
    hipServo3.write(previous_position3);
    hipServo4.write(previous_position4);
    
    
    }
    //Serial.print(90+hipServo1_signal); 
    //Serial.print(","); 
  //  Serial.print("Pitch Angle ------------");
    Serial.println(pitch);
  //Serial.print("Roll Angle -------------");
 //Serial.println(roll);
    delay(30);
}
