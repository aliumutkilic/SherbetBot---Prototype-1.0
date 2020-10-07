#include <Servo.h>
int hipServoPin1=9;                   //Servo Pin attachments
int hipServoPin2=11;
int hipServoPin3 = 3;
int hipServoPin4 = 5;
Servo hipServo1;
Servo hipServo2;
Servo hipServo3;
Servo hipServo4;

void setup() {
  // put your setup code here, to run once:
  hipServo1.attach(hipServoPin1);
  hipServo2.attach(hipServoPin2);
  hipServo3.attach(hipServoPin3);
  hipServo4.attach(hipServoPin4);

}

void loop() {
  hipServo1.write(180);
  hipServo2.write(180);
  hipServo3.write(180);
  hipServo4.write(180);
  // put your main code here, to run repeatedly:

}
