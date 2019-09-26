
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  
  }

void loop() {
  motor(150,150,3000);
  motor(-150,-150,1000);
  motor(-150,150,2000);

}

void motor(int rightPower,int leftPower,int delayTime){
  if(rightPower>0 && leftPower>0){
    motorRight  -> run(FORWARD);
    motorLeft   -> run(FORWARD);
    motorRight  -> setSpeed(rightPower);
    motorLeft   -> setSpeed(leftPower);      
  }   
  else if(rightPower>0 && leftPower<0){
    motorRight  -> run(FORWARD);
    motorLeft   -> run(BACKWARD);
    motorRight  -> setSpeed(rightPower);
    motorLeft   -> setSpeed(-leftPower);      
  }
  else if(rightPower<0 && leftPower>0){
    motorRight  -> run(BACKWARD);
    motorLeft   -> run(FORWARD);
    motorRight  -> setSpeed(-rightPower);
    motorLeft   -> setSpeed(leftPower);      
  }
  else{
    motorRight  -> run(BACKWARD);
    motorLeft   -> run(BACKWARD);
    motorRight  -> setSpeed(-rightPower);
    motorLeft   -> setSpeed(-leftPower);      
  }  
  
  delay(delayTime);

  motorRight ->run(RELEASE);
  motorLeft  ->run(RELEASE);
  
}
