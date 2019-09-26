#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MotorShield.h>
#include <TinyGPS++.h>
#include<SoftwareSerial.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2);

TinyGPSPlus tinyGPS;
SoftwareSerial gps(6,7);

double robot_rotation, goal_direction, direction_error, distance_error;
double robot_position_lat, robot_position_lng;
double position_lat_error, position_lng_error;
double mP, motorR_power, motorL_power;
int int_motorR_power, int_motorL_power;

double Kp = 1.4;
double Kp_forward = 200;
double goal_position_lat = 35.004695;
double goal_position_lng = 135.886764;




void setup( ) {
  Serial.begin(9600);
  AFMS.begin();
  gps.begin(9600);
  delay(2000);

  if (!bno.begin()) {
    Serial.print("ERROR, no BNO055 detected...");
    while (1);
  }
  smartDelay(1000);
  robot_position_lat = tinyGPS.location.lat();
  robot_position_lng = tinyGPS.location.lng();

  bno.setExtCrystalUse(true);

}
void loop( ) {

  sensors_event_t event;
  bno.getEvent(&event);

  robot_rotation = event.orientation.x;
  //  Serial.print(robot_rotation);
  //  Serial.print(",");

  robot_position_lat = tinyGPS.location.lat();
  robot_position_lng = tinyGPS.location.lng();

  distance_error = convertToGoalDistance(robot_position_lat, robot_position_lng, goal_position_lat, goal_position_lng);

  Serial.print(tinyGPS.location.lat(), 6);
  Serial.print(",");
  Serial.println(tinyGPS.location.lng(), 6);

  if (distance_error < 5) {
    motor(0, 0);
    Serial.print("Goal!!");
    delay(3000);
  }


  position_lat_error = goal_position_lat - robot_position_lat;
  position_lng_error = goal_position_lng - robot_position_lng;
  goal_direction = atan2(position_lng_error, position_lat_error) * 180 / M_PI;

  direction_error = convertToGoalCoordinates(goal_direction, robot_rotation);

  Serial.print(distance_error);
  Serial.print(",");
  Serial.println(direction_error);

  mP = (1 - abs(direction_error) / 180);
  //  mP = 1/ (1 + exp((abs(direction_error)-50)/10));

  motorR_power =  Kp * direction_error + mP * Kp_forward;
  motorL_power = -Kp * direction_error + mP * Kp_forward;

  int_motorR_power = (int) motorR_power;
  int_motorL_power = (int) motorL_power;

  //  Serial.print(int_motorR_power);
  //  Serial.print(",");
  //  Serial.println(int_motorL_power);
  Serial.println("");

  motor(int_motorR_power, int_motorL_power);
  smartDelay(1000);

}


double convertToGoalDistance(double Rlat, double Rlng, double Glat, double Glng) {
  double dis_lat, dis_lng, distance;
  dis_lat = (Glat - Rlat) * 110.94297;
  dis_lng = 111.3162 * cos(Rlat * M_PI / 180) * (Glng - Rlng);
  distance = sqrt(pow(dis_lat, 2) + pow(dis_lng, 2)) * 1000;
  return distance;
}

double convertToGoalCoordinates(double goal_direction, double robot_direction) {
  double robot_relative_direction;

  if (robot_direction > 180) {
    robot_direction = robot_direction - 360 ;
  }

  robot_relative_direction = robot_direction - goal_direction;
  if (robot_relative_direction > 180) {
    robot_relative_direction = robot_relative_direction - 360;
  } else if (robot_relative_direction < -180) {
    robot_relative_direction = robot_relative_direction + 360;
  }

  //  Serial.print("X: ");
  //  Serial.print(robot_direction, 4);
  //  Serial.print(" X: ");
  //  Serial.println(robot_relative_direction, 4);

  return robot_relative_direction;
}

void motor(int rightPower, int leftPower) {
  if (rightPower > 255) rightPower = 255;
  if (rightPower < -255) rightPower = -255;
  if (leftPower > 255) leftPower = 255;
  if (leftPower < -255) leftPower = -255;

  leftPower = leftPower * 0.8;

  if (rightPower > 0 && leftPower > 0) {
    motorRight  -> run(FORWARD);
    motorLeft   -> run(FORWARD);
    motorRight  -> setSpeed(rightPower);
    motorLeft   -> setSpeed(leftPower);
  }
  else if (rightPower > 0 && leftPower < 0) {
    motorRight  -> run(FORWARD);
    motorLeft   -> run(BACKWARD);
    motorRight  -> setSpeed(rightPower);
    motorLeft   -> setSpeed(-leftPower);
  }
  else if (rightPower < 0 && leftPower > 0) {
    motorRight  -> run(BACKWARD);
    motorLeft   -> run(FORWARD);
    motorRight  -> setSpeed(-rightPower);
    motorLeft   -> setSpeed(leftPower);
  }
  else if (rightPower < 0 && leftPower < 0) {
    motorRight  -> run(BACKWARD);
    motorLeft   -> run(BACKWARD);
    motorRight  -> setSpeed(-rightPower);
    motorLeft   -> setSpeed(-leftPower);
  }
  else {
    Serial.print("Stop!!");
    motorRight ->run(RELEASE);
    motorLeft  ->run(RELEASE);
  }
}

void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gps.available())
      tinyGPS.encode(gps.read());
  } while (millis() - start < ms);
}
