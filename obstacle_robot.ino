#include <Servo.h>
#include <AFMotor.h>

#define trig_pin A2
#define echo_pin A3
#define IRsensor A0

// declare servo
Servo myservo;
// the angles of the servo
int middle = 90;
int right = 20;
int left = 160;

// define the motors
AF_DCMotor left_front(4, MOTOR34_1KHZ);
AF_DCMotor left_back(1, MOTOR12_1KHZ);
AF_DCMotor right_front(3, MOTOR34_1KHZ);
AF_DCMotor right_back(2, MOTOR12_1KHZ);

// declare the sensor variable
float duration, distance;
int sensorStatus;

void setup() {
  //Serial.begin(9600);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(IRsensor, INPUT);
  setUpSpeed();
  myservo.attach(10);
  myservo.write(middle);
  delay(5000);
}

void loop() {
  distance = getDistance();
  float distanceR,distanceL = 0;

  sensorStatus = IRdetect();
  
  // if distance less than 20 cm
  // then run backwards
  // and stop to get the new distance for turning
  if (distance <= 20.00) {
      runStop();
      delay(200);
      // if ir sensor detects nothing obstacle in the back
      // then keep moving back
      if (sensorStatus == 0) {
        //Serial.print(sensorStatus);
        //Serial.println("something is detected");
        runForward();
        delay(400);
        runStop();
        delay(200);
        distanceR = lookRight();
        delay(200);
        distanceL = lookLeft();
        delay(200);
        // if the distance of right side more than left side
        // then turn right, otherwise turn left
        if (distanceR >= distanceL) {
          turnRight();
          delay(500);
          runStop();
//        Serial.print("right");
//        Serial.println(distanceR);
        }else {
          turnLeft();
          delay(500);
          runStop();
//        Serial.print("left");
//        Serial.println(distanceL);
        }
      // or ir sensor detects any obstacle in the back
      // then keep stop motors and move forward
      }else {
        //Serial.print(sensorStatus);
        //Serial.println("nothing is detected");  
        runBackward();
        delay(400);
        runStop();
        delay(200);
        distanceR = lookRight();
        delay(200);
        distanceL = lookLeft();
        delay(200);
        // if the distance of right side more than left side
        // then turn right, otherwise turn left
        if (distanceR >= distanceL) {
          turnRight();
          delay(500);
          runStop();
//      Serial.print("right");
//      Serial.println(distanceR);
        }else {
          turnLeft();
          delay(500);
          runStop();
//      Serial.print("left");
//      Serial.println(distanceL);
        }
 
      }
    
    }else {
      runForward();
  }
  
}

// get the distance
float getDistance() {
  // set the status of trgger pin to be deactivated
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // active the trigger pin to send the signal
  digitalWrite(trig_pin, HIGH);
  // needs 10 us to send the signal
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  // read the pulse signal
  duration = pulseIn(echo_pin, HIGH);
  // calculate the distance, sounds speed equal to 340m/s or 0.034cm/us, v = s/t
  distance = duration * 0.034 / 2;
  delay(50);
  return distance;

}

// detect obstacle of back side
int IRdetect() {
  sensorStatus = digitalRead(IRsensor);

  if (sensorStatus == 1) {
    //delay(50);
    return sensorStatus;
  }else {
    //delay(50);
    return sensorStatus;
  }
}

// run forward
void runForward() {
    left_front.run(FORWARD);
    left_back.run(FORWARD);
    right_front.run(FORWARD);
    right_back.run(FORWARD);
}

// run backward
void runBackward() {
  left_front.run(BACKWARD);
  left_back.run(BACKWARD);
  right_front.run(BACKWARD);
  right_back.run(BACKWARD);
}

// stop the motors
void runStop() {
  left_front.run(RELEASE);
  left_back.run(RELEASE);
  right_front.run(RELEASE);
  right_back.run(RELEASE);
}

// turn servo to right side to check the distance
float lookRight() {
  myservo.write(right);
  delay(500);
  float distancne = getDistance();
  delay(100);
  myservo.write(middle);
  return distance;
}

// turn servo to left side to check the distance
float lookLeft() {
  myservo.write(left);
  delay(500);
  float distancne = getDistance();
  delay(100);
  myservo.write(middle);
  return distance;
}

// turn right
void turnRight() {
  left_front.run(FORWARD);
  left_back.run(FORWARD);
  right_front.run(BACKWARD);
  right_back.run(BACKWARD);
}

// turn left
void turnLeft() {
  right_front.run(FORWARD);
  right_back.run(FORWARD);
  left_front.run(BACKWARD);
  left_back.run(BACKWARD);
}

// set up the speed of motors
void setUpSpeed() {
  left_front.setSpeed(170);
  left_back.setSpeed(170);
  right_front.setSpeed(170);
  right_back.setSpeed(170);
}
