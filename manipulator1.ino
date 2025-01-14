#include <Herkulex.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>
int n = 6; // every motor has different id;
int m = 11;
int l = 14;
Servo myservo;
Servo myservo2;
typedef struct thetas{
  float theta1;
  float theta2;
  float theta3;
}theta;
int numbers[3];
void setup() {
  myservo.attach(7);
  myservo2.attach(4);
  delay(200);  // a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.begin(115200, 10, 11); // open serial with rx=10 and tx=11 
  Herkulex.reboot(m);
  delay(500); 
  Herkulex.initialize(); // initialize motors
  delay(200);
}

theta inversekinematics(float x, float y, float z) {
    theta t;
    double t1 = atan2(y, x);
    double a = 15;
    double b = 15;
    double c = x / cos(t1);
    double d = z;
    double c3 = (pow(c, 2) + pow(d, 2) - pow(a, 2) - pow(b, 2)) / (2 * a * b);
    double s3 = sqrt(1 - pow(c3, 2));
    double t3 = atan2(s3, c3);
    double r = a * c3 + b;
    double s = a * s3;
    double t2 = atan2(r * d - s * c, r * c + s * d);
    t.theta1 = t1*180/3.14;
    t.theta2 = t2*180/3.14;
    t.theta3 = t3*180/3.14;
    return t;
}

void run(float x,float y,float z,float phi){
  thetas t = inversekinematics(x, y, z); // Corrected return type
  float t1 = t.theta1;
  float t2 = t.theta2;
  float t3 = t.theta3;
  float t4 = phi + t2 + t3 -180;
  m = 10;
  Serial.println(t1);
  Herkulex.moveOneAngle(m, 0, 750, LED_BLUE); // move motor with 300 speed  
  delay(1200);
  Herkulex.moveOneAngle(m, t1, 750, LED_BLUE); // move motor with 300 speed  
  m = 1;
  Serial.println(t2);
  Herkulex.moveOneAngle(m, 0, 750, LED_BLUE); //move motor with 300 speed  
  delay(1200);
  Herkulex.moveOneAngle(m, t2, 750, LED_BLUE); //move motor with 300 speed  
  m = 4;
  Serial.println(t3);
  Herkulex.moveOneAngle(m, 0, 750, LED_BLUE); //move motor with 300 speed  
  delay(1200);
  Herkulex.moveOneAngle(m, t3, 750, LED_BLUE); //move motor with 300 speed  
  delay(1200);
  m = 253;
  Serial.println(t4);
  Herkulex.moveOneAngle(m, 0, 750, LED_BLUE); //move motor with 300 speed  
  delay(1200);
  Herkulex.moveOneAngle(m, t4, 750, LED_BLUE); //move motor with 300 speed  
  delay(1200);

  int pos2 = 0;
  for (pos2 = 0; pos2 <= 120; pos2 += 1) { // goes from 0 degrees to 180 degrees
    myservo2.write(pos2);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}
void loop() {
  Serial.println("T");
  String receivedData;
  if (Serial.available()) {
    receivedData = Serial.readStringUntil('\n');
    String tempNumber = "";

  for (int i = 0; i < receivedData.length(); i++) {
    char currentChar = receivedData[i];
      if (isDigit(currentChar) || currentChar == '.' || currentChar == '-') {
        tempNumber += currentChar;
      } else if (tempNumber.length() > 0) {
      if (numbersCount < maxNumbers) {
        numbers[numbersCount++] = tempNumber.toFloat();
      }
        tempNumber = ""; // Reset for the next number
      }
    }
    if (tempNumber.length() > 0 && numbersCount < maxNumbers) {
      numbers[numbersCount++] = tempNumber.toFloat();
    }
    run(numbers[0],number[1],numbers[2],90);
  }
  delay(2000);
}
