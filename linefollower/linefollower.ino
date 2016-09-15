#include <QTRSensors.h>


//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

#define Kp 0.03 
#define Ki 0.0000001
#define Kd 0.2 // Kp < Kd, 1 min 50s with new modded model!!
#define rightMaxSpeed 120 // max capped speed of the robot
#define leftMaxSpeed 120 // max capped speed of the robot
#define rightBaseSpeed 45 // basic speed when on the line
#define leftBaseSpeed 45  // basic speed when on the line
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

#define NUM_SENSORS             5  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
//#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  1, 2, 3, 4, 5
},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
unsigned int sensorValues[NUM_SENSORS];


void setup()
{

  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

//CALIBRATION CODE
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  //END CALIBRATION CODE

  //wait();
  delay(3000); // position the bot
}

int lastError = 0;
long integral = 0;

void loop()
{
  unsigned int sensors[5];
  int position = qtra.readLine(sensors);
  Serial.println(position);
  //Serial.print(analogRead(A0)); 
  //Serial.print(", ");
  Serial.print(analogRead(A1));
  Serial.print(", ");
  Serial.print(analogRead(A2));
  Serial.print(", ");
  Serial.print(analogRead(A3));
  Serial.print(", ");
  Serial.print(analogRead(A4));
  Serial.print(", ");
  Serial.print(analogRead(A5));
  Serial.print(", ");
  Serial.print(position);
  int error = position - 2000;
  Serial.print(", error:");
  Serial.print(error);
  integral += error;
  Serial.print(",  integral:");
  Serial.print(Ki*integral);
  int motorSpeed = Kp * error +  (Ki * integral) + Kd * (error - lastError);
  lastError = error;
  Serial.print(", motorSpeed:");
  Serial.print(motorSpeed);

  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;



  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  {
    Serial.print(", Rspeed:");
    Serial.print(rightMotorSpeed);
    Serial.print(", Lspeed:");
    Serial.println(leftMotorSpeed);
    //  digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    //  digitalWrite(M1, HIGH);
    //  digitalWrite(rightMotor2, LOW);
    digitalWrite(M1, HIGH);
    analogWrite(E1, rightMotorSpeed);
    //  digitalWrite(motorPower, HIGH);
    //  digitalWrite(M2, HIGH);
    //  digitalWrite(leftMotor2, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E2, leftMotorSpeed);
  }
}

/*
void wait() {
  // digitalWrite(motorPower, LOW);
  analogWrite(E1, 0);
  analogWrite(E2, 0);

}
*/
