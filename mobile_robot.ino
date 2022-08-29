#include <ros.h> 
#include <std_msgs/Int32.h>

const int IN1 = 4;
const int IN2 = 5;
const int PWM1 = 6;
const int IN3 = 8;
const int IN4 = 9;
const int PWM2 = 10;

ros::NodeHandle  nh;

void messageCb(const std_msgs:: Int32& msg)

{
  if(msg.data == 1)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM1, 50);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWM2, 50);
    Serial.println("Straight");
  }
  if(msg.data ==3)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM1, 100);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWM2, 35);
    Serial.println("right");
  }
  if(msg.data ==2)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM1, 75);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWM2, 40);
    Serial.println("slight right");
  }
  if(msg.data ==5)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM1, 35);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWM2, 100);
    Serial.println("Left");
  }
  if(msg.data ==4)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM1, 40);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWM2, 750);
    Serial.println("Slight Left");
  }
}

ros::Subscriber<std_msgs::Int32> sub("direction", &messageCb);

void setup() 
{
  nh.initNode();
  nh.subscribe(sub);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (PWM1, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (PWM2, OUTPUT);
}

void loop() 
{
  nh.spinOnce();
  delay(1000);
}
