#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

int feedback_pin[] = {A0, A1, A2};

ros::NodeHandle  nh;
std_msgs::Int16MultiArray feedback_msg;
ros::Publisher pub("servo_feedback", &feedback_msg);
int feedback_data[3];

Servo gripper;
Servo wrist;
Servo elbow;
Servo shoulder;
Servo base;
Servo offset;

double base_angle=90;
double shoulder_angle=90;
double elbow_angle=90;
double wrist_angle=90;
double offset_angle=90;

double prev_base = 0;
double prev_shoulder = 0;
double prev_elbow = 0;
double prev_wrist = 0;

int gripperState = 0;
int positionChanged = 0;

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  base_angle=radiansToDegrees(cmd_msg.position[0]);
  shoulder_angle=radiansToDegrees(cmd_msg.position[1]);
  elbow_angle=radiansToDegrees(cmd_msg.position[2]);
  wrist_angle=radiansToDegrees(cmd_msg.position[3]);
  
  base.write(base_angle);
  shoulder.write(shoulder_angle);
  elbow.write(elbow_angle);
  wrist.write(wrist_angle);

  if (prev_base==base_angle && prev_shoulder==shoulder_angle && prev_elbow==elbow_angle && prev_wrist==wrist_angle && positionChanged==0)
  {
    if (gripperState==0)
    {
      gripper.write(60);
      gripperState = 1;
    }
    else if (gripperState==1)
    {
      gripper.write(0);
      gripperState = 0;
    }
    positionChanged = 1;
  }
  else if ((prev_base!=base_angle || prev_shoulder!=shoulder_angle || prev_elbow!=elbow_angle || prev_wrist!=wrist_angle) && positionChanged==1)
  {
    positionChanged = 0;
  }

  prev_base = base_angle;
  prev_shoulder = shoulder_angle;
  prev_elbow = elbow_angle;
  prev_wrist = wrist_angle;
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  base.attach(4);
  shoulder.attach(5); 
  elbow.attach(6);
  wrist.attach(7);
  offset.attach(8);
  gripper.attach(12); 
  
  delay(1);
  base.write(90);
  shoulder.write(90);
  elbow.write(90);
  wrist.write(90);
  offset.write(90);
  gripper.write(90);

  feedback_msg.data_length = 3;
  feedback_msg.data = feedback_data;
}

void loop(){
  feedback_msg.data[0] = map(analogRead(feedback_pin[0]),68,610,0,180);
  feedback_msg.data[1] = map(analogRead(feedback_pin[1]),61,606,0,180);
  feedback_msg.data[2] = map(analogRead(feedback_pin[2]),60,610,0,180);
  pub.publish(&feedback_msg);
  //Serial.println(analogRead(feedback_pin[0]));
  Serial.print(analogRead(feedback_pin[1]));
  //Serial.print(analogRead(feedback_pin[2]));
  nh.spinOnce();
  delay(70);
}

double radiansToDegrees(float position_radians)
{

  position_radians = position_radians + 1.6;

  return position_radians * 57.2958;

}
