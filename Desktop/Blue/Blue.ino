////////////////////Ground Robot (blue)///////////////////
/*


*/
//////////////////////////////////////////////////////////
//Ros Setup

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
ros::NodeHandle nh;
//
//Sensor Libraries
#include "MQ5.h"
#include "TFMini.h"
#include "LIDARLite.h"
//
//External Hardware Libraries
#include "Servo.h"
//
//////////ROS INPUT MESSAGES/////////////////////////////
//MESSAGE CALLBACKS
int s1 = 0;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
int s6 = 0;
int Cam = 0;
int IR = 0;
int VIS = 0;
int V = 0;
int I = 0;
int Rs2i = 0;
int IMU_X = 0;
int IMU_Y = 0;
int IMU_Z = 0;
void s1Cb(const std_msgs::UInt16 &msg)
{
  s1 = msg.data;
}
void s2Cb(const std_msgs::UInt16 &msg)
{
  s2 = msg.data;
}
void s3Cb(const std_msgs::UInt16 &msg)
{
  s3 = msg.data;
}
void s4Cb(const std_msgs::UInt16 &msg)
{
  s4 = msg.data;
}
void s5Cb(const std_msgs::UInt16 &msg)
{
  s5 = msg.data;
}
void s6Cb(const std_msgs::UInt16 &msg)
{
  s6 = msg.data;
}
void CAMN(const std_msgs::UInt16 &msg)
{
  Cam = msg.data;
}
void IRN(const std_msgs::UInt16 &msg)
{
  IR = msg.data;
}
void VISN(const std_msgs::UInt16 &msg)
{
  VIS = msg.data;
}
void VN(const std_msgs::UInt16 &msg)
{
  V = msg.data;
}
void IN(const std_msgs::UInt16 &msg)
{
  I = msg.data;
}
void RSSIN(const std_msgs::UInt16 &msg)
{
  Rs2i = msg.data;
}
void IMUXN(const std_msgs::UInt16 &msg)
{
  IMU_X = msg.data;
}
void IMUYN(const std_msgs::UInt16 &msg)
{
  IMU_Y = msg.data;
}
void IMUZN(const std_msgs::UInt16 &msg)
{
  IMU_Z = msg.data;
}
//Ros subscribers
ros::Subscriber<std_msgs::UInt16> s1r("S1", &s1Cb);
ros::Subscriber<std_msgs::UInt16> s2r("S2", &s2Cb);
ros::Subscriber<std_msgs::UInt16> s3r("S3", &s3Cb);
ros::Subscriber<std_msgs::UInt16> s4r("S4", &s4Cb);
ros::Subscriber<std_msgs::UInt16> s5r("S5", &s5Cb);
ros::Subscriber<std_msgs::UInt16> s6r("S6", &s6Cb);
ros::Subscriber<std_msgs::UInt16> Camr("CAM", &CAMN);
ros::Subscriber<std_msgs::UInt16> IRr("IR", &IRN);
ros::Subscriber<std_msgs::UInt16> VISr("VIS", &VISN);
ros::Subscriber<std_msgs::UInt16> Vr("V", &VN);
ros::Subscriber<std_msgs::UInt16> Ir("I", &IN);
ros::Subscriber<std_msgs::UInt16> Rs2ir("RSSI", &RSSIN);
ros::Subscriber<std_msgs::UInt16> IMU_Xr("IMU_X", &IMUXN);
ros::Subscriber<std_msgs::UInt16> IMU_Yr("IMU_Y", &IMUYN);
ros::Subscriber<std_msgs::UInt16> IMU_Zr("IMU_Z", &IMUZN);
//////////ROS OUTPUT MESSAGES////////////////////////////
std_msgs::UInt16 str_msg;//NATURAL GAS
ros::Publisher NG("NG", &str_msg);

std_msgs::UInt16 LEFTMSG;//left
ros::Publisher LR("LR", &LEFTMSG);

std_msgs::UInt16 RIGHTMSG;//right
ros::Publisher RR("RR", &RIGHTMSG);

std_msgs::UInt16 BACKMSG;//back
ros::Publisher BR("BR", &BACKMSG);

std_msgs::UInt16 FRONTMSG;//front
ros::Publisher FR("FR", &FRONTMSG);

std_msgs::UInt16 HUMSG;//Humidity
ros::Publisher HU("HU", &HUMSG);
/////////////////////////////////////////////////////////
//////////HARDWARE INITIALIZATIONS/////////////////////////
MQ5 mq5;
TFMini left;
TFMini right;
TFMini back;
LIDARLite front;
int cal_cnt = 0;
Servo deploy1;
/////////////////////////////////////////////////////////

void setup()
{
  nh.initNode();
  pinMode(29, OUTPUT);
  pinMode(A6, INPUT);
  deploy1.attach(29);
  front.begin(0, true);
  front.configure(0);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(9600);
  mq5.attachPin(A6);
  nh.advertise(NG);
  nh.advertise(LR);
  nh.advertise(RR);
  nh.advertise(BR);
  nh.advertise(FR);
  nh.advertise(HU);
  nh.subscribe(s1r);
  nh.subscribe(s2r);
  nh.subscribe(s3r);
  nh.subscribe(s4r);
  nh.subscribe(s5r);
  nh.subscribe(s6r);
  nh.subscribe(Camr);
  nh.subscribe(IRr);
  nh.subscribe(VISr);
  nh.subscribe(Vr);
  nh.subscribe(Ir);
  nh.subscribe(Rs2ir);
  nh.subscribe(IMU_Xr);
  nh.subscribe(IMU_Yr);
  nh.subscribe(IMU_Zr);
  mq5.Cal();
  left.begin(&Serial1);
  right.begin(&Serial2);
  back.begin(&Serial3);

}

void loop()
{
  delay(15);
  int dist;
  int hum;
  //READ RANGES/////////////////////////////////////////////
  uint16_t distl = left.getDistance();
  uint16_t strengthl = left.getRecentSignalStrength();

  uint16_t distr = right.getDistance();
  uint16_t strengthr = right.getRecentSignalStrength();

  uint16_t distb = back.getDistance();
  uint16_t strengthb = back.getRecentSignalStrength();
  //
  if ( cal_cnt == 0 ) {
    dist = front.distance();      // With bias correction
  } else {
    dist = front.distance(false); // Without bias correction
  }
  cal_cnt++;
  cal_cnt = cal_cnt % 100;
  //////////////////////////////////////////////////////////
  str_msg.data = mq5.PPM();
  LEFTMSG.data = distl;
  RIGHTMSG.data = distr;
  BACKMSG.data = distb;
  FRONTMSG.data = dist;
  HUMSG.data = hum;
  HU.publish( &HUMSG );
  NG.publish( &str_msg );
  FR.publish( &FRONTMSG );
  LR.publish( &LEFTMSG );
  RR.publish( &RIGHTMSG );
  BR.publish( &BACKMSG );
  nh.spinOnce();
}
