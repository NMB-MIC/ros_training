/*
 * 
 * 
 * 
 * 
 * 
 * 
   rosserial PubSub Example
   Prints "hello world!" and toggles led
*/

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <AccelStepper.h>

#define DIR_PIN 4 //step pin
#define STEP_PIN 5//pin step
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, STEP_PIN, DIR_PIN);

ros::NodeHandle  nh;

int mg=0;
unsigned long prvtime,prvtime_main;
bool ready_mg400;

void subrelay( const std_msgs::Int32& ir1_msg) {
  int i = ir1_msg.data;
  if (i == 1) {
    digitalWrite(13 , HIGH );
  } else {
    digitalWrite(13 , LOW );
  }

}

void subconv1( const std_msgs::Int32& Auto_msg) {

  mg = Auto_msg.data;

}

 
void conv2() {

  if (mg == 0) {
    
    digitalWrite(14 , 1);
  } else {
    delay(1000);
    digitalWrite(14 , 0);
  }

}

void sublamp( const std_msgs::Int32& swb_msg) {
  int i = swb_msg.data;
  if (i ==1) {
    digitalWrite(12 , HIGH );
  } else {
    digitalWrite(12 , LOW );
  }

}

std_msgs::Int32 sp_msg;
ros::Publisher SP("sp", &sp_msg);

void submotor( const std_msgs::Int32MultiArray& swc_msg) {
  int i = swc_msg.data[0] * swc_msg.data[1];
  stepper.setSpeed(i);
  stepper.runSpeed();
}

ros::Subscriber<std_msgs::Int32> relay("ir1", &subrelay);

ros::Subscriber<std_msgs::Int32> conv_1("/Auto", &subconv1);

//ros::Subscriber<std_msgs::Int32> conv_2("conv2", &subconv2);

ros::Subscriber<std_msgs::Int32> lamp("swb", &sublamp);

ros::Subscriber<std_msgs::Int32MultiArray> motor("swc", &submotor);

std_msgs::Int32 ir1_msg;
ros::Publisher IR1("ir1", &ir1_msg);

std_msgs::Int32 Auto_msg;
ros::Publisher IR2("/Auto", &Auto_msg);

std_msgs::Int32 swb_msg;
ros::Publisher but("swb", &swb_msg);

std_msgs::Int32MultiArray swc_msg;
ros::Publisher swcm("swc", &swc_msg);

std_msgs::Int32MultiArray msg1_msg;
ros::Publisher msg1("msg", &msg1_msg);


void setup_pin()
{

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(39, INPUT_PULLUP);
  pinMode(38, INPUT_PULLUP);
  pinMode(37, INPUT_PULLUP);
  pinMode(36, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);
}

void ir_1() {
  int rir1 = 0;
  rir1 = digitalRead(36);
  ir1_msg.data = rir1;
  IR1.publish( &ir1_msg );
}

void ir_2() {
  //  int rir2 = 0;
  if (mg == 0) {
    int rir2 = digitalRead(35);
    if (rir2 == 1) {
      Auto_msg.data = 1;
      IR2.publish( &Auto_msg );
      mg = -1;
      prvtime = millis();
    }
  }
  if (((millis() - prvtime) >= 3000) && mg == -1) {
    mg = 0;
  }

}

void sw_b() {
  int rswb = 0;
  rswb = !digitalRead(39);
  swb_msg.data = rswb;
  but.publish( &swb_msg );
}

void sw_c() {
  int m[2];
  int rswcu = digitalRead(38);
  int rswcd = digitalRead(37);
  if (rswcu == 0) {
    m[0] = -1;
  } else if (rswcd == 0) {
    m[0] = 1;
  } else {
    m[0] = 0;
  }
  m[1] = 500;
  swc_msg.data = m;
  swc_msg.data_length = 2;
  swcm.publish( &swc_msg );
}

void msg() {
  int msg[1];
  msg[0] = mg;
  msg1_msg.data = msg;
  msg1_msg.data_length = 1;
  msg1.publish( &msg1_msg );
}

void check_ready()
{
  if (mg == -3)
  {
    ready_mg400 = true;
    }
  }


void setup()
{
  setup_pin();
  nh.initNode();
  nh.advertise(IR1);
  nh.advertise(IR2);
  nh.advertise(swcm);
  nh.advertise(but);
  nh.advertise(msg1);
  nh.subscribe(relay);
  nh.subscribe(lamp);
  nh.subscribe(conv_1);
  //  nh.subscribe(conv_2);
  nh.subscribe(motor);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  digitalWrite(14 , 0);
//  Serial.begin(9600);
  prvtime = millis();
  prvtime_main = millis();
  mg = 0;

}

void loop()
{
 if(millis()-prvtime_main>=40)
 {
  prvtime_main = millis();
  ir_1();
  ir_2();
  conv2();//Control conveyer
  sw_b();
  sw_c();
  msg(); //Check state for MG400 and conveyer
 }
  nh.spinOnce();

}
