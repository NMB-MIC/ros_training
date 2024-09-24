#include "Config.h"

//---------------------------------------------Setting for communication------------------------------------------------------//
using rosserial_arduino::Test;

SoftwareSerial mySerial(pin_IMU_RX, pin_IMU_TX);
DFRobot_WT61PC sensor(&mySerial);

///Config INA266
INA226_WE ina226 = INA226_WE(I2C_INA226);

ros::NodeHandle nh;

//-------------------------------------------------------------Setting for ROS Publish-----------------------------------------------------------------------//
// Pub data for odom and tf
std_msgs::Float32MultiArray forOdom_msg;                 //type_msg::type_data_in_msg name_of_msg;
ros::Publisher FORODOM_("forOdom_topic", &forOdom_msg);  //ros::Publisher variable_for_Publisher("topic_name", &name_of_msg);
// Pub encoder for each wheel (tick)
std_msgs::Int64MultiArray enc_msg;
ros::Publisher ENC_("enc_topic", &enc_msg);
// Pub Target for each wheel (m/s)
std_msgs::Float32MultiArray target_msg;
ros::Publisher TARGET_("target_topic", &target_msg);
// Pub Actual for each wheel (m/s)
std_msgs::Float32MultiArray actual_msg;
ros::Publisher ACTUAL_("actual_topic", &actual_msg);
// Pub Battery (volte)
std_msgs::Float32 battery_msg;
ros::Publisher BATTERY_("battery_topic", &battery_msg);
// Pub PWM (-1023-1023,-100 to 100 percent)
std_msgs::Int32MultiArray pwm_msg;
ros::Publisher PWM_("pwm_topic", &pwm_msg);

//-------------------------------------------------------------Setting for ROS Subscription-----------------------------------------------------------------------//
//CMD_vel_callback
void CMD_vel_callback(const geometry_msgs::Twist &CVel_msg) {
  Cvel_x = CVel_msg.linear.x; // m/s
  //Cvel_y = CVel_msg.linear.y; // m/s
  Cvel_z = CVel_msg.angular.z; // red/s
  if (Cvel_x == 0 && Cvel_y == 0 && Cvel_z > 0.5) {
    Cvel_z = 0.5;
  }
  vel_tar[0] = (Cvel_x - (lx / 2) * Cvel_z); // m/s
  vel_tar[1] = (Cvel_x + (lx / 2) * Cvel_z); // m/s
  if (Cvel_x == 0 && Cvel_y == 0 && Cvel_z == 0) {
    state_agv = 1;  //blue parking
  } else if (Cvel_x != 0 || Cvel_y != 0 || Cvel_z != 0) {
    state_agv = 2;  //green moving
  }
}


ros::Subscriber<geometry_msgs::Twist> SUB_CMDvel("/cmd_vel", &CMD_vel_callback);

//-------------------------------------------------------------Setting for ROS Service-----------------------------------------------------------------------//
ros::ServiceServer<Test::Request, Test::Response> clear_data("clear_data", &srv_clear_data);

//------------------------------------------------------------------Main Function for Void Setup-----------------------------------------------------------------//
//gpio_define
void gpio_define() {
  for (int i = 0; i < 2; i++) {
    pinMode(pin_EA[i], INPUT_PULLUP);
    pinMode(pin_EB[i], INPUT_PULLUP);
    pinMode(pin_DIR[i], OUTPUT);
    pinMode(pin_PWM[i], OUTPUT);
    ledcSetup(pwmChannel[i], freq, resolution);
    ledcAttachPin(pin_PWM[i], pwmChannel[i]);
    pwm_value[i] = 0;
    pwm_be_sat[i] = 0;
    vel_tar[i] = 0;
    cur_enc[i] = 0;
    prv_enc[i] = 0;
    cur_vel_act[i] = 0;
    prv_vel_act[i] = 0;
    cur_vel_flt[i] = 0;
  }
  attachInterrupt(digitalPinToInterrupt(pin_EA[0]), enc_count0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_EA[1]), enc_count1, CHANGE);
}
//IMU_define
void IMU_define() {
  //Use software serial port mySerial as communication seiral port
  mySerial.begin(9600);
  sensor.modifyFrequency(FREQUENCY_100HZ);
  bool start_IMU = true;
  while (start_IMU) {
    if (sensor.available()) {
      ofs_IMU = sensor.Angle.Z;
      prv_IMU = sensor.Angle.Z;
      start_IMU = false;
    }
    if (iter_IMU >= 1000) {
      start_IMU = false;
    }
    iter_IMU++;
    delay(5);
  }
  count_IMU = 0;
}
//INA226_define
void INA226_define() {
  // Serial.println("INA266 NG");
  Wire.begin(pin_SCL, pin_SDA);
  if (!ina226.init()) {
    // Serial.println("Failed to init INA226. Check your wiring.");
    while (1) {
      batt = -999;
    }
    delay(1000);
  }
  ina226.waitUntilConversionCompleted();
}
//RGB_define
void RGB_define() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
}
//------------------------------------------------------------------Main Function for Void LOOP-----------------------------------------------------------------//
//calculate_vel_act
void calculate_vel_act() {
  d_time0 = millis() - prv_time0;
  prv_time0 = millis();
  for (int i = 0; i < 2; i++) {  // for encoder type 1024 tpr
    distance[i] = float(cur_enc[i] - prv_enc[i]) / TPM; //convert to m
    cur_vel_act[i] = 1000 * (distance[i] / d_time0); // convert to m/s
    if (vel_tar[i] == 0 && cur_vel_act[i] == 0) {
      cur_vel_flt[i] = 0;
    } else {
      cur_vel_flt[i] = 0.8 * cur_vel_flt[i] + 0.2 * cur_vel_act[i]; //Low pass fillter use this to calculate with PID
    }
    prv_vel_act[i] = cur_vel_act[i];
    prv_enc[i] = cur_enc[i];
  }
  // forward kinematic
  vel_x = (cur_vel_flt[0] + cur_vel_flt[1]) / 2;
  vel_y = 0;
  d_XTravel = (distance[0] + distance[1]) / 2;
  x += d_XTravel * cos(cur_th * M_PI / 180); // distance x from origine
  y += d_XTravel * sin(cur_th * M_PI / 180); // distance y from origine
}
//pid_control
void pid_control() {
  d_time1 = millis() - prv_time1;
  prv_time1 = millis();
  for (int i = 0; i < 2; i++) {
    cur_err[i] = vel_tar[i] - cur_vel_flt[i];
    D_err[i] = 1000 * (cur_err[i] - prv_err[i]) / d_time1;
    prv_err[i] = cur_err[i];

    //Anti Windup-error
    if ((abs(pwm_be_sat[i]) > abs(pwm_sat[i])) && (findSign(cur_err[i]) == findSign(pwm_be_sat[i]))) {
      err_I[i] = 0;
    } else {
      err_I[i] = cur_err[i];
    }
    I_err[i] = I_err[i] + err_I[i] * d_time1 / 1000;
    pwm_be_sat[i] = round(Kp[i] * cur_err[i] + Ki[i] * I_err[i] + Kd[i] * D_err[i]);
    //Saturation
    if (pwm_be_sat[i] > 1023) {
      pwm_sat[i] = 1023;
    } else if (pwm_be_sat[i] < -1023) {
      pwm_sat[i] = -1023;
    } else {
      pwm_sat[i] = round(pwm_be_sat[i]);
    }
    pwm_value[i] = abs(pwm_sat[i]);
    //Set Direction
    if (vel_tar[i] > 0) {
      dir[i] = 1;
      if (pwm_sat[i] < 0) {
        pwm_value[i] = 0;
      }
    } else if (vel_tar[i] < 0) {
      dir[i] = 0;
      if (pwm_sat[i] > 0) {
        pwm_value[i] = 0;
      }
    } else {
      pwm_value[i] = 0;
    }
    //the output from this function is pwm_value and direction
  }
}
//agv_run
void agv_run() {

  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      dir[0] = !dir[0]; //revese direction of wheel 0
    }
    digitalWrite(pin_DIR[i], dir[i]);
    ledcWrite(pwmChannel[i], pwm_value[i]);
  }
}
//LED
void status_() {
  switch (state_agv) {
    case 0:
      setAllLeds(CRGB::Red);  // สีแดง
      break;
    case 1:
      setAllLeds(CRGB::Blue);  // สีน้ำเงิน
      break;
    case 2:
      setAllLeds(CRGB::Green);  // สีเขียว
      break;
    case 3:
      setAllLeds(CRGB::Yellow);  // สีเหลือง
      break;
    default:
      setAllLeds(CRGB::Blue);  // สีน้ำเงินในกรณีที่ค่า state_agv ไม่ตรงกับ 0, 1 หรือ 2
      break;
  }
}

void clear_() {
  //  Clear enc_data
  if ((cur_vel_act[0] == 0 && cur_vel_act[1] == 0 && vel_tar[0] == 0 && vel_tar[1] == 0)) {
    for (int i = 0; i < 2; i++) {
      prv_enc[i] = 0;
      cur_enc[i] = 0;
      cur_vel_flt[i] = 0;
      pwm_value[i] = 0;
      pwm_sat[i] = 0;
      pwm_be_sat[i] = 0;
      I_err[i] = 0;
    }
  }
}

void voltage_fun() {
  batt = ina226.getBusVoltage_V() + (ina226.getShuntVoltage_mV() / 1000);
  // batt = ((0.9472 * batt) - 0.0371);
}

void IMU_fun() {
  if (sensor.available()) {
    d_time2 = millis() - prv_time2;
    prv_time2 = millis();
    cur_IMU = sensor.Angle.Z;
    if (prv_IMU - cur_IMU > 300) {
      count_IMU++;
    } else if (cur_IMU - prv_IMU > 300) {
      count_IMU--;
    }
    prv_IMU = cur_IMU;
    cur_th = 360 * count_IMU + cur_IMU - ofs_IMU; // Degree
    w_th = 1000 * (cur_th - prv_th) / d_time2; // Degree/sec
    prv_th = cur_th;
  }
}

//------------------------------------------------------------------Function for Publish Data-----------------------------------------------------------------------//
void enc_publish() {
  long long int enc_p[2] = { cur_enc[0], cur_enc[1] };
  enc_msg.data = enc_p;
  enc_msg.data_length = 2;
  ENC_.publish(&enc_msg);
}
void target_publish() {
  float target_p[2] = { vel_tar[0], vel_tar[1] };
  target_msg.data = target_p;
  target_msg.data_length = 2;
  TARGET_.publish(&target_msg);
}
void actual_publish() {
  float actual_p[2] = { cur_vel_flt[0], cur_vel_flt[1] };
  actual_msg.data = actual_p;
  actual_msg.data_length = 2;
  ACTUAL_.publish(&actual_msg);
}
void pwm_publish() {
  int pwm_p[2] = { pwm_value[0], pwm_value[1] };
  pwm_msg.data = pwm_p;
  pwm_msg.data_length = 2;
  PWM_.publish(&pwm_msg);
}
void battery_publish() {
  float battery_p = batt;
  battery_msg.data = battery_p;
  BATTERY_.publish(&battery_msg);
}
void forOdom_publish() {
  float forOdom_p[6] = { x, y, cur_th, vel_x, vel_y, w_th }; // {m,m,degree,m/s,m/s,degree/sec}
  forOdom_msg.data = forOdom_p;
  forOdom_msg.data_length = 6;
  FORODOM_.publish(&forOdom_msg);
}

//------------------------------------------------------------------Function for ROS Service-----------------------------------------------------------------------//
void srv_clear_data(const Test::Request &req, Test::Response &res) {
  bool start_IMU = true;
  while (start_IMU) {
    if (sensor.available()) {
      ofs_IMU = sensor.Angle.Z;
      prv_IMU = sensor.Angle.Z;
      start_IMU = false;
    }
    if (iter_IMU >= 1000) {
      start_IMU = false;
    }
    iter_IMU++;
    delay(5);
  }
  count_IMU = 0;
  x = 0;
  y = 0;
  for (int i = 0; i < 2; i++) {
    pwm_value[i] = 0;
    pwm_be_sat[i] = 0;
    I_err[i] = 0;
    vel_tar[i] = 0;
    cur_enc[i] = 0;
    prv_enc[i] = 0;
    cur_vel_act[i] = 0;
    prv_vel_act[i] = 0;
    cur_vel_flt[i] = 0;
  }
  res.output = "Data has been cleared ";
}

//----------------------------------------------------------------------------Void Setup-------------------------------------------------------------------------//
void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  //-----------------Publisher advertise
  nh.advertise(FORODOM_);
  nh.advertise(ENC_);
  nh.advertise(TARGET_);
  nh.advertise(ACTUAL_);
  nh.advertise(BATTERY_);
  nh.advertise(PWM_);
  //------------------Service advertise
  nh.advertiseService(clear_data);
  //-------------------Subscribe advertise
  nh.subscribe(SUB_CMDvel);
  //-------------------main function for setup loop
  gpio_define();
  IMU_define();
  INA226_define();
  RGB_define();
}

//----------------------------------------------------------------------------Void LOOP-------------------------------------------------------------------------//
void loop() {
  d_time3 = millis() - prv_time3;
  if (d_time3 >= 30) {
    prv_time3 = millis();
    calculate_vel_act();
    pid_control();
    agv_run();
    status_();
    clear_();
    voltage_fun();
    enc_publish();
    target_publish();
    actual_publish();
    pwm_publish();
  }
  d_time4 = millis() - prv_time4;
  if (d_time4 >= 1000) {
    prv_time4 = millis();
    battery_publish();
  }
  IMU_fun();
  forOdom_publish();
  nh.spinOnce();
}

//--------------------------------------------------------------------------Encoder Interupt Function---------------------------------------------------------------------//
void enc_count0() {
  if (digitalRead(pin_EB[0]) == 0) {
    if (digitalRead(pin_EA[0]) == 0) {
      cur_enc[0]++;
    } else {
      cur_enc[0]--;
    }
  }
}
void enc_count1() {
  if (digitalRead(pin_EB[1]) == 0) {
    if (digitalRead(pin_EA[1]) == 0) {
      cur_enc[1]++;
    } else {
      cur_enc[1]--;
    }
  }
}


//--------------------------------------------------------------------------Others Function---------------------------------------------------------------------//
int findSign(float number) {
  if (number > 0) {
    return 1;
  } else if (number < 0) {
    return -1;
  } else {
    return 0;
  }
}
// ฟังก์ชันสำหรับการเปลี่ยนสีทั้งหมดของแถบ LED
void setAllLeds(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    // เช็คว่าตำแหน่ง LED เป็น 0, 10, 11 หรือ 21
    if (i == 0 || i == 10 || i == 11 || i == 21) {
      leds[i] = CRGB::Black;  // ตั้งค่าให้หลอด LED ตำแหน่งนี้เป็นสีดำ (ปิดไฟ)
    } else {
      leds[i] = color;  // ตั้งค่าหลอด LED ที่เหลือให้เป็นสีที่ต้องการ
    }
  }
  FastLED.show();  // แสดงผลการเปลี่ยนแปลง
}
