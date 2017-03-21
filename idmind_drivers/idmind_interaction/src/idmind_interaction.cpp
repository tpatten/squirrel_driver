#include "idmind_interaction/idmind_interaction.h"

IdmindInteraction::IdmindInteraction()
    : n_("~"), serial_("/dev/idmind-interactionboard",115200, 5), idmind_ns_("squirrel_robot"), green_light(false), last_head_vel_(0), last_neck_vel_(0), last_cam_vel_(0),
    send_head_(false), send_neck_(false), send_headneck_(false), send_camera_(false), final_pos_(0), head_position_(0), neck_position_(0), camera_position_(0), dist_inc_(6),send_leds_(false),
    send_leds_base_(false),send_leds_mouth_(false),BASE_LEDS(42),MOUTH_LEDS(4)
{
  head_sub_ = n_.subscribe<idmind_interaction::MotorControl>("head", 100, &IdmindInteraction::headCallback, this);
  neck_sub_ = n_.subscribe<idmind_interaction::MotorControl>("neck", 100, &IdmindInteraction::neckCallback, this);
  head_neck_sub_ = n_.subscribe<idmind_interaction::MotorControl>("head_and_neck", 100, &IdmindInteraction::headNeckCallback, this);
  camera_sub_ = n_.subscribe<idmind_interaction::MotorControl>("camera_tilt", 100, &IdmindInteraction::cameraTiltCallback, this);

  leds_sub_ = n_.subscribe<idmind_interaction::LedControl>("leds", 100, &IdmindInteraction::ledsCallback, this);
  leds_base_sub_ = n_.subscribe<idmind_interaction::LedControlBase>("leds_base", 100, &IdmindInteraction::ledsBaseCallback, this);
  leds_mouth_sub_ = n_.subscribe<idmind_interaction::LedControlMouth>("leds_mouth", 100, &IdmindInteraction::ledsMouthCallback, this);

  head_position_pub_ = n_.advertise<std_msgs::Int64>("head_position", 100);
  neck_position_pub_ = n_.advertise<std_msgs::Int64>("neck_position", 100);
  camera_position_pub_ = n_.advertise<std_msgs::Int64>("camera_position", 100);

  ros::param::param<bool>("~initialize", initialize_, true);

  motor_head_ = 0;
  motor_neck_ = 1;
  motor_camera_ = 2;
  set_position_command_ = 0x30;
  set_velocity_command_ = 0x31;
  get_position_command_ = 0x52;
  set_base_leds_command_ = 0x35;
  set_mouth_leds_command_ = 0x3A;
  set_num_base_leds_command_ = 0x34;

  if (serial_.ready_)
  {
      ROS_INFO("\e[32m%s ---> SUCCESSFUL <---\e[0m", ros::this_node::getName().c_str());
      green_light = true;
  }
  else
      ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}


void IdmindInteraction::initialize()
{
  if (initialize_)
  {
    ROS_INFO("Defining base LEDS...");
    setNumberBaseLeds();
    send_head_ = true;
    sendHeadCommand(true);
    send_neck_ = true;
    sendNeckCommand(true);
  }
}


void IdmindInteraction::runPeriodically()
{
  ros::Rate r(10.0);
  while(n_.ok())
  {
      ros::spinOnce();
      
      getMotorPosition(true);

      if(send_head_){
          sendHeadCommand();
          ros::Duration(0.005).sleep();
      }
      
      if(send_neck_){
          sendNeckCommand();
          ros::Duration(0.005).sleep();
      }

      if(send_headneck_){
          sendHeadNeckCommand();
          ros::Duration(0.005).sleep();
      }
      if(send_camera_){
          sendCameraTiltCommand();
          ros::Duration(0.005).sleep();
      }
      if(send_leds_){
        sendLedsCommand();
        ros::Duration(0.005).sleep();
      }
      if(send_leds_base_){
        sendLedsBaseCommand();
        ros::Duration(0.005).sleep();
      }
      if(send_leds_mouth_){
        sendLedsMouthCommand();
        ros::Duration(0.005).sleep();
      }
      r.sleep();
  }
}


void IdmindInteraction::shutdown()
{
  ros::Duration(0.005).sleep();
  /*final_pos_ = 90;
  ref_time_ = 3;
  sendHeadCommand();
  sendNeckCommand();
  getMotorPosition();
  ROS_INFO("HEAD: %u, NECK: %u",head_position_,neck_position_);*/
}


void IdmindInteraction::headCallback(const idmind_interaction::MotorControl::ConstPtr& msg)
{
  final_pos_ = (*msg).position;
  ref_time_ = (*msg).time;
  send_head_ = true;
  //ROS_INFO("Receiving head message");
}


void IdmindInteraction::sendHeadCommand(bool init)
{
  if(send_head_){
    
    if(init == true){
      final_pos_ = 0;
      ref_time_ = 4.0;
    }

    if(final_pos_ < -90 || final_pos_>90)
      ROS_ERROR("Failed, head position must be between -90 and 90");
    else
      head_[0] = final_pos_ + 90;

    getMotorPosition();
    D_head = final_pos_-head_position_;
    
    if (abs(D_head) < 5)
      vel_head = 30;
    else
      vel_head = (int)-14.04*180/abs(D_head)*ref_time_+95.95;
    if(vel_head < 30) vel_head=30;
    else if(vel_head > 90) vel_head=90;
    head_[1] = vel_head;
  }

  if (head_[1] != last_head_vel_)
  {
    uint8_t velocity[] = {head_[1] >> 8, head_[1] & 0xFF};
    uint8_t vel_to_write[] = {set_velocity_command_, motor_head_, velocity[0], velocity[1]};
    if (serial_.write(vel_to_write, 4))
    {
      uint8_t buffer[5];
      if (!serial_.read(buffer, 5, true) || buffer[0] != set_velocity_command_)
        ROS_ERROR("%s --> Failed to send head velocity command.", ros::this_node::getName().c_str());
    }
    last_head_vel_ = head_[1];
    ros::Duration(0.005).sleep();
  }

  uint8_t position[] = {head_[0] >> 8, head_[0] & 0xFF};
  uint8_t pos_to_write[] = {set_position_command_, motor_head_, position[0], position[1]};

  if (serial_.write(pos_to_write, 4))
  {
    uint8_t buffer[5];
    if (!serial_.read(buffer, 5, true) || buffer[0] != set_position_command_)
      ROS_ERROR("%s --> Failed to send head position command.", ros::this_node::getName().c_str());
  }
  ros::Duration(0.005).sleep();
  send_head_ = false;
}


void IdmindInteraction::neckCallback(const idmind_interaction::MotorControl::ConstPtr& msg)
{
  final_pos_ = (*msg).position;
  ref_time_ = (*msg).time;
  send_neck_ = true;
  //ROS_INFO("Receiving neck message");
}


void IdmindInteraction::sendNeckCommand(bool init)
{
  if(send_neck_){

    if(init == true){
      final_pos_ = 0;
      ref_time_ = 4.0;
    }

    if(final_pos_ < -90 || final_pos_>90)
      ROS_ERROR("Failed, head position must be between -90 and 90");
    else
      neck_[0] = final_pos_ + 90;

    getMotorPosition();
    D_neck = final_pos_-neck_position_;
    
    if (abs(D_neck) < 5)
      vel_neck = 30;
    else
      vel_neck = (int)-10.86*180/abs(D_neck)*ref_time_+97.05;
    
    if(vel_neck < 30) vel_neck=30;
    else if(vel_neck > 90) vel_neck=90;
    neck_[1] = vel_neck;
  }

  if (neck_[1] != last_neck_vel_)
  {
    uint8_t velocity[] = {neck_[1] >> 8, neck_[1] & 0xFF};
    uint8_t vel_to_write[] = {set_velocity_command_, motor_neck_, velocity[0], velocity[1]};
    if (serial_.write(vel_to_write, 4))
    {
      uint8_t buffer[5];
      if (!serial_.read(buffer, 5, true) || buffer[0] != set_velocity_command_)
        ROS_ERROR("%s --> Failed to send neck velocity command.", ros::this_node::getName().c_str());
    }
    last_neck_vel_ = neck_[1];
    ros::Duration(0.005).sleep();
  }

  uint8_t position[] = {neck_[0] >> 8, neck_[0] & 0xFF};
  uint8_t pos_to_write[] = {set_position_command_, motor_neck_, position[0], position[1]};

  if (serial_.write(pos_to_write, 4))
  {
    uint8_t buffer[5];
    if (!serial_.read(buffer, 5, true) || buffer[0] != set_position_command_)
      ROS_ERROR("%s --> Failed to send neck position command.", ros::this_node::getName().c_str());
  }
  ros::Duration(0.005).sleep();
  send_neck_ = false;
}


void IdmindInteraction::headNeckCallback(const idmind_interaction::MotorControl::ConstPtr& msg)
{
  final_pos_ = (*msg).position + 90;
  ref_time_ = (*msg).time;
  send_headneck_ = true;
  //ROS_INFO("Receiving head and neck message");
}


void IdmindInteraction::sendHeadNeckCommand()
{
  //ROS_INFO("-------------------------------------------------------------");
  //ROS_INFO("-------------------------------------------------------------");
  //ROS_INFO("FINAL POSITION: %u",final_pos_);

  getMotorPosition();
  //ROS_INFO("HEAD: %u, NECK: %u",head_position_,neck_position_);

  D_head = final_pos_-head_position_;
  D_neck = final_pos_-neck_position_;
  //ROS_INFO("D_head: %d, D_neck: %d",D_head,D_neck);

  if (abs(D_head) < 5)
    vel_head = 30;
  else
    vel_head = (int)-14.04*180/abs(D_head)*ref_time_+95.95;
  if(vel_head < 30) vel_head=30;
  else if(vel_head > 90) vel_head=90;
  
  if (abs(D_neck) < 5)
    vel_neck = 30;
  else
  vel_neck = (int)-10.86*180/abs(D_neck)*ref_time_+97.05;
  if(vel_neck < 30) vel_neck=30;
  else if(vel_neck > 90) vel_neck=90;

  //ROS_INFO("Vel original %d %d",vel_head,vel_neck);

  sleep_time_ = ref_time_*(abs(D_head)+abs(D_neck))/(2*180*dist_inc_*12);
  //ROS_INFO("Sleeping Time: %f",sleep_time_);
  //ROS_INFO("-------------------------------------------------------------");


  //CONTROL LOOP
  while(abs(D_head) > 0 || abs(D_neck) > 0)
  {
    //ROS_INFO("Head: %u, Neck: %u",head_position_,neck_position_);
    //ROS_INFO("D_head: %d, D_neck: %d",D_head,D_neck);
    //POSITION CONTROL
    if(abs(D_head) > 0)
    {
      if(abs(D_head) <= dist_inc_)
        head_[0] = final_pos_;
      else
        head_[0] = head_position_+ sign(D_head)*dist_inc_;
    }
    if(abs(D_neck) > 0)
    {
      if(abs(D_neck) <= dist_inc_)
        neck_[0] = final_pos_;
      else
        neck_[0] = neck_position_+ sign(D_neck)*dist_inc_;
    }
    //ROS_INFO("Next Pos %u %u",head_[0],neck_[0]);

    // VELOCITY CONTROL
    if(abs(D_head)- abs(D_neck) >= 5)
    {
      //ROS_INFO("Increase head");
      vel_head += 2;
      vel_neck -= 2;
    }
    else if(abs(D_head)- abs(D_neck) <= -5)
    {
      //ROS_INFO("Increase neck");
      vel_head -= 2;
      vel_neck += 2;
    }

    if(vel_head<30) vel_head=30;
    if(vel_neck<30) vel_neck=30;

    //ROS_INFO("VEL %d %d",vel_head,vel_neck);

    head_[1]=vel_head;
    neck_[1]=vel_neck;
    
    sendHeadCommand();
    sendNeckCommand();
    ros::Duration(sleep_time_).sleep();

    //FEEDBACK    
    getMotorPosition();
    D_head = final_pos_-head_position_;
    D_neck = final_pos_-neck_position_;
    if(abs(D_head) < 2)
      D_head = 0;
    if(abs(D_neck) < 2)
      D_neck = 0;
    //ROS_INFO(".....................................................");
  }

  //ROS_INFO("Finishing...");
  getMotorPosition();
  //ROS_INFO("Head: %u, Neck: %u",head_position_,neck_position_);

  send_headneck_ = false;
}


void IdmindInteraction::cameraTiltCallback(const idmind_interaction::MotorControl::ConstPtr& msg)
{
  final_pos_ = (*msg).position;
  ref_time_ = (*msg).time;
  send_camera_ = true;
}


void IdmindInteraction::sendCameraTiltCommand()
{
  if(send_camera_){
    /*if(final_pos_ < 0 || final_pos_>180)
      ROS_ERROR("Failed, head position must be between 0 and 180");
    else*/
    cam_tilt_[0] = final_pos_;

    getMotorPosition();
    D_cam = final_pos_ - camera_position_;

  }
  cam_tilt_[1] = 80;

  uint8_t velocity[] = {cam_tilt_[1] >> 8, cam_tilt_[1] & 0xFF};
  uint8_t vel_to_write[] = {set_velocity_command_, motor_camera_, velocity[0], velocity[1]};
  if (serial_.write(vel_to_write, 4))
  {
    uint8_t buffer[5];
    if (!serial_.read(buffer, 5, true) || buffer[0] != set_velocity_command_)
      ROS_ERROR("%s --> Failed to send camera tilt velocity command.", ros::this_node::getName().c_str());
  }
  last_cam_vel_ = cam_tilt_[1];
  ros::Duration(0.005).sleep();

  uint8_t position[] = {cam_tilt_[0] >> 8, cam_tilt_[0] & 0xFF};
  uint8_t pos_to_write[] = {set_position_command_, motor_camera_, position[0], position[1]};

  if (serial_.write(pos_to_write, 4))
  {
    uint8_t buffer[5];
    if (!serial_.read(buffer, 5, true) || buffer[0] != set_position_command_)
      ROS_ERROR("%s --> Failed to send camera tilt position command.", ros::this_node::getName().c_str());
  }
  ros::Duration(0.005).sleep();
  send_camera_ = false;
}


void IdmindInteraction::ledsCallback(const idmind_interaction::LedControl::ConstPtr& msg)
{
  led_device_ = (*msg).device;
  led_rgb_[0] = (*msg).r;
  led_rgb_[1] = (*msg).g;
  led_rgb_[2] = (*msg).b;

  send_leds_ = true;
}


void IdmindInteraction::sendLedsCommand()
{
  if (led_device_ < 0 || led_device_ > 1){
    ROS_ERROR("LED Device must be between 0 and 1");
    return;
  }

  uint8_t command;

  if(led_device_ == 0)
  {
    number_of_leds_ = 4;
    command = set_mouth_leds_command_;
  }
  else if(led_device_ == 1){
    number_of_leds_ = BASE_LEDS;
    command = set_base_leds_command_;
  }

  int size = 1+number_of_leds_*3;
  uint8_t led_control[size];
  led_control[0] = command;

  for(int i = 1; i < number_of_leds_*3; i+=3){
    led_control[i] = led_rgb_[0];
    led_control[i+1] = led_rgb_[1];
    led_control[i+2] = led_rgb_[2];
  }

  if (serial_.write(led_control, size))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != command)
      ROS_ERROR("%s --> Failed to send led control command.", ros::this_node::getName().c_str());
  }
  //ROS_INFO("Successful write on LEDs");
  send_leds_ = false;  
}


void IdmindInteraction::ledsBaseCallback(const idmind_interaction::LedControlBase::ConstPtr& msg)
{
  number_of_leds_ = BASE_LEDS;
  for (int i = 0; i < number_of_leds_*3; ++i)
    array_leds_base_[i] = (*msg).led_array[i];
  send_leds_base_ = true;
}


void IdmindInteraction::sendLedsBaseCommand()
{
  uint8_t command = set_base_leds_command_;
  
  int size = 1+BASE_LEDS*3;
  uint8_t led_control[size];
  led_control[0] = command;

  for(int i = 0; i < BASE_LEDS*3; i+=3){
    led_control[i+1] = array_leds_base_[i];
    led_control[i+2] = array_leds_base_[i+1];
    led_control[i+3] = array_leds_base_[i+2];
  }

  if (serial_.write(led_control, size))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != command)
      ROS_ERROR("%s --> Failed to send led control command.", ros::this_node::getName().c_str());
  }
  //ROS_INFO("Successful write on LEDs");
  send_leds_base_ = false;
}



void IdmindInteraction::ledsMouthCallback(const idmind_interaction::LedControlMouth::ConstPtr& msg)
{
  number_of_leds_ = 4;
  for (int i = 0; i < number_of_leds_*3; ++i)
    array_leds_mouth_[i] = (*msg).led_array[i];
  send_leds_mouth_ = true;
}


void IdmindInteraction::sendLedsMouthCommand()
{
  uint8_t command = set_mouth_leds_command_;
  
  int size = 1+MOUTH_LEDS*3;
  uint8_t led_control[size];
  led_control[0] = command;

  for(int i = 0; i < MOUTH_LEDS*3; i+=3){
    led_control[i+1] = array_leds_mouth_[i];
    led_control[i+2] = array_leds_mouth_[i+1];
    led_control[i+3] = array_leds_mouth_[i+2];
  }

  if (serial_.write(led_control, size))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != command)
      ROS_ERROR("%s --> Failed to send led control command.", ros::this_node::getName().c_str());
  }
  //ROS_INFO("Successful write on LEDs");
  send_leds_mouth_ = false;
}


bool IdmindInteraction::setNumberBaseLeds()
{
  uint8_t command[] = {set_num_base_leds_command_,BASE_LEDS};

    if (serial_.write(command, 2))
    {
      uint8_t buffer[4];
      if (serial_.read(buffer, 4, true) && buffer[0] == set_num_base_leds_command_)
      {
        ROS_INFO("\e[32m%s ---> Successfully set number of base leds <---\e[0m", ros::this_node::getName().c_str());
        return true;
      }
      else
        ROS_ERROR("%s --> Failed to set number of base leds.", ros::this_node::getName().c_str());
    }
    return false;
}


void IdmindInteraction::getMotorPosition(bool publish_topics)
{
  uint8_t head_write[] = {get_position_command_, motor_head_};
  if (serial_.write(head_write, 2))
  {
    uint8_t buffer[7];
    if (!serial_.read(buffer, 7, true) || buffer[0] != get_position_command_)
      ROS_ERROR("%s --> Failed to get head position.", ros::this_node::getName().c_str());
    else if (buffer[1] == 0)
      head_position_ = (int)buffer[2] * 0x0100 + (int)buffer[3];
  }
  ros::Duration(0.005).sleep();

  uint8_t neck_write[] = {get_position_command_, motor_neck_};
  if (serial_.write(neck_write, 2))
  {
    uint8_t buffer[7];
    if (!serial_.read(buffer, 7, true) || buffer[0] != get_position_command_)
      ROS_ERROR("%s --> Failed to get head position.", ros::this_node::getName().c_str());
    else if (buffer[1] == 1)
      neck_position_ = (int)buffer[2] * 0x0100 + (int)buffer[3];
  }
  ros::Duration(0.005).sleep();
  
  uint8_t camera_write[] = {get_position_command_, motor_camera_};
  if (serial_.write(camera_write, 2))
  {
    uint8_t buffer[7];
    if (!serial_.read(buffer, 7, true) || buffer[0] != get_position_command_)
      ROS_ERROR("%s --> Failed to get head position.", ros::this_node::getName().c_str());
    else if (buffer[1] == 1)
      camera_position_ = (int)buffer[2] * 0x0100 + (int)buffer[3];
  }

  //ROS_INFO("REAL POSITION %d %d %d",head_position_,neck_position_,camera_position_);
  if(head_position_ > 250) head_position_ = 0;
  if(neck_position_ > 250) neck_position_ = 0;

  if(publish_topics == true){
    std_msgs::Int64 h;
    std_msgs::Int64 n;
    std_msgs::Int64 c;
    n.data = head_position_ - 90;
    h.data = neck_position_ - 90;
    c.data = camera_position_ - 90;
    head_position_pub_.publish(n);
    neck_position_pub_.publish(h);
    camera_position_pub_.publish(c);
  }
}


int IdmindInteraction::sign(int num)
{
    if(num > 0) return 1;
    if(num < 0) return -1;
    return 0;
}


int main(int argc, char** argv){

  ros::init(argc, argv, "idmind_interaction");

  IdmindInteraction interaction;

  interaction.initialize();

  if (interaction.green_light)
      interaction.runPeriodically();

  interaction.shutdown();

  return 0;
}