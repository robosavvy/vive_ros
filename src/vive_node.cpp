#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "vive_ros/vr_interface.h"

#include <geometry_msgs/TwistStamped.h>

void handleDebugMessages(const std::string &msg) {ROS_DEBUG(" [VIVE] %s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {ROS_INFO(" [VIVE] %s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {ROS_ERROR(" [VIVE] %s",msg.c_str());}

class VIVEnode
{
  public:
    VIVEnode(int rate);
    ~VIVEnode();
    bool Init();
    void Run();
    void Shutdown();
    bool setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    std::vector<double> world_offset_;
    double world_yaw_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    ros::ServiceServer set_origin_server_;
    ros::Publisher twist1_pub_;
    ros::Publisher twist2_pub_;

    VRInterface vr_;

};

VIVEnode::VIVEnode(int rate)
  : loop_rate_(rate)
  , nh_()
  , tf_broadcaster_()
  , tf_listener_()
  , vr_()
  , world_offset_({0, 0, 0})
  , world_yaw_(0)
{

  nh_.getParam("/vive/world_offset", world_offset_);
  nh_.getParam("/vive/world_yaw", world_yaw_);
  ROS_INFO(" [VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);

  set_origin_server_ = nh_.advertiseService("/vive/set_origin", &VIVEnode::setOriginCB, this);

  //~ twist1_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist1", 10);
  //~ twist2_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist2", 10);

  return;
}

VIVEnode::~VIVEnode()
{
  return;
}

bool VIVEnode::Init()
{
  //  Set logging functions
  vr_.setDebugMsgCallback(handleDebugMessages);
  vr_.setInfoMsgCallback(handleInfoMessages);
  vr_.setErrorMsgCallback(handleErrorMessages);

  if (!vr_.Init())
  {
    return false;
  }

  return true;
}

void VIVEnode::Shutdown()
{
  vr_.Shutdown();
}

bool VIVEnode::setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  double tf_matrix[3][4];
  int index = 1, dev_type;
  while (dev_type != 2) 
  {
    dev_type = vr_.GetDeviceMatrix(index++, tf_matrix);
  }
  if (dev_type == 0) 
  {
    ROS_WARN(" [VIVE] Coulnd't find controller 1.");
    return false;
  }

  tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                           tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                           tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
  tf::Vector3 c_z;
  c_z = rot_matrix*tf::Vector3(0,0,1);
  c_z[1] = 0;
  c_z.normalize();
  double new_yaw = acos(tf::Vector3(0,0,1).dot(c_z)) + M_PI_2;
  if (c_z[0] < 0) new_yaw = -new_yaw;
  world_yaw_ = -new_yaw;

  tf::Vector3 new_offset;
  tf::Matrix3x3 new_rot;
  new_rot.setRPY(0, 0, world_yaw_);
  new_offset = new_rot*tf::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

  world_offset_[0] = new_offset[0];
  world_offset_[1] = new_offset[1];
  world_offset_[2] = new_offset[2];

  nh_.setParam("/vive/world_offset", world_offset_);
  nh_.setParam("/vive/world_yaw", world_yaw_);
  ROS_INFO(" [VIVE] New world offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);

  return true;
}

void VIVEnode::Run()
{
  double tf_matrix[3][4];

  while (ros::ok())
  {
    // do stuff
    vr_.Update();

    int controller_count = 1;
    int lighthouse_count = 1;
    for (int i=0; i<5; i++)
    {
      int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);

      // No device
      if (dev_type == 0) continue;

      tf::Transform tf;
      tf.setOrigin(tf::Vector3(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]));

      tf::Quaternion quat;
      tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                               tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                               tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

      rot_matrix.getRotation(quat);
      tf.setRotation(quat);

      // It's a HMD
      if (dev_type == 1)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "hmd"));
      }
      // It's a controller
      if (dev_type == 2)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "controller"+std::to_string(controller_count++)));
      }
      // It's a lighthouse
      if (dev_type == 4)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "lighthouse"+std::to_string(lighthouse_count++)));
      }

    }

    // Publish corrective transform
    tf::Transform tf_world;
    tf_world.setOrigin(tf::Vector3(world_offset_[0], world_offset_[1], world_offset_[2]));
    tf::Quaternion quat_world;
    quat_world.setRPY(M_PI/2, 0, world_yaw_);
    tf_world.setRotation(quat_world);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_world, ros::Time::now(), "world", "world_vive"));

    // Publish twist messages for controller1 and controller2
    //~ double lin_vel[3], ang_vel[3];
    //~ if (vr_.GetDeviceVel(1, lin_vel, ang_vel))
    //~ {
      //~ geometry_msgs::Twist twist_msg;
      //~ twist_msg.linear.x = lin_vel[0];
      //~ twist_msg.linear.y = lin_vel[1];
      //~ twist_msg.linear.z = lin_vel[2];
      //~ twist_msg.angular.x = ang_vel[0];
      //~ twist_msg.angular.y = ang_vel[1];
      //~ twist_msg.angular.z = ang_vel[2];

      //~ geometry_msgs::TwistStamped twist_msg_stamped;
      //~ twist_msg_stamped.header.stamp = ros::Time::now();
      //~ twist_msg_stamped.header.frame_id = "world_vive";
      //~ twist_msg_stamped.twist = twist_msg;

      //~ twist1_pub_.publish(twist_msg_stamped);
    //~ }
    //~ if (vr_.GetDeviceVel(2, lin_vel, ang_vel))
    //~ {
      //~ geometry_msgs::Twist twist_msg;
      //~ twist_msg.linear.x = lin_vel[0];
      //~ twist_msg.linear.y = lin_vel[1];
      //~ twist_msg.linear.z = lin_vel[2];
      //~ twist_msg.angular.x = ang_vel[0];
      //~ twist_msg.angular.y = ang_vel[1];
      //~ twist_msg.angular.z = ang_vel[2];

      //~ geometry_msgs::TwistStamped twist_msg_stamped;
      //~ twist_msg_stamped.header.stamp = ros::Time::now();
      //~ twist_msg_stamped.header.frame_id = "world_vive";
      //~ twist_msg_stamped.twist = twist_msg;

      //~ twist2_pub_.publish(twist_msg_stamped);
    //~ }

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// Main
int main(int argc, char** argv){
  ros::init(argc, argv, "vive_node");

  VIVEnode nodeApp(20);

  if (!nodeApp.Init())
  {
    nodeApp.Shutdown();
    return 1;
  }

  nodeApp.Run();

  nodeApp.Shutdown();

  return 0;
};
