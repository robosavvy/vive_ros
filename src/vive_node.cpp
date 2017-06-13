#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "vive_ros/vr_interface.h"

#include <geometry_msgs/TwistStamped.h>

// for image
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
//#define L 0
//#define R 1
//#define LR 2
enum {X, Y, XY};
enum {L, R, LR};

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
    void processROSStereoImage(cv::Mat (&in)[LR], cv::Mat (&out)[LR]){
      //calc eye to HMD panel distance
    //    const double hmd_eye2panel_z[XY] = { (double)hmd_panel_img[i].cols/2/tan(hmd_fov/2), (double)hmd_panel_img[i].rows/2/tan(hmd_fov/2) };
      const double hmd_eye2panel_z[XY] = { (double)out[L].rows/2/tan(hmd_fov/2), (double)out[L].rows/2/tan(hmd_fov/2) };//[pixel]パネル距離水平垂直で違うのはおかしいので垂直画角を信じる
      const double cam_pic_size[LR][XY] = { { (double)in[L].cols, (double)in[L].rows }, { (double)in[R].cols, (double)in[R].rows } };
      double cam_fov[LR][XY];
      int cam_pic_size_on_hmd[LR][XY];
      cv::Mat hmd_panel_roi[LR];
      const cv::Point parallax_adjust[LR] = {cv::Point(-50,0),cv::Point(+50,0)};//視差調整用
      for(int i=L;i<LR;i++){
        if(ros_image_isNew[i]){
            for(int j=X;j<XY;j++){
              cam_fov[i][j] = 2*atan( cam_pic_size[i][j]/2 / cam_f[i][j] );
              cam_pic_size_on_hmd[i][j] = (int)( hmd_eye2panel_z[X] * 2*tan(cam_fov[i][j]/2) );
            }
            cv::resize(in[i], ros_image_stereo_resized[i], cv::Size(cam_pic_size_on_hmd[i][X],cam_pic_size_on_hmd[i][Y]));
            cv::putText(ros_image_stereo_resized[i], txt_ros, cv::Point(0,500), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,250,250), 2, CV_AA);
            cv::flip(ros_image_stereo_resized[i],ros_image_stereo_resized[i],0);
            float scale = 1.0;
            // 画像の中心を求める
            cv::Point2f center(ros_image_stereo_resized[i].cols / 2.0, ros_image_stereo_resized[i].rows / 2.0 - 500);
            cv::Rect hmd_panel_area_rect( ros_image_stereo_resized[i].cols/2-out[i].cols/2, ros_image_stereo_resized[i].rows/2-out[i].rows/2, out[i].cols, out[i].rows);
            hmd_panel_area_rect += parallax_adjust[i];
            cv::Rect ros_image_stereo_resized_rect( 0, 0, ros_image_stereo_resized[i].cols, ros_image_stereo_resized[i].rows);
            cv::Point ros_image_stereo_resized_center(ros_image_stereo_resized[i].cols/2, ros_image_stereo_resized[i].rows/2);
            cv::Rect cropped_rect;
            if( !hmd_panel_area_rect.contains( cv::Point(ros_image_stereo_resized_rect.x, ros_image_stereo_resized_rect.y) )
                || !hmd_panel_area_rect.contains( cv::Point(ros_image_stereo_resized_rect.x+ros_image_stereo_resized_rect.width,ros_image_stereo_resized_rect.y+ros_image_stereo_resized_rect.height) ) ){
              ROS_WARN_THROTTLE(3.0,"Resized ROS image[%d] (%dx%d (%+d,%+d)) exceed HMD eye texture (%dx%d) -> Cropping",i,cam_pic_size_on_hmd[i][X],cam_pic_size_on_hmd[i][Y],parallax_adjust[i].x,parallax_adjust[i].y,hmd_panel_size[X],hmd_panel_size[Y]);
              cropped_rect = ros_image_stereo_resized_rect & hmd_panel_area_rect;
              ros_image_stereo_resized[i] = ros_image_stereo_resized[i](cropped_rect);
            }
            cv::Rect hmd_panel_draw_rect( cropped_rect.x-hmd_panel_area_rect.x, cropped_rect.y-hmd_panel_area_rect.y, ros_image_stereo_resized[i].cols, ros_image_stereo_resized[i].rows);
            ros_image_stereo_resized[i].copyTo(out[i](hmd_panel_draw_rect));
        }
      }
    }
    bool CMainApplication::UpdateTexturemaps(){
      if(view_mode==IMAGE_VIEW_MODE::STEREO){
        processROSStereoImage(ros_image_stereo, hmd_panel_img);
        for(int i=L;i<LR;i++){
          if(ros_image_isNew[i]){
            int cur_tex_w,cur_tex_h;
            glBindTexture( GL_TEXTURE_2D, m_EyeTexture[i] );
            glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_WIDTH , &cur_tex_w );
            glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_HEIGHT , &cur_tex_h );
            glTexSubImage2D( GL_TEXTURE_2D, 0, cur_tex_w/2 - hmd_panel_img[i].cols/2, cur_tex_h/2 - hmd_panel_img[i].rows/2, hmd_panel_img[i].cols, hmd_panel_img[i].rows,GL_RGB, GL_UNSIGNED_BYTE, hmd_panel_img[i].data );
            glGenerateMipmap(GL_TEXTURE_2D);
            glBindTexture( GL_TEXTURE_2D, 0 );
          }
        }
        return ( (m_EyeTexture[L]!=0) && (m_EyeTexture[R]!=0) );
      }else{
        glBindTexture( GL_TEXTURE_2D, m_iTexture );
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, ros_image_monoeye.cols, ros_image_monoeye.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, ros_image_monoeye.data );
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
        GLfloat fLargest;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);
        glBindTexture( GL_TEXTURE_2D, 0 );
        return ( m_iTexture != 0 );
      }
    }
    void convertImage(const sensor_msgs::ImageConstPtr& msg, cv::Mat& out){
      try {
        out = cv_bridge::toCvCopy(msg,"rgb8")->image;
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
      }
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      convertImage(msg, pMainApplication->ros_image_monoeye);
      t_cb_now = ros::WallTime::now();
      ROS_INFO_STREAM_THROTTLE(3.0,"imageCb() subscribed "<<msg->width<<" x "<<msg->height<<" enc: "<<msg->encoding<<" @ "<<1.0/(t_cb_now - t_cb_old).toSec()<<" fps");
      t_cb_old = t_cb_now;
      ros_image_isNew_mono = true;
    }
    void imageCb_L(const sensor_msgs::ImageConstPtr& msg){
      convertImage(msg, pMainApplication->ros_image_stereo[L]);
      t_cb_l_now = ros::WallTime::now();
      ROS_INFO_STREAM_THROTTLE(3.0,"imageCb_L() subscribed "<<msg->width<<" x "<<msg->height<<" enc: "<<msg->encoding<<" @ "<<1.0/(t_cb_l_now - t_cb_l_old).toSec()<<" fps");
      t_cb_l_old = t_cb_l_now;
      ros_image_isNew[L] = true;
    }
    void imageCb_R(const sensor_msgs::ImageConstPtr& msg){
      convertImage(msg, pMainApplication->ros_image_stereo[R]);
      t_cb_r_now = ros::WallTime::now();
      ROS_INFO_STREAM_THROTTLE(3.0,"imageCb_R() subscribed "<<msg->width<<" x "<<msg->height<<" enc: "<<msg->encoding<<" @ "<<1.0/(t_cb_r_now - t_cb_r_old).toSec()<<" fps");
      t_cb_r_old = t_cb_r_now;
      ros_image_isNew[R] = true;
    }
    void infoCb_L(const sensor_msgs::CameraInfoConstPtr& msg){ cam_f[L][0] = msg->K[0]; cam_f[L][1] = msg->K[4];}
    void infoCb_R(const sensor_msgs::CameraInfoConstPtr& msg){ cam_f[R][0] = msg->K[0]; cam_f[R][1] = msg->K[4];}

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
    // for image
//    enum {L, R, LR};
	GLuint m_EyeTexture[LR];

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
