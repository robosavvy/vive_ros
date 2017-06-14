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


#include "vive_ros/hellovr_opengl_main.cpp"
//#include <SDL.h>
//#include <SDL_opengl.h>
#include <GL/glew.h>
#include <GL/glu.h>
enum {X, Y, XY};
enum {L, R, LR};

void handleDebugMessages(const std::string &msg) {ROS_DEBUG(" [VIVE] %s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {ROS_INFO(" [VIVE] %s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {ROS_ERROR(" [VIVE] %s",msg.c_str());}

class CMainApplicationMod : public CMainApplication
{
  public:
    CMainApplicationMod( int argc, char *argv[] )
    : CMainApplication( argc, argv )
    , hmd_fov(110*M_PI/180)
    {
      for(int i=0;i<LR;i++){
        cam_f[i][X] = cam_f[i][Y] = 600;
      }
    };
    ~CMainApplicationMod(){};

    cv::Mat ros_img[LR];
    double cam_f[LR][XY];
    const double hmd_fov;//field of view

    void InitTextures(){
      ros_img[L] = cv::Mat(cv::Size(640, 480), CV_8UC3, CV_RGB(255,0,0));
      ros_img[R] = cv::Mat(cv::Size(640, 480), CV_8UC3, CV_RGB(0,255,0));
      hmd_panel_img[L] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      hmd_panel_img[R] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
    }
    void RenderFrame(){
      if ( m_pHMD ){
        DrawControllers();
        UpdateTexturemaps();
        RenderDistortion();
        vr::Texture_t leftEyeTexture = {(void*)leftEyeDesc.m_nResolveTextureId, vr::API_OpenGL, vr::ColorSpace_Gamma };
        vr::Texture_t rightEyeTexture = {(void*)rightEyeDesc.m_nResolveTextureId, vr::API_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
      }
      if ( m_bVblank && m_bGlFinishHack ){ glFinish(); }
      SDL_GL_SwapWindow( m_pWindow );
      glClearColor( 0, 0, 0, 1 );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      if ( m_bVblank ){
        glFlush();
        glFinish();
      }
      if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last ){
        m_iValidPoseCount_Last = m_iValidPoseCount;
        m_iTrackedControllerCount_Last = m_iTrackedControllerCount;
        dprintf( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
      }
      UpdateHMDMatrixPose();
    }

  private:
    cv::Mat hmd_panel_img[LR];
    cv::Mat ros_img_resized[LR];
    void processROSStereoImage(cv::Mat (&in)[LR], cv::Mat (&out)[LR]){
      const double hmd_eye2panel_z[XY] = { (double)out[L].rows/2/tan(hmd_fov/2), (double)out[L].rows/2/tan(hmd_fov/2) };
      const double cam_pic_size[LR][XY] = { { (double)in[L].cols, (double)in[L].rows }, { (double)in[R].cols, (double)in[R].rows } };
      double cam_fov[LR][XY];
      int cam_pic_size_on_hmd[LR][XY];
      cv::Mat hmd_panel_roi[LR];
      for(int i=0;i<LR;i++){
        ROS_INFO_THROTTLE(3.0,"Process ROS image[%d] (%dx%d) with fov (%dx%d) to (%dx%d)", i, in[i].cols, in[i].rows, (int)cam_f[i][X], (int)cam_f[i][Y], out[i].cols, out[i].rows);
        for(int j=0;j<XY;j++){
          cam_fov[i][j] = 2 * atan( cam_pic_size[i][j]/2 / cam_f[i][j] );
          cam_pic_size_on_hmd[i][j] = (int)( hmd_eye2panel_z[X] * 2 * tan(cam_fov[i][j]/2) );
        }
        cv::resize(in[i], ros_img_resized[i], cv::Size(cam_pic_size_on_hmd[i][X], cam_pic_size_on_hmd[i][Y]));
        cv::flip(ros_img_resized[i], ros_img_resized[i], 0);
        cv::Rect hmd_panel_area_rect( ros_img_resized[i].cols/2-out[i].cols/2, ros_img_resized[i].rows/2-out[i].rows/2, out[i].cols, out[i].rows);
        cv::Rect ros_img_resized_rect( 0, 0, ros_img_resized[i].cols, ros_img_resized[i].rows);
        cv::Point ros_img_resized_center(ros_img_resized[i].cols/2, ros_img_resized[i].rows/2);
        cv::Rect cropped_rect;
        if( !hmd_panel_area_rect.contains( cv::Point(ros_img_resized_rect.x, ros_img_resized_rect.y) )
            || !hmd_panel_area_rect.contains( cv::Point(ros_img_resized_rect.x+ros_img_resized_rect.width,ros_img_resized_rect.y+ros_img_resized_rect.height) ) ){
          ROS_WARN_THROTTLE(3.0,"Resized ROS image[%d] (%dx%d) exceed HMD eye texture (%dx%d) -> Cropping",i,cam_pic_size_on_hmd[i][X],cam_pic_size_on_hmd[i][Y],m_nRenderWidth,m_nRenderHeight);
          cropped_rect = ros_img_resized_rect & hmd_panel_area_rect;
          ros_img_resized[i] = ros_img_resized[i](cropped_rect);
        }
        cv::Rect hmd_panel_draw_rect( cropped_rect.x-hmd_panel_area_rect.x, cropped_rect.y-hmd_panel_area_rect.y, ros_img_resized[i].cols, ros_img_resized[i].rows);
        ros_img_resized[i].copyTo(out[i](hmd_panel_draw_rect));
      }
    }

    void UpdateTexturemaps(){
      processROSStereoImage(ros_img, hmd_panel_img);
      for(int i=0;i<LR;i++){
        if(i==L)glBindTexture( GL_TEXTURE_2D, leftEyeDesc.m_nResolveTextureId );
        else if(i==R)glBindTexture( GL_TEXTURE_2D, rightEyeDesc.m_nResolveTextureId );
        else break;
        int cur_tex_w,cur_tex_h;
        glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_WIDTH , &cur_tex_w );
        glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_HEIGHT , &cur_tex_h );
        glTexSubImage2D( GL_TEXTURE_2D, 0, cur_tex_w/2 - hmd_panel_img[i].cols/2, cur_tex_h/2 - hmd_panel_img[i].rows/2, hmd_panel_img[i].cols, hmd_panel_img[i].rows, GL_RGB, GL_UNSIGNED_BYTE, hmd_panel_img[i].data );
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, 0 );
      }
    }
};


class VIVEnode
{
  public:
    VIVEnode(int rate);
    ~VIVEnode();
    bool Init();
    void Run();
    void Shutdown();
    bool setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    CMainApplicationMod *pMainApplication;

    ros::NodeHandle nh_;
  private:
//    ros::NodeHandle nh_;
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

    pMainApplication->HandleInput();
    pMainApplication->RenderFrame();


    ros::spinOnce();
    loop_rate_.sleep();
  }
}

VIVEnode* nodeApp_p;

void imageCb_L(const sensor_msgs::ImageConstPtr& msg){
  if(msg->width > 0 && msg->height > 0 ){
    try {
      nodeApp_p->pMainApplication->ros_img[L] = cv_bridge::toCvCopy(msg,"rgb8")->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(1, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
  }else{
    ROS_WARN_THROTTLE(3, "Invalid image_left size (%dx%d) use default", msg->width, msg->height);
  }
}
void imageCb_R(const sensor_msgs::ImageConstPtr& msg){
  if(msg->width > 0 && msg->height > 0 ){
    try {
      nodeApp_p->pMainApplication->ros_img[R] = cv_bridge::toCvCopy(msg,"rgb8")->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(1, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
  }else{
    ROS_WARN_THROTTLE(3, "Invalid image_right size (%dx%d) use default", msg->width, msg->height);
  }
}
void infoCb_L(const sensor_msgs::CameraInfoConstPtr& msg){
  if(msg->K[0] > 0.0 && msg->K[4] > 0.0 ){
    nodeApp_p->pMainApplication->cam_f[L][0] = msg->K[0];
    nodeApp_p->pMainApplication->cam_f[L][1] = msg->K[4];
  }else{
    ROS_WARN_THROTTLE(3, "Invalid camera_info_left fov (%fx%f) use default", msg->K[0], msg->K[4]);
  }
}
void infoCb_R(const sensor_msgs::CameraInfoConstPtr& msg){
  if(msg->K[0] > 0.0 && msg->K[4] > 0.0 ){
    nodeApp_p->pMainApplication->cam_f[R][0] = msg->K[0];
    nodeApp_p->pMainApplication->cam_f[R][1] = msg->K[4];
  }else{
    ROS_WARN_THROTTLE(3, "Invalid camera_info_right fov (%fx%f) use default", msg->K[0], msg->K[4]);
  }
}


// Main
int main(int argc, char** argv){
  ros::init(argc, argv, "vive_node");

  VIVEnode nodeApp(60);

  if (!nodeApp.Init())
  {
    nodeApp.Shutdown();
    return 1;
  }

  nodeApp_p = &nodeApp;

  image_transport::ImageTransport it(nodeApp.nh_);
  image_transport::Subscriber sub_L,sub_R;
  ros::Subscriber sub_i_L,sub_i_R;
  sub_L = it.subscribe("/image_left", 1, imageCb_L);
  sub_R = it.subscribe("/image_right", 1, imageCb_R);
  sub_i_L = nodeApp.nh_.subscribe("/camera_info_left", 1, infoCb_L);
  sub_i_R = nodeApp.nh_.subscribe("/camera_info_right", 1, infoCb_R);

  nodeApp.pMainApplication = new CMainApplicationMod( argc, argv );

  if (!nodeApp.pMainApplication->BInit())
  {
    nodeApp.pMainApplication->Shutdown();
    return 1;
  }

  nodeApp.pMainApplication->InitTextures();

  nodeApp.Run();

  nodeApp.Shutdown();

  return 0;
};
