#include <cmath>
#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include "vive_ros/vr_interface.h"
#include <geometry_msgs/TwistStamped.h>

using namespace std;

void handleDebugMessages(const std::string &msg) {ROS_DEBUG(" [VIVE] %s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {ROS_INFO(" [VIVE] %s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {ROS_ERROR(" [VIVE] %s",msg.c_str());}
void mySigintHandler(int sig){
// Do some custom action.
// For example, publish a stop message to some other nodes.
// All the default sigint handler does is call shutdown()
ros::shutdown();
}

//#define USE_IMAGE

#define USE_OPENGL
//#define USE_VULKAN

#ifdef USE_IMAGE
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
enum {X, Y, XY};
enum {L, R, LR};

#if defined USE_OPENGL
#include "vive_ros/hellovr_opengl_main.h"
class CMainApplicationMod : public CMainApplication{
  public:
    CMainApplicationMod( int argc, char *argv[] )
    : CMainApplication( argc, argv )
    , hmd_fov(110*M_PI/180) {
//      m_bShowCubes = false;
      for(int i=0;i<LR;i++){
        cam_f[i][X] = cam_f[i][Y] = 600;
      }
      RenderFrame_hz_count = 0;
    };
    ~CMainApplicationMod(){};
    VRInterface* vr_p;

    cv::Mat ros_img[LR];
    double cam_f[LR][XY];
    const double hmd_fov;//field of view
    float hmd_fov_h, hmd_fov_v;
    int RenderFrame_hz_count;

    void InitTextures(){
      ros_img[L] = cv::Mat(cv::Size(640, 480), CV_8UC3, CV_RGB(255,0,0));
      ros_img[R] = cv::Mat(cv::Size(640, 480), CV_8UC3, CV_RGB(0,255,0));
      hmd_panel_img[L] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      hmd_panel_img[R] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      for ( int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++){
        if(m_pHMD->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD){
          m_pHMD->GetStringTrackedDeviceProperty( i, vr::Prop_ScreenshotHorizontalFieldOfViewDegrees_Float, (char *)&hmd_fov_h, sizeof(float), NULL );
          m_pHMD->GetStringTrackedDeviceProperty( i, vr::Prop_ScreenshotVerticalFieldOfViewDegrees_Float, (char *)&hmd_fov_v, sizeof(float), NULL );
        }
      }
    }
    void RenderFrame(){
      ros::Time tmp = ros::Time::now();
      if ( m_pHMD ){
        RenderControllerAxes();
        RenderStereoTargets();
        UpdateTexturemaps();
        RenderCompanionWindow();
        vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
      }
      if ( m_bVblank && m_bGlFinishHack ){ glFinish(); }
      SDL_GL_SwapWindow( m_pCompanionWindow );
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
      ROS_INFO_THROTTLE(3.0,"RenderFrame() @ %d [fps]", [](int& cin, int dur){int ans = cin; cin=0; return ans/dur;}(RenderFrame_hz_count, 3));
      RenderFrame_hz_count++;
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
//        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, 0 );
      }
    }
};

#elif defined USE_VULKAN
#include "vive_ros/hellovr_vulkan_main.h"

class CMainApplicationMod : public CMainApplication
{
  public:
    CMainApplicationMod( int argc, char *argv[] )
    : CMainApplication( argc, argv )
    , hmd_fov(110*M_PI/180) {
//      m_bShowCubes = false;
      for(int i=0;i<LR;i++){
        cam_f[i][X] = cam_f[i][Y] = 600;
      }
      RenderFrame_hz_count = 0;
    };
    ~CMainApplicationMod(){};
    VRInterface* vr_p;

    cv::Mat ros_img[LR];
    double cam_f[LR][XY];
    const double hmd_fov;//field of view
    float hmd_fov_h, hmd_fov_v;
    int RenderFrame_hz_count;

    void InitTextures(){
      ros_img[L] = cv::Mat(cv::Size(640, 480), CV_8UC3, CV_RGB(255,0,0));
      ros_img[R] = cv::Mat(cv::Size(640, 480), CV_8UC3, CV_RGB(0,255,0));
      hmd_panel_img[L] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      hmd_panel_img[R] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      for ( int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++){
        if(m_pHMD->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD){
          m_pHMD->GetStringTrackedDeviceProperty( i, vr::Prop_ScreenshotHorizontalFieldOfViewDegrees_Float, (char *)&hmd_fov_h, sizeof(float), NULL );
          m_pHMD->GetStringTrackedDeviceProperty( i, vr::Prop_ScreenshotVerticalFieldOfViewDegrees_Float, (char *)&hmd_fov_v, sizeof(float), NULL );
        }
      }
    }
    void RenderFrame() {
      if ( m_pHMD ) {
        m_currentCommandBuffer = GetCommandBuffer();
        // Start the command buffer
        VkCommandBufferBeginInfo commandBufferBeginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer( m_currentCommandBuffer.m_pCommandBuffer, &commandBufferBeginInfo );
        UpdateControllerAxes();
        RenderStereoTargets();
        UpdateTexturemaps();
        RenderCompanionWindow();
        // End the command buffer
        vkEndCommandBuffer( m_currentCommandBuffer.m_pCommandBuffer );
        // Submit the command buffer
        VkPipelineStageFlags nWaitDstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo submitInfo = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &m_currentCommandBuffer.m_pCommandBuffer;
        submitInfo.waitSemaphoreCount = 1;
        submitInfo.pWaitSemaphores = &m_pSwapchainSemaphores[ m_nFrameIndex ];
        submitInfo.pWaitDstStageMask = &nWaitDstStageMask;
        vkQueueSubmit( m_pQueue, 1, &submitInfo, m_currentCommandBuffer.m_pFence );
        // Add the command buffer back for later recycling
        m_commandBuffers.push_front( m_currentCommandBuffer );
        // Submit to SteamVR
        vr::VRTextureBounds_t bounds;
        bounds.uMin = 0.0f;
        bounds.uMax = 1.0f;
        bounds.vMin = 0.0f;
        bounds.vMax = 1.0f;
        vr::VRVulkanTextureData_t vulkanData;
        vulkanData.m_nImage = ( uint64_t ) m_leftEyeDesc.m_pImage;
        vulkanData.m_pDevice = ( VkDevice_T * ) m_pDevice;
        vulkanData.m_pPhysicalDevice = ( VkPhysicalDevice_T * ) m_pPhysicalDevice;
        vulkanData.m_pInstance = ( VkInstance_T *) m_pInstance;
        vulkanData.m_pQueue = ( VkQueue_T * ) m_pQueue;
        vulkanData.m_nQueueFamilyIndex = m_nQueueFamilyIndex;
        vulkanData.m_nWidth = m_nRenderWidth;
        vulkanData.m_nHeight = m_nRenderHeight;
        vulkanData.m_nFormat = VK_FORMAT_R8G8B8A8_SRGB;
        vulkanData.m_nSampleCount = m_nMSAASampleCount;
        vr::Texture_t texture = { &vulkanData, vr::TextureType_Vulkan, vr::ColorSpace_Auto };




//
//        VkDeviceSize nBufferSize = 0;
//        uint8_t *pBuffer = new uint8_t[ m_nRenderWidth * m_nRenderHeight * 4 * 2 ];
//        uint8_t *pPrevBuffer = pBuffer;
//        uint8_t *pCurBuffer = pBuffer;
//        memcpy( pCurBuffer, hmd_panel_img[L].data, sizeof( uint8_t ) * m_nRenderWidth * m_nRenderHeight * 3 );// 4 -> 3
//        pCurBuffer += sizeof( uint8_t ) * m_nRenderWidth * m_nRenderHeight * 4;
//
//        std::vector< VkBufferImageCopy > bufferImageCopies;
//        VkBufferImageCopy bufferImageCopy = {};
//        bufferImageCopy.bufferOffset = 0;
//        bufferImageCopy.bufferRowLength = 0;
//        bufferImageCopy.bufferImageHeight = 0;
//        bufferImageCopy.imageSubresource.baseArrayLayer = 0;
//        bufferImageCopy.imageSubresource.layerCount = 1;
//        bufferImageCopy.imageSubresource.mipLevel = 0;
//        bufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
//        bufferImageCopy.imageOffset.x = 0;
//        bufferImageCopy.imageOffset.y = 0;
//        bufferImageCopy.imageOffset.z = 0;
//        bufferImageCopy.imageExtent.width = m_nRenderWidth;
//        bufferImageCopy.imageExtent.height = m_nRenderHeight;
//        bufferImageCopy.imageExtent.depth = 1;
//        bufferImageCopies.push_back( bufferImageCopy );
//        int nMipWidth = m_nRenderWidth;
//        int nMipHeight = m_nRenderHeight;
//        while( nMipWidth > 1 && nMipHeight > 1 )
//        {
//          GenMipMapRGBA( pPrevBuffer, pCurBuffer, nMipWidth, nMipHeight, &nMipWidth, &nMipHeight );
//          bufferImageCopy.bufferOffset = pCurBuffer - pBuffer;
//          bufferImageCopy.imageSubresource.mipLevel++;
//          bufferImageCopy.imageExtent.width = nMipWidth;
//          bufferImageCopy.imageExtent.height = nMipHeight;
//          bufferImageCopies.push_back( bufferImageCopy );
//          pPrevBuffer = pCurBuffer;
//          pCurBuffer += ( nMipWidth * nMipHeight * 4 * sizeof( uint8_t ) );
//        }
//        nBufferSize = pCurBuffer - pBuffer;
//
//        // Create the image
//        VkImageCreateInfo imageCreateInfo = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
//        imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
//        imageCreateInfo.extent.width = m_nRenderWidth;
//        imageCreateInfo.extent.height = m_nRenderHeight;
//        imageCreateInfo.extent.depth = 1;
//        imageCreateInfo.mipLevels = ( uint32_t ) bufferImageCopies.size();
//        imageCreateInfo.arrayLayers = 1;
//        imageCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
//        imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
//        imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
//        imageCreateInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
//        imageCreateInfo.flags = 0;
//        vkCreateImage( m_pDevice, &imageCreateInfo, nullptr, &m_pSceneImage );
//
//
//        vulkanData.m_nImage = ( uint64_t ) m_leftEyeDesc.m_pImage;


//        vkCmdCopyBufferToImage( m_currentCommandBuffer.m_pCommandBuffer, m_pSceneStagingBuffer, m_pSceneImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, ( uint32_t ) bufferImageCopies.size(), &bufferImageCopies[ 0 ] );

//        std::cerr<<"test_ptr"<<test_ptr<<std::endl;
//        memcpy( test_ptr, hmd_panel_img[L].data, sizeof( uint8_t ) * m_nRenderWidth * m_nRenderHeight  );
//        memcpy( test_ptr, hmd_panel_img[L].data, sizeof( uint8_t )*100);






        vr::VRCompositor()->Submit( vr::Eye_Left, &texture, &bounds );
        vulkanData.m_nImage = ( uint64_t ) m_rightEyeDesc.m_pImage;
        vr::VRCompositor()->Submit( vr::Eye_Right, &texture, &bounds );
      }
      VkPresentInfoKHR presentInfo = { VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
      presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
      presentInfo.pNext = NULL;
      presentInfo.swapchainCount = 1;
      presentInfo.pSwapchains = &m_pSwapchain;
      presentInfo.pImageIndices = &m_nCurrentSwapchainImage;
      vkQueuePresentKHR( m_pQueue, &presentInfo );
      // Spew out the controller and pose count whenever they change.
      if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last ) {
        m_iValidPoseCount_Last = m_iValidPoseCount;
        m_iTrackedControllerCount_Last = m_iTrackedControllerCount;
        dprintf( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
      }
      UpdateHMDMatrixPose();
      m_nFrameIndex = ( m_nFrameIndex + 1 ) % m_swapchainImages.size();
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
//      VkDeviceSize nBufferSize = 0;
//      uint8_t *pBuffer = new uint8_t[ m_nRenderWidth * m_nRenderHeight * 4 * 2 ];
//      uint8_t *pPrevBuffer = pBuffer;
//      uint8_t *pCurBuffer = pBuffer;
//      memcpy( pCurBuffer, &imageRGBA[0], sizeof( uint8_t ) * m_nRenderWidth * m_nRenderHeight * 4 );
//      pCurBuffer += sizeof( uint8_t ) * m_nRenderWidth * m_nRenderHeight * 4;
//
//      std::vector< VkBufferImageCopy > bufferImageCopies;
//      VkBufferImageCopy bufferImageCopy = {};
//      bufferImageCopy.bufferOffset = 0;
//      bufferImageCopy.bufferRowLength = 0;
//      bufferImageCopy.bufferImageHeight = 0;
//      bufferImageCopy.imageSubresource.baseArrayLayer = 0;
//      bufferImageCopy.imageSubresource.layerCount = 1;
//      bufferImageCopy.imageSubresource.mipLevel = 0;
//      bufferImageCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
//      bufferImageCopy.imageOffset.x = 0;
//      bufferImageCopy.imageOffset.y = 0;
//      bufferImageCopy.imageOffset.z = 0;
//      bufferImageCopy.imageExtent.width = m_nRenderWidth;
//      bufferImageCopy.imageExtent.height = m_nRenderHeight;
//      bufferImageCopy.imageExtent.depth = 1;
//      bufferImageCopies.push_back( bufferImageCopy );
//
//      int nMipWidth = m_nRenderWidth;
//      int nMipHeight = m_nRenderHeight;
//
//      while( nMipWidth > 1 && nMipHeight > 1 )
//      {
//        GenMipMapRGBA( pPrevBuffer, pCurBuffer, nMipWidth, nMipHeight, &nMipWidth, &nMipHeight );
//        bufferImageCopy.bufferOffset = pCurBuffer - pBuffer;
//        bufferImageCopy.imageSubresource.mipLevel++;
//        bufferImageCopy.imageExtent.width = nMipWidth;
//        bufferImageCopy.imageExtent.height = nMipHeight;
//        bufferImageCopies.push_back( bufferImageCopy );
//        pPrevBuffer = pCurBuffer;
//        pCurBuffer += ( nMipWidth * nMipHeight * 4 * sizeof( uint8_t ) );
//      }
//      nBufferSize = pCurBuffer - pBuffer;
//
//      // Create the image
//      VkImageCreateInfo imageCreateInfo = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
//      imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
//      imageCreateInfo.extent.width = m_nRenderWidth;
//      imageCreateInfo.extent.height = m_nRenderHeight;
//      imageCreateInfo.extent.depth = 1;
//      imageCreateInfo.mipLevels = ( uint32_t ) bufferImageCopies.size();
//      imageCreateInfo.arrayLayers = 1;
//      imageCreateInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
//      imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
//      imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
//      imageCreateInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
//      imageCreateInfo.flags = 0;
//      vkCreateImage( m_pDevice, &imageCreateInfo, nullptr, &m_pSceneImage );
    }
};
#endif
#else
// import from opengl sample
std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
  if( unRequiredBufferLen == 0 )
    return "";

  char *pchBuffer = new char[ unRequiredBufferLen ];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
  std::string sResult = pchBuffer;
  delete [] pchBuffer;
  return sResult;
}
#endif


class VIVEnode
{
  public:
    VIVEnode(int rate);
    ~VIVEnode();
    bool Init();
    void Run();
    void Shutdown();
    bool setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void set_feedback(sensor_msgs::JoyFeedbackConstPtr msg);
    ros::NodeHandle nh_;
    VRInterface vr_;

#ifdef USE_IMAGE
    void imageCb_L(const sensor_msgs::ImageConstPtr& msg);
    void imageCb_R(const sensor_msgs::ImageConstPtr& msg);
    void infoCb_L(const sensor_msgs::CameraInfoConstPtr& msg);
    void infoCb_R(const sensor_msgs::CameraInfoConstPtr& msg);
    CMainApplicationMod *pMainApplication;
    image_transport::Subscriber sub_L,sub_R;
    ros::Subscriber sub_i_L,sub_i_R;
#endif

  private:
    ros::Rate loop_rate_;
    std::vector<double> world_offset_;
    double world_yaw_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    ros::ServiceServer set_origin_server_;
    ros::Publisher twist0_pub_;
    ros::Publisher twist1_pub_;
    ros::Publisher twist2_pub_;
    std::map<std::string, ros::Publisher> button_states_pubs_map;
    ros::Subscriber feedback_sub_;

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
  twist0_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist0", 10);
  twist1_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist1", 10);
  twist2_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist2", 10);
  feedback_sub_ = nh_.subscribe("/vive/set_feedback", 10, &VIVEnode::set_feedback, this);

#ifdef USE_IMAGE
  image_transport::ImageTransport it(nh_);
  sub_L = it.subscribe("/image_left", 1, &VIVEnode::imageCb_L, this);
  sub_R = it.subscribe("/image_right", 1, &VIVEnode::imageCb_R, this);
  sub_i_L = nh_.subscribe("/camera_info_left", 1, &VIVEnode::infoCb_L, this);
  sub_i_R = nh_.subscribe("/camera_info_right", 1, &VIVEnode::infoCb_R, this);
  pMainApplication = new CMainApplicationMod( 0, NULL );
  if (!pMainApplication->BInit()){
    pMainApplication->Shutdown();
    Shutdown();
  }
  pMainApplication->vr_p = &(vr_);
  pMainApplication->InitTextures();
#endif
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

void VIVEnode::set_feedback(sensor_msgs::JoyFeedbackConstPtr msg) {
  if(msg->type == 1 /* TYPE_RUMBLE */) {
    vr_.TriggerHapticPulse(msg->id, 0, (int)(msg->intensity));
    for(int i=0;i<16;i++)
      vr_.TriggerHapticPulse(i, 0, (int)(msg->intensity));
  }
}

void VIVEnode::Run()
{
  double tf_matrix[3][4];
  int run_hz_count = 0;

  while (ros::ok())
  {
    // do stuff
    vr_.Update();

    int controller_count = 1;
    int tracker_count = 1;
    int lighthouse_count = 1;
    for (int i=0; i<vr::k_unMaxTrackedDeviceCount; i++)
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
      //get device serial number
      std::string cur_sn = GetTrackedDeviceString( vr_.pHMD_, i, vr::Prop_SerialNumber_String );
      std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');

      // It's a HMD
      if (dev_type == 1)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "hmd"));
      }
      // It's a controller
      if (dev_type == 2)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "controller_"+cur_sn));

        vr::VRControllerState_t state;
        vr_.HandleInput(i, state);
        sensor_msgs::Joy joy;
        joy.header.stamp = ros::Time::now();
        joy.header.frame_id = "controller_"+cur_sn;
        joy.buttons.assign(BUTTON_NUM, 0);
        joy.axes.assign(AXES_NUM, 0.0); // x-axis, y-axis
        if((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
          joy.buttons[0] = 1;
        if((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
          joy.buttons[1] = 1;
        if((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
          joy.buttons[2] = 1;
        if((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
          joy.buttons[3] = 1;
        // TrackPad's axis
        joy.axes[0] = state.rAxis[0].x;
        joy.axes[1] = state.rAxis[0].y;
        // Trigger's axis
        joy.axes[2] = state.rAxis[1].x;
//        #include <bitset> // bit debug
//        std::cout << static_cast<std::bitset<64> >(state.ulButtonPressed) << std::endl;
//        std::cout << static_cast<std::bitset<64> >(state.ulButtonTouched) << std::endl;
        if(button_states_pubs_map.count(cur_sn) == 0){
          button_states_pubs_map[cur_sn] = nh_.advertise<sensor_msgs::Joy>("/vive/controller_"+cur_sn+"/joy", 10);
        }
        button_states_pubs_map[cur_sn].publish(joy);
      }
      // It's a tracker
      if (dev_type == 3)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "tracker_"+cur_sn));
      }
      // It's a lighthouse
      if (dev_type == 4)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "lighthouse_"+cur_sn));
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
    double lin_vel[3], ang_vel[3];
    if (vr_.GetDeviceVel(0, lin_vel, ang_vel))
    {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = lin_vel[0];
        twist_msg.linear.y = lin_vel[1];
        twist_msg.linear.z = lin_vel[2];
        twist_msg.angular.x = ang_vel[0];
        twist_msg.angular.y = ang_vel[1];
        twist_msg.angular.z = ang_vel[2];

        geometry_msgs::TwistStamped twist_msg_stamped;
        twist_msg_stamped.header.stamp = ros::Time::now();
        twist_msg_stamped.header.frame_id = "world_vive";
        twist_msg_stamped.twist = twist_msg;

        twist0_pub_.publish(twist_msg_stamped);
     
        // std::cout<<"HMD:";
        // std::cout<<twist_msg_stamped;
    }
    if (vr_.GetDeviceVel(1, lin_vel, ang_vel))
    {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = lin_vel[0];
        twist_msg.linear.y = lin_vel[1];
        twist_msg.linear.z = lin_vel[2];
        twist_msg.angular.x = ang_vel[0];
        twist_msg.angular.y = ang_vel[1];
        twist_msg.angular.z = ang_vel[2];

        geometry_msgs::TwistStamped twist_msg_stamped;
        twist_msg_stamped.header.stamp = ros::Time::now();
        twist_msg_stamped.header.frame_id = "world_vive";
        twist_msg_stamped.twist = twist_msg;

        twist1_pub_.publish(twist_msg_stamped);
     
        // std::cout<<"Controller 1:";
        // std::cout<<twist_msg_stamped;
    }
    if (vr_.GetDeviceVel(2, lin_vel, ang_vel))
    {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = lin_vel[0];
        twist_msg.linear.y = lin_vel[1];
        twist_msg.linear.z = lin_vel[2];
        twist_msg.angular.x = ang_vel[0];
        twist_msg.angular.y = ang_vel[1];
        twist_msg.angular.z = ang_vel[2];

        geometry_msgs::TwistStamped twist_msg_stamped;
        twist_msg_stamped.header.stamp = ros::Time::now();
        twist_msg_stamped.header.frame_id = "world_vive";
        twist_msg_stamped.twist = twist_msg;

        twist2_pub_.publish(twist_msg_stamped);
     
        // std::cout<<"Controller 2:";
        // std::cout<<twist_msg_stamped;
    }
   
#ifdef USE_IMAGE
    pMainApplication->HandleInput();
    pMainApplication->RenderFrame();
#endif

    ROS_INFO_THROTTLE(1.0,"Run() @ %d [fps]", [](int& cin){int ans = cin; cin=0; return ans;}(run_hz_count));
    run_hz_count++;
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

#ifdef USE_IMAGE
void VIVEnode::imageCb_L(const sensor_msgs::ImageConstPtr& msg){
  if(msg->width > 0 && msg->height > 0 ){
    try {
      pMainApplication->ros_img[L] = cv_bridge::toCvCopy(msg,"rgb8")->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(1, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
  }else{
    ROS_WARN_THROTTLE(3, "Invalid image_left size (%dx%d) use default", msg->width, msg->height);
  }
}
void VIVEnode::imageCb_R(const sensor_msgs::ImageConstPtr& msg){
  if(msg->width > 0 && msg->height > 0 ){
    try {
      pMainApplication->ros_img[R] = cv_bridge::toCvCopy(msg,"rgb8")->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_THROTTLE(1, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
  }else{
    ROS_WARN_THROTTLE(3, "Invalid image_right size (%dx%d) use default", msg->width, msg->height);
  }
}
void VIVEnode::infoCb_L(const sensor_msgs::CameraInfoConstPtr& msg){
  if(msg->K[0] > 0.0 && msg->K[4] > 0.0 ){
    pMainApplication->cam_f[L][0] = msg->K[0];
    pMainApplication->cam_f[L][1] = msg->K[4];
  }else{
    ROS_WARN_THROTTLE(3, "Invalid camera_info_left fov (%fx%f) use default", msg->K[0], msg->K[4]);
  }
}
void VIVEnode::infoCb_R(const sensor_msgs::CameraInfoConstPtr& msg){
  if(msg->K[0] > 0.0 && msg->K[4] > 0.0 ){
    pMainApplication->cam_f[R][0] = msg->K[0];
    pMainApplication->cam_f[R][1] = msg->K[4];
  }else{
    ROS_WARN_THROTTLE(3, "Invalid camera_info_right fov (%fx%f) use default", msg->K[0], msg->K[4]);
  }
}
#endif


// Main
int main(int argc, char** argv){
  signal(SIGINT, mySigintHandler);
  ros::init(argc, argv, "vive_node");

#ifdef USE_IMAGE
  VIVEnode nodeApp(90); // VIVE display max fps
#else
  VIVEnode nodeApp(30);
#endif
  if (!nodeApp.Init()){
    nodeApp.Shutdown();
    return 1;
  }
  
  nodeApp.Run();
  nodeApp.Shutdown();
  

  return 0;
};
