#include <iostream>
#include <string>
#include <map>
#include "vive_ros/vr_interface.h"

inline void defaultDebugMsgCallback(const std::string &msg) {
    std::cerr << "VIVE Debug: " << msg << std::endl;
}

inline void defaultInfoMsgCallback(const std::string &msg) {
    std::cerr << "VIVE Info: " << msg << std::endl;
}

inline void defaultErrorMsgCallback(const std::string &msg) {
    std::cerr << "VIVE Error: " << msg << std::endl;
}

std::map<vr::ChaperoneCalibrationState, std::string> mapChaperonStrings
{ 
  { vr::ChaperoneCalibrationState_OK, "Chaperone is fully calibrated and working correctly" }, 
  { vr::ChaperoneCalibrationState_Warning, "Warning" },
  { vr::ChaperoneCalibrationState_Warning_BaseStationMayHaveMoved, "A base station thinks that it might have moved" },
  { vr::ChaperoneCalibrationState_Warning_BaseStationRemoved, "There are less base stations than when calibrated" },
  { vr::ChaperoneCalibrationState_Warning_SeatedBoundsInvalid, "Seated bounds haven't been calibrated for the current tracking center" },
  { vr::ChaperoneCalibrationState_Error, "The UniverseID is invalid" },
  { vr::ChaperoneCalibrationState_Error_BaseStationUninitalized, "Tracking center hasn't be calibrated for at least one of the base stations" },
  { vr::ChaperoneCalibrationState_Error_BaseStationConflict, "Tracking center is calibrated, but base stations disagree on the tracking space" },
  { vr::ChaperoneCalibrationState_Error_PlayAreaInvalid, "Play Area hasn't been calibrated for the current tracking center" },
  { vr::ChaperoneCalibrationState_Error_CollisionBoundsInvalid, "Collision Bounds haven't been calibrated for the current tracking center" }
};

VRInterface::VRInterface()
  : error_(defaultErrorMsgCallback)
  , debug_(defaultDebugMsgCallback)
  , info_(defaultInfoMsgCallback)
  , max_devices_(5) // or vr::k_unMaxTrackedDeviceCount
{
  play_area_[0] = -1;
  play_area_[1] = -1;
  for (int i=0; i<4; i++) 
    for (int o=0; o<3; o++)
      play_quat_.vCorners[i].v[o] = -1;
      
  return;
}

VRInterface::~VRInterface()
{
  return;
}

bool VRInterface::Init()
{
  // Loading the SteamVR Runtime
  vr::EVRInitError eError = vr::VRInitError_None;
  
  pHMD_ = vr::VR_Init( &eError, vr::VRApplication_Scene );

  if (eError != vr::VRInitError_None)
  {
    pHMD_ = NULL;
    error_("VR_Init Failed.");
    return false;
  }

  info_("VR_Init Success.");
  //~ strDriver_ = GetTrackedDeviceString( pHMD_, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String );
  //~ strDisplay_ = GetTrackedDeviceString( pHMD_, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String );
  //~ info_("Device: " + strDriver_ + ", " + strDisplay_);
  
  //~ pHMD_->ResetSeatedZeroPose();
  
  UpdateCalibration();

  return true;
}

void VRInterface::Shutdown()
{
  info_("Shutting down.");
  if( pHMD_ )
  {
    vr::VR_Shutdown();
    pHMD_ = NULL;
  }
}

void VRInterface::Update()
{
  if (pHMD_)
  {
    pHMD_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, device_poses_, max_devices_);
                                    
    //~ for (vr::TrackedDeviceIndex_t device_index = vr::k_unTrackedDeviceIndex_Hmd; device_index < max_devices_; ++device_index)
    //~ {
      //~ if (device_poses_[device_index].bDeviceIsConnected && device_poses_[device_index].bPoseIsValid)
      //~ {
        //~ info_("device[" + std::to_string(device_index) + "]: " + std::to_string(pHMD_->GetTrackedDeviceClass(device_index)) + " " + std::to_string(device_poses_[device_index].eTrackingResult));
      //~ }
    //~ }
  }
}

bool VRInterface::IsDeviceConnected(int index)
{
  if (index < max_devices_)
  {
    return pHMD_->IsTrackedDeviceConnected(index);
  }
  return false;
}

int VRInterface::GetDeviceMatrix(int index, double pMatrix[3][4])
{
  if (index < max_devices_)
  {
    if (device_poses_[index].bDeviceIsConnected && device_poses_[index].bPoseIsValid && device_poses_[index].eTrackingResult == vr::TrackingResult_Running_OK)
    {
      // +y is up
      // +x is to the right
      // -z is going away from you
      for (int i=0; i<3; i++)
        for (int o=0; o<4; o++)
          pMatrix[i][o] = static_cast<double>(device_poses_[index].mDeviceToAbsoluteTracking.m[i][o]);

      
      return pHMD_->GetTrackedDeviceClass(index);
    }
  }
  return 0;
}

int VRInterface::GetDeviceVel(int index, double lin_vel[3], double ang_vel[3])
{
  if (index < max_devices_)
  {
    if (device_poses_[index].bDeviceIsConnected && device_poses_[index].eTrackingResult == vr::TrackingResult_Running_OK)
    {
      for (int i=0; i<3; i++)
      {
        lin_vel[i] = device_poses_[index].vVelocity.v[i];
        ang_vel[i] = device_poses_[index].vAngularVelocity.v[i];
      }
      return pHMD_->GetTrackedDeviceClass(index);
    }
  }
    
  return 0;
}

void VRInterface::UpdateCalibration()
{
  vr::EVRInitError eError = vr::VRInitError_None;
  // Print out calibration state
  pChaperone_ = (vr::IVRChaperone *)vr::VR_GetGenericInterface(vr::IVRChaperone_Version, &eError);
  if (eError != 0)
  {
    error_("Could not find chaperone");
  }
  cal_state_ = pChaperone_->GetCalibrationState();
  info_("Calibration state: " + mapChaperonStrings[cal_state_]);
  if (!pChaperone_->GetPlayAreaSize(&play_area_[0], &play_area_[1]))
    info_("Play area: " + std::to_string(play_area_[0]) + " x " + std::to_string(play_area_[1]));
  else
    info_("Empty play area.");
    
  if (!pChaperone_->GetPlayAreaRect(&play_quat_))
    for (int i=0; i<4; i++)
      info_("Corner " + std::to_string(i) + " - x: " + std::to_string(play_quat_.vCorners[i].v[0]) + ", y: " + std::to_string(play_quat_.vCorners[i].v[1])  + ", z:" + std::to_string(play_quat_.vCorners[i].v[2]));
  
}

std::string VRInterface::GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError )
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

void VRInterface::setErrorMsgCallback(ErrorMsgCallback fn) { error_ = fn; }
void VRInterface::setInfoMsgCallback(InfoMsgCallback fn) { info_ = fn; }
void VRInterface::setDebugMsgCallback(DebugMsgCallback fn) { debug_ = fn; }
