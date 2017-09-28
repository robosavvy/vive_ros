#ifndef _VR_INTERFACE_H_
#define _VR_INTERFACE_H_

#include <openvr.h>
#include <boost/function.hpp>

typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;


class VRInterface
{
  public:
    VRInterface();
    ~VRInterface();
    
    bool Init();
    void Shutdown();
    
    void Update();
    void UpdateCalibration();
    
    int GetDeviceMatrix(int index, double pMatrix[3][4]);
    int GetDeviceVel(int index, double lin_vel[3], double ang_vel[3]);
    bool IsDeviceConnected(int index);

    // Head-Mounted Displays.
    bool IsHMDConnected();
    bool IsHMD(int deviceType);

    // Tracked controllers.
    bool IsControllerConnected();
    bool IsController(int deviceType);

    // Generic trackers, similar to controllers.
    bool IsGenericTrackerConnected();
    bool IsGenericTracker(int deviceType);


    // Camera and base stations that serve as tracking reference points.
    // Also known as 'lighthouse'.
    bool IsGenericTrackingReferenceConnected();
    bool IsGenericTrackingReference(int deviceType);

    // Accessories that aren't necessarily tracked themselves,
    // but may redirect video output from other tracked devices.
    bool IsGenericDisplayRedirectConnected();
    bool IsGenericDisplayRedirect(int deviceType);
    
    void setErrorMsgCallback(ErrorMsgCallback fn);
    void setInfoMsgCallback(InfoMsgCallback fn);
    void setDebugMsgCallback(DebugMsgCallback fn);


  private:
    vr::IVRSystem *pHMD_;
    vr::IVRChaperone *pChaperone_;
  public:
    uint max_devices_;
  private:
    vr::TrackedDevicePose_t device_poses_[vr::k_unMaxTrackedDeviceCount];

  private:
    DebugMsgCallback debug_;
    InfoMsgCallback info_;
    ErrorMsgCallback error_;
    
    vr::ChaperoneCalibrationState cal_state_;
    float play_area_[2];
    vr::HmdQuad_t play_quat_;
    
    std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL);
};

#endif  // _VR_INTERFACE_H_
