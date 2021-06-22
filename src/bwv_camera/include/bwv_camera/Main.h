#ifndef __MAIN_H__
#define __MAIN_H__

#include <ros/ros.h>        					// ROS
#include <memory>								// shared_ptr

#include "bwv_api.h"       						// device_t
#include "VideoWrapper.h"   					// VideoWrapper
#include "GatingWrapper.h"  					// GatingWrapper
#include <bwv_camera/BWVCameraConfig.h>         // BWVCameraConfig
#include <dynamic_reconfigure/server.h>         // dynamic_reconfigure::Server

/// this is main class.
///
/// controls the driver's operation flow.
class Main
{
public:
	/// Main ctor
	///
	/// @param context - pointer to the global related content
	/// @param nodeHandle - node handle instance
	/// @see ContextStruct ros::NodeHandle
    explicit Main(context_t* context, const ros::NodeHandle& nodeHandle);

    /// dtor
    ~Main();

    /// start the camera operation and flow.
    ///
    /// @returns 0 for success \n -1 for allocation fail \n -2 for connection fail
    int StartCamera();

    /// @return the instance of master's camera gating wrapper
    /// @see GatingWrapper
    GatingWrapper* GetGatingMaster() { return m_gateMaster; }

    /// same as GetGatingMaster for slave camera
    /// @see GatingWrapper
    GatingWrapper* GetGatingSlave()  { return m_gateSlave ; }

private:
    Main(const Main& other) = delete;
    Main operator=(const Main& other) = delete;

    context_t* m_context;
    const ros::NodeHandle& m_nh;
    VideoWrapper* m_vidMaster;
    VideoWrapper* m_vidSlave;
    GatingWrapper* m_gateMaster;
    GatingWrapper* m_gateSlave;
    void* m_autoHandle;


    // Dynamic reconfigure handle
    std::shared_ptr<dynamic_reconfigure::Server<bwv_camera::BWVCameraConfig>> m_server;

    // callbacks
    void ConfigCallback(bwv_camera::BWVCameraConfig &config, uint32_t level);
    static void CallbackDisconnect(void* param);
    static void DiscoveryCallback(device_t device, void* param);
    static void DisconnectCallback(device_t device, void *param);
    static void AlertCallback(gating_data_t data, void* param);
    

    // auto connect thread
    bool m_autoConnectThread;
    bool m_autoConnect;
    static void AutoConnectThread(void* param);

};

#endif // __MAIN_H__
