#include <ros/ros.h>                            // ros::init
#include <signal.h>                             // SIGINT
#include <ros/console.h>						// ros::console
#include <vector>								// std::vector

#include "bwv_api.h"                            // bwv camera driver
#include "bwv_types.h"							// context_t
#include "Main.h"                               // Main

//sig_atomic_t volatile g_request_shutdown = 0;

static Main* g_driver = nullptr;
static context_t g_context;


// Replacement for SIGINT handler
void SigIntHandler(int sig)
{
    ROS_INFO("closing node");
//    g_request_shutdown = 1;

    if (g_driver)
    {
        delete g_driver;
    }

    ros::requestShutdown();
}

int main(int argc, char** argv)
{

//	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
//		ros::console::levels::Debug) )
//	{
////	   ros::console::notifyLoggerLevelsChanged();
//	}

    ros::init(argc, argv, "bwv_camera_driver", ros::init_options::NoSigintHandler);
    signal(SIGINT, SigIntHandler);

    ros::NodeHandle nh("~/bwv_camera");

    ROS_INFO("ROS node ~/bwv_camera init");

    g_driver = new Main(&g_context, nh);
    g_driver->StartCamera();

    ros::spin();
    // another option:
    // while (!g_request_shutdown)
    // {
    //     // Do non-callback stuff
    //     ros::spinOnce();
    //     usleep(1000);
    // }

    ROS_INFO("Exit program");
    ros::shutdown();

    return (0);
}
