#ifndef __VIDEO_WRAPPER_H__
#define __VIDEO_WRAPPER_H__

#include <ros/ros.h>                            // NodeHandle
#include <image_transport/image_transport.h>    // image_transport::ImageTransport
#include <opencv2/highgui/highgui.hpp>          // opencv
#include <cv_bridge/cv_bridge.h>                // cv_bridge

#include "bwv_types.h"                          // context_t
#include "bwv_api.h"                            // capture_data_t


/// a wrapper to the video operations
///
/// responsible for getting video frames from a connected camera
/// master or slave, \n and convert it to a standard sensor_msgs format. \n
/// possible printing of a relevant metadata if ROS DEBUG verbosity is enabled.
class VideoWrapper
{
public:
	/// VideoWrapper ctor
	///
	/// @param context - pointer to the global related content
	/// @param nodeHandle - node handle instance
	/// @see ContextStruct ros::NodeHandle
    explicit VideoWrapper(context_t* context, const ros::NodeHandle& nh,
         int masterOrSlave);

    /// VideoWrapper dtor
    ~VideoWrapper();


private:
    // deleted special member functions
    VideoWrapper(const VideoWrapper& other) = delete;
    VideoWrapper operator=(const VideoWrapper& other) = delete;

    context_t* m_context;

    const ros::NodeHandle& m_nh;
    cv::Mat m_mat;
    image_transport::ImageTransport m_it;
    image_transport::Publisher m_imagePub ;
    sensor_msgs::ImagePtr m_imageMsg;
    int m_isSlave;

    // callback functions
    static void CallbackVideoDataMaster(capture_data_t data, void* param);
    static void CallbackVideoDataSlave(capture_data_t data, void* param);

    void printFPS();

    void PrintMetadata(bool isSlave, uint16_t* data);
    int CompareMetadata(bool isSlave, uint16_t* data);
    std::string GetTimeStamp(uint16_t* data);
    uint16_t GetMetadata(uint16_t* data, int value_offset);
};

#endif // __VIDEO_WRAPPER_H__
