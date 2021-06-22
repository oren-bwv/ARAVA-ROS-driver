#include <ros/ros.h>                            // ros::NodeHandle
#include <image_transport/image_transport.h>    // image_transport::ImageTransport
#include <cv_bridge/cv_bridge.h>                // cv_bridge
#include <chrono>                               // chrono::time_point, steady_clock
#include <string>								// std::string
//#include <ctime>								// acstime

#include "VideoWrapper.h"                       // VideoWrapper
#include "bwv_types.h"							// config_t, g_metadataTimer
#include "utils.h"								// Sleep

static uint64_t tsm = bwv_api_get_timestamp();
static uint64_t tss = bwv_api_get_timestamp();


VideoWrapper::VideoWrapper(context_t* context, const ros::NodeHandle& nh,
     int masterOrSlave):
    m_context(context),
    m_nh(nh),
    m_mat(),
    m_it(nh),
    m_imagePub(),
    m_imageMsg(),
    m_isSlave(masterOrSlave)
{
    if (m_isSlave == MASTER)
    {
        bwv_api_capture_add_callback_data(m_context->apiMaster,
        		CallbackVideoDataMaster, this);
        m_imagePub = m_it.advertise("bwv_camera/image_master", 100);
        int ret = bwv_api_capture_start_preview(m_context->apiMaster);

        if (0 == ret)
        {
            ROS_INFO("capture master started");
        }
        else
        {
            ROS_ERROR("preview master failed, error: %d", ret);
        }
    }
    else if (m_isSlave == SLAVE)
    {
        bwv_api_capture_add_callback_data(m_context->apiSlave,
        		CallbackVideoDataSlave, this);
        m_imagePub = m_it.advertise("bwv_camera/image_slave", 100);
        int ret = bwv_api_capture_start_preview(m_context->apiSlave);

        if (0 == ret)
        {
            ROS_INFO("capture slave started");
        }
        else
        {
            ROS_ERROR("preview slave failed, error: %d", ret);
        }
    }
}

VideoWrapper::~VideoWrapper()
{
    ROS_INFO("video wrapper destructor");

    if (m_isSlave == MASTER)
    {
        bwv_api_capture_stop_preview(m_context->apiMaster);
        Sleep(1500);
    }
    else
    {
        bwv_api_capture_stop_preview(m_context->apiSlave);
        Sleep(1500);
    }
}

void VideoWrapper::CallbackVideoDataMaster(capture_data_t data, void* param)
{
    VideoWrapper* vidWrap = static_cast<VideoWrapper*>(param);

    static unsigned char s_power = 1;

    vidWrap->m_mat = cv::Mat(482, 800, CV_16UC1, data.rawimage);
    vidWrap->m_imageMsg = cv_bridge::CvImage(std_msgs::Header(),
        "mono16", vidWrap->m_mat).toImageMsg();


    vidWrap->m_imagePub.publish(vidWrap->m_imageMsg);

    static unsigned short frameCounter = 0;
    static int iter = 0;
    static bool isThirdIter = false;

    if ((data.rawimage[20] - frameCounter != 1) && (frameCounter -
    		data.rawimage[20] != 4095) && isThirdIter)
    {
    	ROS_WARN("Video is stuck, current frame counter: %d,"
    			" previous frame counter: %d",
				data.rawimage[20], frameCounter);
    }

	if (3 == iter)
	{
		isThirdIter = true;
	}

	frameCounter = data.rawimage[HW_FRAME_COUNTER];
	++iter;

	if (((long)bwv_api_get_timestamp() - (long)g_metadataTimer > 150) && isThirdIter)
	{
		ROS_INFO("%d", bwv_api_get_timestamp() - g_metadataTimer );
		ROS_INFO("%d", bwv_api_get_timestamp() - g_metadataTimer  > 300);
		vidWrap->CompareMetadata(MASTER, data.rawimage);
		g_metadataTimer = 200000000000000;
	}

    static uint32_t masterCounter = 0;
    ++masterCounter;
    if ((bwv_api_get_timestamp() - tsm) > 1000) {
    	ROS_DEBUG("Master FPS: %d", masterCounter);
        tsm = bwv_api_get_timestamp();
        masterCounter = 0;
        vidWrap->PrintMetadata(MASTER, data.rawimage);
    }

    // cv::Mat Im = cv_bridge::toCvShare(vidWrap->m_imageMsg, "mono16")->image;
}

void VideoWrapper::CallbackVideoDataSlave(capture_data_t data, void* param)
{
    VideoWrapper* vidWrap = static_cast<VideoWrapper*>(param);

    if (data.is_8bpp){
        // vidWrap->m_mat = cv::Mat(482, 800, CV_8UC1, data.rawimage);
        vidWrap->m_mat = cv::Mat(482, 800, CV_8UC1);
        for (size_t i = 0; i < data.height ; ++i)
        {
            for (size_t j = 0; j < data.width ; ++j)
            {
                vidWrap->m_mat.at<uchar>(i, j) = data.rawimage[i * data.width + j] & 0xff;
            }
        }
        vidWrap->m_imageMsg = cv_bridge::CvImage(std_msgs::Header(),
            "mono8", vidWrap->m_mat).toImageMsg();

    }
    else{
        vidWrap->m_mat = cv::Mat(482, 800, CV_16UC1, data.rawimage);
        vidWrap->m_imageMsg = cv_bridge::CvImage(std_msgs::Header(),
            "mono16", vidWrap->m_mat).toImageMsg();
    }

    vidWrap->m_imagePub.publish(vidWrap->m_imageMsg);

    static uint32_t slaveCounter = 0;
    ++slaveCounter;
    if ((bwv_api_get_timestamp() - tss) > 1000) {
        ROS_DEBUG("Slave FPS: %d", slaveCounter);
        tss = bwv_api_get_timestamp();
        slaveCounter = 0;
        vidWrap->PrintMetadata(SLAVE, data.rawimage);
    }

    // cv::Mat Im = cv_bridge::toCvShare(vidWrap->m_imageMsg, "mono16")->image;
}

void VideoWrapper::PrintMetadata(bool isSlave, uint16_t* data)
{
	std::string masterOrSlave = isSlave ? "SLAVE" : "MASTER";

    ROS_DEBUG("%s range_start: %d", masterOrSlave.c_str(), GetMetadata(data, X_START));
    ROS_DEBUG("%s range_end: %d", masterOrSlave.c_str(), GetMetadata(data, X_END));
    ROS_DEBUG("%s power: %d", masterOrSlave.c_str(), GetMetadata(data, POWER));
    ROS_DEBUG("%s tlaser: %d", masterOrSlave.c_str(), GetMetadata(data, TLASER));
    ROS_DEBUG("%s frame counter: %d", masterOrSlave.c_str(), data[HW_FRAME_COUNTER]);
    ROS_DEBUG("%s illuminator: %d", masterOrSlave.c_str(), GetMetadata(data, ILLUMINATOR));

    uint16_t workingMode = GetMetadata(data, WORKING_MODE);
    std::string workModStr;
    switch (workingMode)
    {
    case 0:	workModStr = "LRN";	break;
    case 1: workModStr = "BNN";	break;
    case 2: workModStr = "FULL";break;

    default: workModStr = "no work mode"; break;


    }
    ROS_DEBUG("%s working mode: %s", masterOrSlave.c_str(), workModStr.c_str());
    ROS_DEBUG("%s type %d series 0 pulses: %d, tlaser: %d",
    		masterOrSlave.c_str(), GetMetadata(data, 5),
			GetMetadata(data, 21), GetMetadata(data, 25));
    ROS_DEBUG("%s timestamp: %s\n", masterOrSlave.c_str(), GetTimeStamp(data).c_str());
}

int VideoWrapper::CompareMetadata(bool isSlave, uint16_t* data)
{
	ROS_ERROR_COND(m_context->config.illuminationPower != GetMetadata(data, POWER),
			"Error in metadata power, wanted: %d, received: %d",
			m_context->config.illuminationPower, GetMetadata(data, POWER));
	ROS_ERROR_COND(m_context->config.minRange != GetMetadata(data, X_START),
			"Error in metadata min_range, wanted: %d, received: %d",
			m_context->config.minRange, GetMetadata(data, X_START));
	ROS_ERROR_COND(m_context->config.maxRange != GetMetadata(data, X_END),
			"Error in metadata maxRange, wanted: %d, received: %d",
			m_context->config.maxRange, GetMetadata(data, X_END));
}

uint16_t VideoWrapper::GetMetadata(uint16_t* data, int pixelNumber)
{
	return data[FRAME_WIDTH + pixelNumber];
}

std::string VideoWrapper::GetTimeStamp(uint16_t* data)
{
	uint64_t time = (((uint64_t)data[8] ) | ((uint64_t)data[9] << 16) |
			((uint64_t)data[10] << 32) | (uint64_t)data[11] << 48);

	uint32_t reminder = time % 1000;
	time /= 1000;

	std::time_t result = time;
	std::string timeStr = std::asctime(std::localtime(&result));
	timeStr.pop_back();
	timeStr = timeStr + " milliseconds:" + std::to_string(reminder);

	return (timeStr);
}
