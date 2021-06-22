#include <ros/ros.h>                			 // ros::NodeHandle
#include <dynamic_reconfigure/Reconfigure.h>	 // dynamic_reconfigure::Reconfigure
#include <dynamic_reconfigure/Config.h>			 // dynamic_reconfigure::Config

#include <dynamic_reconfigure/IntParameter.h>	 // dynamic_reconfigure::IntParameter
#include <dynamic_reconfigure/DoubleParameter.h> // dynamic_reconfigure::DoubleParameter

#include "GatingWrapper.h"          			 // GatingWrapper


GatingWrapper::GatingWrapper(context_t* context,
		const ros::NodeHandle& nh, int isSlave):
	m_context(context),
    m_nh(nh),
	m_isSlave(isSlave),
	m_illumMode(0),
	m_illumPower(100),
	m_illumTlaser(35),
	m_rangeStart(78),
	m_rangeEnd(210),
	m_curvature(0)
{
	if (false == m_isSlave)
	{
		bwv_api_gating_add_callback_illumination_working_mode(m_context->apiMaster,
				IlluminationModeCallback, this);
		bwv_api_gating_add_callback_illumination_power(m_context->apiMaster,
				IlluminationPowerCallback, this);
		bwv_api_gating_add_callback_illumination_tlaser(m_context->apiMaster,
				IlluminationTlaserCallback, this);
		bwv_api_gating_add_callback_illumination_range(m_context->apiMaster,
				IlluminationRangeCallback, this);
		bwv_api_gating_add_callback_curvature(m_context->apiMaster,
				CurvatureCallback, this);
	}
	else
	{
		bwv_api_gating_add_callback_illumination_working_mode(m_context->apiSlave,
				IlluminationModeCallback, this);
		bwv_api_gating_add_callback_illumination_power(m_context->apiSlave,
				IlluminationPowerCallback, this);
		bwv_api_gating_add_callback_illumination_tlaser(m_context->apiSlave,
				IlluminationTlaserCallback, this);
		bwv_api_gating_add_callback_illumination_range(m_context->apiSlave,
				IlluminationRangeCallback, this);
		bwv_api_gating_add_callback_curvature(m_context->apiSlave,
				CurvatureCallback, this);
	}
}


void GatingWrapper::IlluminationModeCallback(unsigned char data, void* param)
{
	GatingWrapper* gate = static_cast<GatingWrapper*>(param);
	gate->m_illumMode = data;

	dynamic_reconfigure::IntParameter workingModeParam;
	dynamic_reconfigure::Config conf;

	workingModeParam.name = "CameraWorkingMode";
	workingModeParam.value = static_cast<unsigned char>(data);
	conf.ints.push_back(workingModeParam);

	gate->m_srvReq.config = conf;

	if (ros::service::call("/bwv_camera_driver/bwv_camera/set_parameters",
			gate->m_srvReq, gate->m_srvResp))
	{
//		ROS_INFO("call to set working mode succeeded");
	}
	else
	{
//		ROS_INFO("call to set working mode failed");
	}

}

void GatingWrapper::IlluminationPowerCallback(unsigned char data, void* param)
{
	GatingWrapper* gate = static_cast<GatingWrapper*>(param);
	gate->m_illumMode = data;

	dynamic_reconfigure::IntParameter illuminationPower;
	dynamic_reconfigure::Config conf;

	illuminationPower.name = "IlluminationPower";
	illuminationPower.value = static_cast<unsigned char>(data);
	conf.ints.push_back(illuminationPower);

	gate->m_srvReq.config = conf;

	if (ros::service::call("/bwv_camera_driver/bwv_camera/set_parameters",
			gate->m_srvReq, gate->m_srvResp))
	{
//		ROS_INFO("call to set illumination power succeeded");
	}
	else
	{
//		ROS_INFO("call to set illumination power failed");
	}
}

void GatingWrapper::IlluminationTlaserCallback(unsigned char data, void* param)
{
	GatingWrapper* gate = static_cast<GatingWrapper*>(param);
	gate->m_illumMode = data;

	dynamic_reconfigure::DoubleParameter tlaserParam;
	dynamic_reconfigure::Config conf;

	tlaserParam.name = "Tlaser";
	tlaserParam.value = static_cast<double>(data) / 100;
	conf.doubles.push_back(tlaserParam);

	gate->m_srvReq.config = conf;

	if (ros::service::call("/bwv_camera_driver/bwv_camera/set_parameters",
			gate->m_srvReq, gate->m_srvResp))
	{
//		ROS_INFO("call to set tlaser succeeded");
	}
	else
	{
//		ROS_INFO("call to set tlaser failed");
	}
}

void GatingWrapper::IlluminationRangeCallback(unsigned short start, unsigned short end, void* param)
{
	GatingWrapper* gate = static_cast<GatingWrapper*>(param);
	gate->m_rangeStart = start;
	gate->m_rangeEnd = end;

	dynamic_reconfigure::Config conf;

	dynamic_reconfigure::IntParameter rangeStartParam;
	rangeStartParam.name = "min_range";
	rangeStartParam.value = static_cast<int>(start);
	conf.ints.push_back(rangeStartParam);

	dynamic_reconfigure::IntParameter rangeEndParam;
	rangeEndParam.name = "max_range";
	rangeEndParam.value = static_cast<int>(end);
	conf.ints.push_back(rangeEndParam);

	gate->m_srvReq.config = conf;

	if (ros::service::call("/bwv_camera_driver/bwv_camera/set_parameters",
			gate->m_srvReq, gate->m_srvResp))
	{
//		ROS_INFO("call to set range succeeded");
	}
	else
	{
//		ROS_INFO("call to set range failed");
	}
}

void GatingWrapper::CurvatureCallback(unsigned char data, void* param)
{
	static_cast<GatingWrapper*>(param)->m_curvature = data;
}
