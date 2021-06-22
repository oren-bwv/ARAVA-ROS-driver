#ifndef __GATING_WRAPPER_H__
#define __GATING_WRAPPER_H__

#include <ros/ros.h>							 // ros::NodeHandle
#include <dynamic_reconfigure/Reconfigure.h>	 // dynamic_reconfigure::Reconfigure

#include "bwv_api.h"        					 // gating_data_t
#include "bwv_types.h"							 // context_t

	/// a ros wrapper for the camera gating operation
class GatingWrapper
{
public:
	/// GatingWrapper ctor
	///
	/// @param context - pointer to the global related content
	/// @param nodeHandle - node handle instance
	/// @see ContextStruct ros::NodeHandle
    explicit GatingWrapper(context_t* context, const ros::NodeHandle& nh, int isSlave);

    /// GatingWrapper dtor
    ~GatingWrapper();

    /// @returns the current illumination mode
    /// @see Config
    unsigned char GetIllumMode() 	 { return m_illumMode;  }

    /// @return the current illumination power - \n represented in percents
    /// @see Config
    unsigned char GetIllumPower() 	 { return m_illumPower; }

    /// @returns the current laser time duration - ]n represented in micro seconds * 100
    /// @see Config
    unsigned char GetIllumTlaser() 	 { return m_illumTlaser;}

    /// @returns the current visible start range - \n represented in meters
    /// @see Config
    unsigned short GetRangeStart() 	 { return m_rangeStart; }

    /// @returns the current visible end range - \n represented in meters
    /// @see Config
    unsigned short GetRangeEnd()  	 { return m_rangeEnd;   }

    /// @ returns the current road curvature - \n currently not implemented
    /// @see Config
    unsigned char GetRoadCurvature() { return m_curvature;  }

private:
    // deleted special members
    GatingWrapper(const GatingWrapper& other) = delete;
    GatingWrapper operator=(const GatingWrapper& other) = delete;

    context_t* m_context;
    const ros::NodeHandle& m_nh;
    int m_isSlave;
	dynamic_reconfigure::ReconfigureRequest m_srvReq;
	dynamic_reconfigure::ReconfigureResponse m_srvResp;


    //// callback functions
    // illimnation mode
    unsigned char m_illumMode;
    static void IlluminationModeCallback(unsigned char data, void* param);

    // illumination power
    unsigned char m_illumPower;
    static void IlluminationPowerCallback(unsigned char data, void* param);

    // illumination tlaser
    unsigned char m_illumTlaser;
    static void IlluminationTlaserCallback(unsigned char data, void* param);

    // illumination range
    unsigned short m_rangeStart;
    unsigned short m_rangeEnd;
    static void IlluminationRangeCallback(unsigned short start, unsigned short end, void* param);

    // road curvature
    unsigned char m_curvature;
    static void CurvatureCallback(unsigned char data, void* param);
};

#endif //__GATING_WRAPPER_H__
