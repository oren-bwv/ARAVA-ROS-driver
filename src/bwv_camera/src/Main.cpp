#include <ros/ros.h>        // ROS
#include <iostream>         // std::cout, std::cerr
#include <new>              // std::bad_alloc
#include <memory>			// std::make_shared,

#include "bwv_api.h"		//
#include "Main.h"           // Main
#include "VideoWrapper.h"   // VideoWrapper
#include "GatingWrapper.h"  // GatingWrapper
#include "utils.h"			// Sleep, CreateThread
#include "bwv_types.h"		// g_metadataTimer

uint64_t g_metadataTimer;

Main::Main(context_t* context, const ros::NodeHandle& nh):
    m_context{context},
    m_nh(nh),
    m_vidMaster(nullptr),
    m_vidSlave(nullptr),
	m_gateMaster(nullptr),
	m_gateSlave(nullptr),
    m_autoHandle(nullptr),
	m_server(nullptr),
    m_autoConnectThread(true),
    m_autoConnect(true)
{
    ROS_INFO("main constructor");

    bwv_api_add_device_discovered_callback(DiscoveryCallback, this);
}

Main::~Main()
{
    ROS_INFO("main destructor");

    m_autoConnectThread = false;
    bwv_api_remove_device_discovered_callback(DiscoveryCallback);

    if (m_vidMaster) {
        delete m_vidMaster;
    }

    if (m_vidSlave) {
        delete m_vidSlave;
    }
}

int Main::StartCamera()
{
	try
	{
		m_context->apiMaster = bwv_api_init();
		m_context->apiSlave = bwv_api_init();

		m_server = std::make_shared<dynamic_reconfigure::
				Server<bwv_camera::BWVCameraConfig>> (m_nh);
	}
	catch (std::bad_alloc& ba)
	{
		ROS_FATAL("bad_alloc caught: %s", ba.what());
		return (-1);
	}

    dynamic_reconfigure::Server<bwv_camera::BWVCameraConfig>::CallbackType f;
    f = boost::bind(&Main::ConfigCallback, this, _1, _2);
    m_server->setCallback(f);

    bwv_api_gating_add_callback_alert(m_context->apiMaster, AlertCallback, this);

    bwv_api_start_device_discovery();

    m_autoHandle = CreateThread(reinterpret_cast<void*>(AutoConnectThread),
         this, nullptr);

    if (nullptr == m_autoHandle) {
    	return (-2);
    }


    return (0);
}

void Main::DiscoveryCallback(device_t device, void* param)
{
    Main* mainPtr = static_cast<Main*>(param);

    ROS_INFO("discovery callback");

    if (device.ctype == 3)
    {
        ROS_INFO("Found master ");
        mainPtr->m_context->devMaster = device;
    }

    if (device.ctype == 4)
    {
        mainPtr->m_context->devSlave = device;
    }
}



std::vector<uint64_t> g_latency(500, 0);
unsigned long g_latencySum;
unsigned int g_latencyIdx;
bool g_nextFlag = false;

static void TestLatency(context_t* context)
{
	ROS_INFO("start latency test");
	unsigned char power = 1;
	Sleep(1000);

	g_nextFlag= true;

	while (g_latencyIdx < 120)
	{
		if (g_nextFlag)
		{
			g_nextFlag = false;
			g_latency[g_latencyIdx] = static_cast<uint64_t>(bwv_api_get_timestamp());

			bwv_api_gating_write_illumination_power(context->apiMaster, static_cast<unsigned char> (power));
			++power;
		}
	}

	printf("mean latency: %lu milliseconds\n", g_latencySum / g_latencyIdx);
}

void Main::AutoConnectThread(void* param)
{
    Main* main = static_cast<Main*>(param);
    ROS_INFO("searching devices");

    while (main->m_autoConnectThread)
    {
        if (!main->m_autoConnect)
        {
            Sleep(1000);
            continue;
        }

        if (bwv_api_is_connected(main->m_context->apiMaster) &&
            bwv_api_is_connected(main->m_context->apiSlave))
        {
//            main->m_autoConnect = false;
            Sleep(1000);
            continue;
        }

        if (main->m_context->devMaster.type == CT_UKNOWN )
        {
            ROS_INFO("searching master ");
            Sleep(1000);
        }
        else if (bwv_api_is_connected(main->m_context->apiMaster))
        {

        }
        else if (bwv_api_connect(main->m_context->apiMaster, main->m_context->devMaster))
        {
            ROS_INFO("Connected to %s, serial: %u.%u.%u.%u",
            main->m_context->devMaster.name,
            (uint8_t)(main->m_context->devMaster.addr.ethernet.ip >> 24) & 0xff,
            (uint8_t)(main->m_context->devMaster.addr.ethernet.ip >> 16) & 0xff,
            (uint8_t)(main->m_context->devMaster.addr.ethernet.ip >>  8) & 0xff,
            (uint8_t)(main->m_context->devMaster.addr.ethernet.ip)       & 0xff);

            main->m_vidMaster = new VideoWrapper(main->m_context, main->m_nh, MASTER);
            main->m_gateMaster = new GatingWrapper(main->m_context, main->m_nh, MASTER);

            Sleep(3000);

//            TestLatency(main->m_context);
        }
        else
        {
            ROS_INFO("not connected to master");
        }

        if (main->m_context->devSlave.type == CT_UKNOWN)
        {
            //ROS_INFO("searching slave");
            Sleep(1000);
            continue;
        }
        else if (bwv_api_is_connected(main->m_context->apiSlave))
        {

        }
        else if (bwv_api_connect(main->m_context->apiSlave, main->m_context->devSlave))
        {
            ROS_INFO("Conneced to %s, serial: %u.%u.%u.%u",
            main->m_context->devSlave.name,
            (uint8_t)(main->m_context->devSlave.addr.ethernet.ip >> 24) & 0xff,
            (uint8_t)(main->m_context->devSlave.addr.ethernet.ip >> 16) & 0xff,
            (uint8_t)(main->m_context->devSlave.addr.ethernet.ip >>  8) & 0xff,
            (uint8_t)(main->m_context->devSlave.addr.ethernet.ip)       & 0xff);

            main->m_vidSlave = new VideoWrapper(main->m_context, main->m_nh, SLAVE);
            main->m_gateSlave = new GatingWrapper(main->m_context, main->m_nh, SLAVE);
        }
        else
        {
            ROS_INFO("not connected to slave");
        }
    }
}

void Main::ConfigCallback(bwv_camera::BWVCameraConfig &config, uint32_t level)
{
//	ROS_INFO("Reconfigure Request:\n"
//			"\tCameraWorkingMode = %d"
//			"\tIlluminationPower = %d"
//			"\tRoadCurvature = %f"
//			"\tTlaser = %f"
//			"\tmin_range = %d"
//			"\tmax_range = %d"
//			"\tlevel = %d ",
//			config.CameraWorkingMode,
//			config.IlluminationPower,
//			config.RoadCurvature,
//			config.Tlaser,
//			config.min_range,
//			config.max_range,
//			level);

	m_context->config.illuminationPower = config.IlluminationPower;
	m_context->config.tlaser = config.Tlaser;
	m_context->config.minRange = config.min_range;
	m_context->config.maxRange = config.max_range;
	m_context->config.workingMode = config.CameraWorkingMode;
	m_context->config.curvature = config.RoadCurvature;

	if (m_context->apiMaster && GetGatingMaster())
    {
		switch (level)
		{
		case 0:
//			ROS_INFO("default level callback");
			bwv_api_gating_write_illumination_wroking_mode(m_context->apiMaster, static_cast<unsigned char> (config.CameraWorkingMode));
			bwv_api_gating_write_illumination_power(m_context->apiMaster, static_cast<unsigned char> (config.IlluminationPower));
			bwv_api_gating_write_illumination_tlaser(m_context->apiMaster, static_cast<unsigned char> (config.Tlaser * 100));
			bwv_api_gating_write_curvature(m_context->apiMaster, static_cast<unsigned char> (config.RoadCurvature * 100));

			bwv_api_gating_write_illumination_range(m_context->apiMaster,
									static_cast<unsigned short> (config.min_range),
									static_cast<unsigned short>(config.max_range));
			break;

		case 1:
//			ROS_INFO("camera working mode change request");
//			if (GetGatingMaster()->GetIllumMode() != static_cast<unsigned char> (config.CameraWorkingMode))
			{
				bwv_api_gating_write_illumination_wroking_mode(m_context->apiMaster, static_cast<unsigned char> (config.CameraWorkingMode));
				bwv_api_gating_read_illumination_working_mode(m_context->apiMaster);
			}
			break;

		case 2:
//			ROS_INFO("power change request");
//			if (GetGatingMaster()->GetIllumPower() != static_cast<unsigned char> (config.IlluminationPower))
			{
				bwv_api_gating_write_illumination_power(m_context->apiMaster, static_cast<unsigned char> (config.IlluminationPower));
//				bwv_api_gating_read_illumination_power(m_context->apiMaster);
			}
			break;

		case 3:
			//ROS_INFO("tlaser change request");
//			if (GetGatingMaster()->GetIllumTlaser() != static_cast<unsigned char> (config.Tlaser * 100))
			{
				bwv_api_gating_write_illumination_tlaser(m_context->apiMaster, static_cast<unsigned char> (config.Tlaser * 100));
//				bwv_api_gating_read_illumination_tlaser(m_context->apiMaster);
			}
			break;

		case 4:
			//ROS_INFO("road curve change request");
//			if (GetGatingMaster()->GetRoadCurvature() != static_cast<unsigned char> (config.RoadCurvature * 100))
			{
				bwv_api_gating_write_curvature(m_context->apiMaster, static_cast<unsigned char> (config.RoadCurvature * 100));
//				bwv_api_gating_read_curvature(m_context->apiMaster);
			}
			break;

		case 5:
			ROS_INFO("range change request");
			if (GetGatingMaster()->GetRangeStart() != static_cast<unsigned short> (config.min_range) ||
					GetGatingMaster()->GetRangeEnd() != static_cast<unsigned short> (config.max_range))
			{
				bwv_api_gating_write_illumination_range(m_context->apiMaster,
						static_cast<unsigned short> (config.min_range),
						static_cast<unsigned short>(config.max_range));
				bwv_api_gating_read_illumination_range(m_context->apiMaster);
			}
			break;


		default:
			ROS_INFO("the level received: %d", level);
			break;
		}

		bwv_api_gating_read_alert(m_context->apiMaster);
    }

    g_metadataTimer = bwv_api_get_timestamp();

//    if (g_context.apiSlave && driver->GetGatingSlave())
//    {
//		switch (level)
//		{
//		case 0:
////			ROS_INFO("range change request");
//			if (driver->GetGatingSlave()->GetRangeStart() != static_cast<unsigned short> (config.min_range) ||
//					driver->GetGatingSlave()->GetRangeEnd() != static_cast<unsigned short> (config.max_range))
//			{
//				bwv_api_gating_write_illumination_range(g_context.apiSlave,
//						static_cast<unsigned short> (config.min_range),
//						static_cast<unsigned short>(config.max_range));
//				bwv_api_gating_read_illumination_range(g_context.apiSlave);
//			}
//			break;
//
//		case 1:
////			ROS_INFO("cameara working mode change request");
//			if (driver->GetGatingSlave()->GetIllumMode() != static_cast<unsigned char> (config.CameraWorkingMode))
//			{
//				bwv_api_gating_write_illumination_wroking_mode(g_context.apiSlave, static_cast<unsigned char> (config.CameraWorkingMode));
//				bwv_api_gating_read_illumination_working_mode(g_context.apiSlave);
//			}
//			break;
//
//		case 2:
////			ROS_INFO("power change request");
//			if (driver->GetGatingSlave()->GetIllumPower() != static_cast<unsigned char> (config.IlluminationPower))
//			{
//				bwv_api_gating_write_illumination_power(g_context.apiSlave, static_cast<unsigned char> (config.IlluminationPower));
//				bwv_api_gating_read_illumination_power(g_context.apiSlave);
//			}
//			break;
//
//		case 3:
//			//ROS_INFO("tlaser change request");
//			if (driver->GetGatingSlave()->GetIllumTlaser() != static_cast<unsigned char> (config.Tlaser * 100))
//			{
//				bwv_api_gating_write_illumination_tlaser(g_context.apiSlave, static_cast<unsigned char> (config.Tlaser * 100));
//				bwv_api_gating_read_illumination_tlaser(g_context.apiSlave);
//			}
//			break;
//
//		case 4:
//			//ROS_INFO("road curve change request");
//			if (driver->GetGatingSlave()->GetRoadCurvature() != static_cast<unsigned char> (config.RoadCurvature * 100))
//			{
//				bwv_api_gating_write_curvature(g_context.apiSlave, static_cast<unsigned char> (config.RoadCurvature * 100));
//				bwv_api_gating_read_curvature(g_context.apiSlave);
//			}
//			break;
//
//		default:
//			ROS_INFO("the level received: %d", level);
//			break;
//		}
//    }
}

void Main::DisconnectCallback(device_t device, void *param)
{
    Main* main = static_cast<Main*>(param);

    //main->m_autoConnect = true;
}

void Main::AlertCallback(gating_data_t data, void* param)
{
    Main* main = static_cast<Main*>(param);

    if ((data.alert & 0x1) == 0x1)
    {
		ROS_ERROR("   Gating overpower   ");
	}
	if ((data.alert & 0x2) == 0x2) {
		ROS_ERROR("   Laser overheat   ");
	}
}

