#ifndef BWV_API_H
#define BWV_API_H

#include <stdbool.h>
#include <stdint.h>

#if defined(_WIN32) || defined(_WIN64)
	#define _PWIN
	#define __MINGW_EXTENSION
	
	#ifdef __GNUC__
      		#define DLL_PUBLIC __attribute__ ((dllexport))
	#else
      		#define DLL_PUBLIC __declspec(dllexport)
	#endif	
   	#define FTYPE __cdecl
	#define DLL_LOCAL
	#define _PACKED_
	#define	PACK_START __pragma(pack(push, 1))
	#define PACK_END __pragma(pack(pop))

#else
	#define _PLIN
	#if __GNUC__ >= 4
    		#define DLL_PUBLIC __attribute__ ((visibility ("default")))
    		#define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  	#else
    		#define DLL_PUBLIC
		#define DLL_LOCAL
	#endif
	#define FTYPE	
	#define _PACKED_  __attribute__((__packed__))
	#define	PACK_START
	#define PACK_END
#endif

#ifdef __cplusplus
extern "C" {
#endif


//     _____                                                 
//    / ____|                                                
//   | |        ___    _ __ ___    _ __ ___     ___    _ __  
//   | |       / _ \  | '_ ` _ \  | '_ ` _ \   / _ \  | '_ \ 
//   | |____  | (_) | | | | | | | | | | | | | | (_) | | | | |
//    \_____|  \___/  |_| |_| |_| |_| |_| |_|  \___/  |_| |_|
//                                                           
//                                                           

#define API_OK 0
#define API_ERR -1

/**
 *
 * @defgroup connection Connection module
 *
 * @brief Connectivity functions
 * 
 * This module provides connectivity routines such as device discovery, device connection/disconnection handling
 *  
 * Usage sequence (in case only one system is connected to PC):
 *     1. Start device discovery by calling to *api_start_device_discovery*
 *     2. Wait till there is at least 1 device (*api_get_device_count* > 0)
 *     3. Connect to system by calling *api_connect* (0, &device_info)
 * 
 * 
 * @{
 */
		
/** 
* 
* @struct connection_type_t
* @brief HW connection types
*/

typedef enum : unsigned char{
	CT_UKNOWN			= 0,		/**< 0x00 unknown type*/	
	CT_SERIAL_CTRL_BOX	= 1,		/**< 0x01 Connection to Control Box through serial port*/	
	CT_SERIAL_COMM_BOX  = 2,		/**< 0x02 Connection to Communication Box through serial port*/	
	CT_ETHERNET			= 3,		/**< 0x03 Connection to Ethernet Camera unit*/	
	CT_END				= 255		
} connection_type_t;

PACK_START

/**
* 
* @struct serial_addr_t
* @brief Serial port description
*/
typedef struct _PACKED_ {
	char addr[16];					/**< Serial port path (E.g. COM3, /dev/ttyUSB0, /dev/ttyACM0, etc... )*/
	unsigned short location[16];	/**< Serial port device location (VID/PID or USB path)*/
} serial_addr_t;					

/** 
* 
* @struct ethernet_addr_t
* @brief Serial port description
*/
typedef struct _PACKED_ {
	unsigned long ip;				/**< IP address of Camera unit*/
	unsigned long link_speed;		/**< Current link layer speed */
	unsigned short rx_port;			/**< Number of Camera unit receiving port*/
	unsigned short tx_port;			/**< Number of Camera unit transmitting port*/
	unsigned short fwu_port;		/**< Number of Camera unit FW update port*/
	unsigned short rtp_port;		/**< Number of Camera unit RTP broadcast port*/
	unsigned long local_ip;			/**< PC's local IP*/
	unsigned char pad[16];			/**< reserved*/
} ethernet_addr_t;

/**
* 
 *  @struct connection_t
 *  @brief Serial port description
*/
typedef struct _PACKED_ {
	connection_type_t type;			/**< Device connection type*/
	unsigned char	ctype;			/**< reserved*/
	char name[64];					/**< Displayable device name*/
	union {				
		serial_addr_t serial;		/**< Serial port description*/
		ethernet_addr_t ethernet;	/**< Ethernet connection description*/
	} addr;
	uint64_t time;					/**< Discovery time*/
} device_t;
PACK_END

/**
*
 * @brief API callback which is called when Device was discovered by API
 *
 * @param device Device connection description 
 * @param param User data
 */
typedef void (*device_discovered_cb_t)(device_t device, void* param);

DLL_PUBLIC int bwv_api_add_device_discovered_callback(device_discovered_cb_t callback, void* param);

DLL_PUBLIC int bwv_api_remove_device_discovered_callback(device_discovered_cb_t callback);

/**
*
* @brief Start device discovery
*
*/
DLL_PUBLIC void bwv_api_start_device_discovery();

/**
*
* @brief Stop device discovery.
*/
DLL_PUBLIC void bwv_api_stop_device_discovery();

/**
* @brief Retrieve number of devices discovered so far
* @return number of devices
*/
DLL_PUBLIC int bwv_api_get_device_count();

/**
* @brief Get device connection description from internal list of devices that were discovered
* @param index 0-based array index
* @param device preallocated buffer for storing device connection description
* @return 0 on success, -1 if index is out of bounds
*/
DLL_PUBLIC int bwv_api_get_device(int index, device_t* device);

DLL_PUBLIC int bwv_api_is_device_discovered(device_t device);


DLL_PUBLIC void* bwv_api_init();

DLL_PUBLIC void bwv_api_deinit(void* api);

/**
* 
 * @brief API callback which is called when API is connected to system
 *
 * @param param User data 
 */
typedef void (*connected_cb_t)(void* param);

/**
* 
 * @brief API callback which is called when API was disconnected
 *
 * @param param User data
 */
typedef void (*disconnected_cb_t)(device_t device, void *param);

/**
* 
 * @brief Register callback for API connected state
 * @param callback Callback function
 * @param param User data
 * @return internal callback index or -1 if failed
 */

DLL_PUBLIC int bwv_api_add_connect_callback(void* api, connected_cb_t callback, void* param);

/**
* 
 * @brief Register callback for API disconnected state
 * @param callback Callback function
 * @param param User data
 * @return internal callback index or -1 if failed
 */
DLL_PUBLIC int bwv_api_add_disconnect_callback(void* api, disconnected_cb_t callback, void* param);


/**
* 
 * @brief Register callback for API disconnected state
 * @param index Internal callback index received from api_add_disconnect_callback
 *
 */
DLL_PUBLIC void bwv_api_remove_disconnect_callback(void* api, int index);


/** 
* @brief Connect to selected device
* @param device 0-based array index
* @param info preallocated buffer for storing device connection information
* @return 1 on success, 0 on failure
*/
DLL_PUBLIC bool bwv_api_connect(void* api, device_t device);

/** 
* @brief Disconnect from device
*/
DLL_PUBLIC void bwv_api_disconnect(void* api);

/** 
* @brief Check if API is connected.
* @return 1 when connected, 0 otherwise
*/

DLL_PUBLIC bool bwv_api_is_connected(void* api);

DLL_PUBLIC device_t bwv_api_get_connected_device(void* api);

/** @}*/


//   __      __                      _                 
//   \ \    / /                     (_)                
//    \ \  / /    ___   _ __   ___   _    ___    _ __  
//     \ \/ /    / _ \ | '__| / __| | |  / _ \  | '_ \ 
//      \  /    |  __/ | |    \__ \ | | | (_) | | | | |
//       \/      \___| |_|    |___/ |_|  \___/  |_| |_|
//                                                     
//        

/**
 *
 **************************************************************
 * \defgroup version Version module
 *
 * @brief Version information functions
 *
 * This module provides functions for reading firmware version information of different system units
 *
 *
 * @{
 */


PACK_START

/**
*
 *  @struct version_data_t
 *  @brief Version information
*/
typedef struct _PACKED_ {
	unsigned char hw_code;
	unsigned char hw_day;
	unsigned char hw_mon;
	unsigned char hw_year;
	unsigned char hw_hour;
	unsigned char hw_min;
	unsigned char hw_sec;
	unsigned char sw_code;
	unsigned char sw_day;
	unsigned char sw_mon;
	unsigned char sw_year;
	unsigned char sw_hour;
	unsigned char sw_min;
	unsigned char sw_sec;
	unsigned int sensor_ver;
	unsigned int board_sn;
	unsigned short number;
} version_data_t;
PACK_END

/**
* 
* @brief Version information service
* @param data Version information structure
* @param param User data
*
*/

typedef void (*version_callback_t)(version_data_t data, void* param);

/**
* @brief Add API callback for Camera unit version
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_version_add_callback_cu(void* api, version_callback_t callback, void* param);

/**
* @brief Add API callback for Control Box/Communication Box version
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_version_add_callback_cb(void* api, version_callback_t callback, void *param);

/**
* @brief Add API callback for FX3 USB peripheral IC 
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_version_add_callback_fx3(void* api, version_callback_t callback, void *param);

/**
* @brief Get API version
* @return Version information
*/
DLL_PUBLIC version_data_t bwv_api_get_version(void* api);

/**
* 
* @brief Read Camera unit version
*
*/

DLL_PUBLIC void bwv_api_version_get_cu(void* api);

/**
*
* @brief Read Control Box/Communication Box version
*
*/
DLL_PUBLIC void bwv_api_version_get_cb(void* api);

/**
*
* @brief Read FX3 USB peripheral IC version
*
*/
DLL_PUBLIC void bwv_api_version_get_fx3(void* api);

/** @}*/

//     _____           _     _                 
//    / ____|         | |   (_)                
//   | |  __    __ _  | |_   _   _ __     __ _ 
//   | | |_ |  / _` | | __| | | | '_ \   / _` |
//   | |__| | | (_| | | |_  | | | | | | | (_| |
//    \_____|  \__,_|  \__| |_| |_| |_|  \__, |
//                                        __/ |
//                                       |___/ 

/**
 * 
 **************************************************************
 * \defgroup gating Gating module
 *
 * @brief Gating timings control functions
 * 
 * This module provides functions for working with Camera unit gating parameters
 * 
 *
 * @{
 */

#define GATING_OVERPOWER_NONE           	0	/**< No laser override*/
#define GATING_OVERPOWER_NORMAL         	1	/**< Normal laser override*/
#define GATING_OVERPOWER_LOW            	2	/**< Low level laser override*/
#define GATING_OVERPOWER_HIGH           	3	/**< High level laser override*/
 


PACK_START



/**
 *  @brief Holds all gating parameters.
*/
typedef struct _PACKED_ {
	unsigned char type;					/**< Frame type*/
	unsigned char series;              	/**< Frame type series*/
	unsigned char function[4];         	/**< Current function*/
	unsigned char laser_en;            	/**< Laser status. 0x00 - disabled, 0x01 - enabled*/
	unsigned char series_en;           	/**< Series status. 0x00 - disabled, 0x01 - enabled*/
	unsigned short passive_pulses;     	/**< Passive pulses*/
	unsigned short active_pulses;      	/**< Active pulses*/
	double tlaser;						/**< tLaser time*/
	double tclose;						/**< tClose time*/
	double tgate;						/**< tGate time*/
	double tcycle;						/**< tCycle time*/
	double tx1_pulse_duration;  		/**< TX1 pulse duration*/
	double tx1_tx2_delta;       		/**< TX1-TX2 delta time*/
	double tlaser_to_tlight;    		/**< Laser to Light time*/
	unsigned char overpower;           	/**< Laser override status*/
	unsigned int alert;               	/**< Alert code*/
} gating_data_t;


/**
 *  @brief Holds Subgating series parameters.
*/
typedef struct _PACKED_ {
	unsigned char series_laser_en;      /**< Laser status. 0 - disabled, 1 - enabled*/
	unsigned short passive_pulses;     	/**< Passive pulses*/
	unsigned short active_pulses;      	/**< Active pulses*/
	double tlaser_start;				/**< tLaser time*/
	double tlaser_end;					/**< tLaser time*/
	double tgate_start;					/**< tClose time*/
	double tgate_end;					/**< tGate time*/
	double tcycle;						/**< tCycle time*/
} subgating_series_t;


/**
 *  @struct subgating_table_t
 *  @brief Holds Subgating table parameters.
*/
typedef struct _PACKED_ {
	unsigned char type;					/**< Frame type*/
	unsigned char table;              	/**< Frame type series*/
	subgating_series_t series[4];
} subgating_table_t;

/**
 *  @struct subgating_burn_t
 *  @brief Holds general Subgating parameters.
*/
typedef struct _PACKED_ {
	unsigned char agc_type;				/**< On which type ACS will work*/
	unsigned char agc_subtype;			/**< On which subtype ACS will work*/
	unsigned short agc_enable;			/**< Enable ACS*/
	unsigned char first;				/**< First table*/
	unsigned char last;					/**< Last table*/
} subgating_burn_t;

PACK_END


/**
 * @brief API callback signature for gating related commands
 *
 * @param data Gating data structure. Only relevant fields are filled when callback arrives.
 * @param param User data
 */

typedef void (*gating_callback_t)(gating_data_t data, void* param);


/**
 * @brief API callback signature for subgating read table command
 *
 * @param data Subgating table. 
 * @param param User data
 */
typedef void (*subgating_table_callback_t)(subgating_table_t data, void* param);
/**
 * @brief API callback signature for subgating burn table command
 *
 * @param data General subgating data.
 * @param param User data
 */
typedef void (*subgating_burn_callback_t)(subgating_burn_t data, void* param);

/**
 * @brief API callback signature for subgating read parameters command
 *
 * @param first First (or start) subgating table. 
 * @param last Last (or end) subgating table. 
 * @param param User data
 */
typedef void (*subgating_params_callback_t)(unsigned char first, unsigned char last, void* param);

/**
 * @brief API callback signature for subgating read status command
 *
 * @param data Subgating status. 0 - disabled, 1 - enabled
 * @param param User data
 */
typedef void (*subgating_en_callback_t)(unsigned char data, void* param);


/**
 * @brief API callback signature for illumination mode read command
 *
 * @param data Illumination mode
 * @param param User data
 */
typedef void (*illumination_mode_callback_t)(unsigned char data, void* param);
typedef void (*illumination_power_callback_t)(unsigned char data, void* param);
typedef void (*illumination_tlaser_callback_t)(unsigned char data, void* param);
typedef void (*illumination_range_callback_t)(unsigned short start, unsigned short end, void* param);
typedef void (*curvature_callback_t)(unsigned char data, void* param);

/**
 *  @struct temperature_data_t
 *  @brief Holds general Subgating parameters.
*/
typedef struct _PACKED_ {
	char sens_temp;				/**< Sensor temperature*/
	char xadc_temp;				/**< XADC temperature*/
	char sens_thr_max;			/**< Sensor max threshold*/
	char sens_thr_min;			/**< Sensor min threshold*/
	char xadc_thr_max;			/**< XADC max threshold*/
	char xadc_thr_min;			/**< XADC min threshold*/
	unsigned char status;		/**< Status*/
} temperature_data_t;

/**
 * @brief API callback signature for reading sensor/XADC temperature from Camera unit
 *
 * @param data Temperature information
 * @param param User data
 */
typedef void (*temperature_callback_t)(temperature_data_t data, void* param);

/**@brief Add API callback for read gating data command (gating_read_data)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */

DLL_PUBLIC int bwv_api_gating_add_callback_data(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read gating type function (gating_read_type_function)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_type_function(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read system function (gating_read_function)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_function(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read TX1 pulse duration (gating_read_tx1_pulse_duration)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_tx1_pulse_duration(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read Tlaser to Tlight (gating_read_tlaser_to_tlight)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_tlaser_to_tlight(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read TX1 to TX2 time delta (gating_read_tx1_tx2_delta)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_tx1_tx2_delta(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read overpower value (gating_read_overpower)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_overpower(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for retrieve CU alert value (gating_read_alert)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_alert(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read active types (gating_read_active_types)
 *
 * @param callback Callback function. Note - only relevant fields of callback data structure are filled when callback arrives.
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_active_types(void* api, gating_callback_t callback, void* param);

/**@brief Add API callback for read temperature (gating_read_temperature)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_temperature(void* api, temperature_callback_t callback, void* param);


/**@brief Add API callback for read Subgating table (gating_sub_read_table)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_sub_table(void* api, subgating_table_callback_t callback, void* param);

/**@brief Add API callback for read Subgating table parameters (gating_sub_burn_get)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_sub_burn(void* api, subgating_burn_callback_t callback, void* param);

/**@brief Add API callback for read Subgating parameters (gating_sub_read_params)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_sub_params(void* api, subgating_params_callback_t callback, void* param);

/**@brief Add API callback for read Subgating status (gating_sub_read_status)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_gating_add_callback_sub_en(void* api, subgating_en_callback_t callback, void* param);

/**@brief Read gating data
 *
 * @param data Gating data structure.
 *
 * type and series members must be provided
 */
DLL_PUBLIC void bwv_api_gating_read_data(void* api, gating_data_t data);

/**@brief Read type function
 *
 * @param data Gating data structure
 *
 * type member must be provided
 */
DLL_PUBLIC void bwv_api_gating_read_type_function(void* api, gating_data_t data);

/**@brief Read system function
 *
 *
 */
DLL_PUBLIC void bwv_api_gating_read_function(void* api);

/**@brief Read TX1 pulse duration
 *
 */
DLL_PUBLIC void bwv_api_gating_read_tx1_pulse_duration(void* api);

/**@brief Read TX1 to TX2 time delta
 *
 */
DLL_PUBLIC void bwv_api_gating_read_tx1_tx2_delta(void* api);

/**@brief Read Tlaser to Tlight
 *
 */
DLL_PUBLIC void bwv_api_gating_read_tlaser_to_tlight(void* api);

/**@brief Read overpower
 *
 */
DLL_PUBLIC void bwv_api_gating_read_overpower(void* api);

/**@brief Read CU alert
 *
 */
DLL_PUBLIC void bwv_api_gating_read_alert(void* api);

/**@brief Read active types
 *
 */
DLL_PUBLIC void bwv_api_gating_read_active_types(void* api);

/**@brief Read CAmera unit temperature
 *
 */
DLL_PUBLIC void bwv_api_gating_read_temperature(void* api);

/**@brief Read Subgating table
 *
 * @param type Type for which to read table
 * @param table Table number to read
 */
DLL_PUBLIC void bwv_api_gating_sub_read_table(void* api, unsigned char type, unsigned char table);

/**@brief Read Subgating table parameters
 *
 */
DLL_PUBLIC void bwv_api_gating_sub_burn_get(void* api);

/**@brief Read Subgating parameters
 *
 */
DLL_PUBLIC void bwv_api_gating_sub_read_params(void* api);

/**@brief Read Subgating status
 *
 */

DLL_PUBLIC void bwv_api_gating_sub_read_status(void* api);


/**@brief Write gating parameters *0540/
 * @param data structure with all relevant members filled
 * 
 * type and series members are mandatory
 *
 * Note - data.tx1_pulse_duration, data.tx1_tx2_delta and data.tlaser_to_tlight should also be filled by values previously obtained from CU
 */
DLL_PUBLIC void bwv_api_gating_write_data(void* api, gating_data_t data);

/**@brief Increment current pulses value for specific type
 *
 * @param type Type number
 *
 */
DLL_PUBLIC void bwv_api_increment_pulses_for_type(void* api, uint8_t type);

/**@brief Decrement current pulses value for specific type
 *
 * @param type Type number
 *
 */
DLL_PUBLIC void bwv_api_decrement_pulses_for_type(void* api, uint8_t type);

DLL_PUBLIC void bwv_api_gating_toggle_laser_and_series(void* api, gating_data_t data);

/**@brief Write type function
 *
 * @param data structure with all relevant members filled
 *
 * type member are mandatory
 */
DLL_PUBLIC void bwv_api_gating_write_type_function(void* api, gating_data_t data);

/**@brief Write system function
 *
 * @param data structure with all relevant members filled
 *
 */
DLL_PUBLIC void bwv_api_gating_write_function(void* api, gating_data_t data);

/**@brief Write TX1 pulse duration
 *
 * @param data structure with all relevant members filled
 * 
 */
DLL_PUBLIC void bwv_api_gating_write_tx1_pulse_duration(void* api, gating_data_t data);

/**@brief Write TX1 to TX2 time delta
 *
 * @param data structure with all relevant members filled
 *
 */
DLL_PUBLIC void bwv_api_gating_write_tx1_tx2_delta(void* api, gating_data_t data);

/**@brief Write Tlaser to Tlight
 *
 * @param data structure with all relevant members filled
 *
 */
DLL_PUBLIC void bwv_api_gating_write_tlaser_to_tlight(void* api, gating_data_t data);

/**@brief Write overpower
 *
 * @param data structure with all relevant members filled
 * @param code protection code. (Should be requested from BWV)
 *
 */
DLL_PUBLIC void bwv_api_gating_write_overpower(void* api, gating_data_t data, unsigned int code);

DLL_PUBLIC void bwv_api_gating_write_temperature(void* api, temperature_data_t data);
DLL_PUBLIC void bwv_api_gating_time_to_distance(double tlaser, double tclose, double tgate, uint16_t* rmin, uint16_t* rmax);
DLL_PUBLIC void bwv_api_gating_distance_to_time(uint16_t rmin, uint16_t rmax, double* tlaser, double* tclose, double* tgate);

/**@brief Write Subgating table
 *
 * @param data structure with all relevant members filled
  *
 */
DLL_PUBLIC void bwv_api_gating_sub_write_table(void* api, subgating_table_t data);

/**@brief Write Subgating table parameters
 *
 * @param data structure with all relevant members filled
  *
 */
DLL_PUBLIC void bwv_api_gating_sub_burn(void* api, subgating_burn_t data);

/**@brief Write Subgating parameters
 *
 * @param first First (or start) subgating table
 * @param last First (or end) subgating table
  *
 */
DLL_PUBLIC void bwv_api_gating_sub_write_params(void* api, unsigned char first, unsigned char last);

/**@brief Start subgating mode
 *
 */
DLL_PUBLIC void bwv_api_gating_sub_start(void* api);

/**@brief Stop subgating mode
 *
 */
DLL_PUBLIC void bwv_api_gating_sub_stop(void* api);

DLL_PUBLIC int bwv_api_gating_add_callback_illumination_working_mode(void* api, illumination_mode_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_gating_read_illumination_working_mode(void* api);
DLL_PUBLIC void bwv_api_gating_write_illumination_wroking_mode(void* api, unsigned char mode);

DLL_PUBLIC int bwv_api_gating_add_callback_illumination_range(void* api, illumination_range_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_gating_read_illumination_range(void* api);
DLL_PUBLIC void bwv_api_gating_write_illumination_range(void* api, unsigned short start, unsigned short end);

DLL_PUBLIC int bwv_api_gating_add_callback_illumination_power(void* api, illumination_power_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_gating_read_illumination_power(void* api);
DLL_PUBLIC void bwv_api_gating_write_illumination_power(void* api, unsigned char value);

DLL_PUBLIC int bwv_api_gating_add_callback_illumination_tlaser(void* api, illumination_tlaser_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_gating_read_illumination_tlaser(void* api);
DLL_PUBLIC void bwv_api_gating_write_illumination_tlaser(void* api, unsigned char value);

DLL_PUBLIC int bwv_api_gating_add_callback_curvature(void* api, curvature_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_gating_read_curvature(void* api);
DLL_PUBLIC void bwv_api_gating_write_curvature(void* api, unsigned char value);

/** @}*/

//   __      __  _       _                     ____            _   
//   \ \    / / (_)     | |                   / __ \          | |  
//    \ \  / /   _    __| |   ___    ___     | |  | |  _   _  | |_ 
//     \ \/ /   | |  / _` |  / _ \  / _ \    | |  | | | | | | | __|
//      \  /    | | | (_| | |  __/ | (_) |   | |__| | | |_| | | |_ 
//       \/     |_|  \__,_|  \___|  \___/     \____/   \__,_|  \__|
//                                                                 
//      

/**
 *
 **************************************************************
 * \defgroup videoout Video Out module
 *
 * @brief Video output control functions
 *
 * This module provides various video output control functions. Part of the related to Camera unit, while others are relevant only for Control Box/Communication Box
 *
 *
 * @{
 */


#define VIDEO_OUT_HDMI_RES_1280x800		0x00
#define VIDEO_OUT_HDMI_RES_800x600		0x01
#define VIDEO_OUT_HDMI_RES_848x480		0x02
#define VIDEO_OUT_HDMI_RES_720x480		0x03
 /**
 *
  *  @struct video_out_data_t
  *  @brief Video output module data structure 
  * 
  * This is the common structure for Video output related commands. However each function uses only relevant members
 */
typedef struct _PACKED_ {
	unsigned char hdmi0_type;			/**< HDMI0 output type*/	
	unsigned char hdmi0_res;			/**< HDMI0 resolution*/	
	unsigned char hdmi1_type;			/**< HDMI1 output type*/	
	unsigned char hdmi1_res;			/**< HDMI1 resolution*/	
	unsigned char av_type;				/**< Analog video type*/	
	unsigned char cl_type0;				/**< CameraLink/USB type 0 enable. 0x00 - disabled, 0x01 - enabled*/
	unsigned char cl_type1;				/**< CameraLink/USB type 1 enable. 0x00 - disabled, 0x01 - enabled*/	
	unsigned char cl_type2;				/**< CameraLink/USB type 2 enable. 0x00 - disabled, 0x01 - enabled*/	
	unsigned char cl_type3;				/**< CameraLink/USB type 3 enable. 0x00 - disabled, 0x01 - enabled*/	
	unsigned char cl_type4;				/**< CameraLink/USB type Subtract enable. 0x00 - disabled, 0x01 - enabled*/	
	unsigned char cl_type5;				/**< CameraLink/USB type CLAHE enable. 0x00 - disabled, 0x01 - enabled*/	
	unsigned char cl8bit;				/**< Image pixel format. 0x0 - 10BPP, 0x1 - 8BPP*/	
	unsigned char cl_type_mask;			/**< CameraLink/USB enabled types mask. Bit per type*/	
	unsigned char contrast_type0;		/**< Contrast type 0 enable.  0x00 - disabled, 0x01 - enabled*/	
	unsigned char contrast_type1;		/**< Contrast type 1 enable.  0x00 - disabled, 0x01 - enabled*/	
	unsigned char contrast_type2;		/**< Contrast type 2 enable.  0x00 - disabled, 0x01 - enabled*/	
	unsigned char contrast_type3;		/**< Contrast type 3 enable.  0x00 - disabled, 0x01 - enabled*/	
	unsigned char contrast_type4;		/**< Contrast type 4 enable.  0x00 - disabled, 0x01 - enabled*/	
	unsigned char clahe_type;			/**< CLAHE input type*/	
	unsigned char tp_cb;				/**< Control Box/Communication Box test pattern selection*/	
	unsigned char tp_cu;				/**< Camera unit test pattern selection*/	
	unsigned char tp_s;					/**< Sensor test pattern selection*/
	unsigned short contrast_low;		/**< Contrast low value*/	
	unsigned short contrast_high;		/**< Contrast high value*/	
	unsigned char subtract_dir;			/**< Subtract direction*/	
	unsigned char system_fps;			/**< Deprecated*/	
	unsigned char full_image;			/**< Deprecated*/	
	unsigned char md_status;			/**< Deprecated*/	
} video_out_data_t;


/**@brief API video output module callback signature
 *
 * @param data Video output module data structure 
 * @param param User data
 *
 */
typedef void (*video_out_callback_t)(video_out_data_t data, void *param);

/**@brief Add API callback for read read video output state from Control Box/Communication Box (video_out_read_state_cb)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_state_cb(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read read video output state from Camera unit (video_out_read_state_cu)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_state_cu(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read enabled contrast types(video_out_read_contrast_types)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_contrast_types(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read enabled CameraLink/USB types (video_out_read_cl_types)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_cl_types(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read CLAHE input type (video_out_read_cl_types)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_clahe_type(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read sensor test pattern (video_out_read_tp_s)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_tp_s(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read Camera unit test pattern (video_out_read_tp_cu)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_tp_cu(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read Control Box/Communication Box test pattern (video_out_read_tp_cb)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_tp_cb(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read contrast low value (video_out_read_contrast_low)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_contrast_low(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read contrast high value (video_out_read_contrast_high)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_contrast_high(void* api, video_out_callback_t callback, void* param);

/**@brief Add API callback for read subtract direction (video_out_read_contrast_high)
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 * 
 * Note - only relevant fields of callback data structure are filled when callback arrives.
 */
DLL_PUBLIC int bwv_api_video_out_add_callback_subtract_dir(void* api, video_out_callback_t callback, void* param);


DLL_PUBLIC int bwv_api_video_out_add_callback_system_fps(void* api, video_out_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_video_out_add_callback_full_image(void* api, video_out_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_video_out_add_callback_md_status(void* api, video_out_callback_t callback, void* param);

/**@brief Remove specific Video output callback
 *
 * @param callback Callback function
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_video_out_remove_callback(void* api, void* callback);

/**
* 
* @brief Read Control Box/Communication Box video output state
*
* This function reads only HDMI0/HDMI1 resolution settings
*/
DLL_PUBLIC void bwv_api_video_out_read_state_cb(void* api);

/**
*
* @brief Read Control Box/Communication Box video output state
*
*/
DLL_PUBLIC void bwv_api_video_out_read_state_cu(void* api);

/**
*
* @brief Read contrast enabled types
*
*/
DLL_PUBLIC void bwv_api_video_out_read_contrast_types(void* api);

/**
*
* @brief Read CameraLink/USB3 enabled types
*
*/
DLL_PUBLIC void bwv_api_video_out_read_cl_types(void* api);

/**
*
* @brief Read CLAHE input type
*
*/
DLL_PUBLIC void bwv_api_video_out_read_clahe_type(void* api);

/**
*
* @brief Read current Control Box/Communication Box test pattern
*
*/
DLL_PUBLIC void bwv_api_video_out_read_tp_cb(void* api);

/**
*
* @brief Read current Camera units test pattern
*
*/
DLL_PUBLIC void bwv_api_video_out_read_tp_cu(void* api);

/**
*
* @brief Read current sensor test pattern
*
*/
DLL_PUBLIC void bwv_api_video_out_read_tp_s(void* api);

/**
*
* @brief Read contrast low value
*
*/
DLL_PUBLIC void bwv_api_video_out_read_contrast_low(void* api);

/**
*
* @brief Read contrast high value
*
*/
DLL_PUBLIC void bwv_api_video_out_read_contrast_high(void* api);

/**
*
* @brief Read subtract direction
*
*/
DLL_PUBLIC void bwv_api_video_out_read_subtract_dir(void* api);

DLL_PUBLIC void bwv_api_video_out_read_full_image(void* api);
DLL_PUBLIC void bwv_api_video_out_read_sys_fps(void* api);
DLL_PUBLIC void bwv_api_video_out_read_md_status(void* api);

/**
*
* @brief Write HDMI0 output type
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_hdmi0_type(void* api, video_out_data_t data);

/**
*
* @brief Write HDMI0 output resolution
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_hdmi0_res(void* api, video_out_data_t data);

/**
*
* @brief Write HDMI1 output type
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_hdmi1_type(void* api, video_out_data_t data);

/**
*
* @brief Write HDMI1 output resolution
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_hdmi1_res(void* api, video_out_data_t data);

/**
*
* @brief Write analog video output type
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_av_type(void* api, video_out_data_t data);

/**
*
* @brief Write CameraLink/USB3 output types
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_cl_types(void* api, video_out_data_t data);

/**
*
* @brief Write contrast types
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_contrast_types(void* api, video_out_data_t data);

/**
*
* @brief Write CLAHE input type
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_clahe_type(void* api, video_out_data_t data);

/**
*
* @brief Write Control Box/Communication Box test pattern
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_tp_cb(void* api, video_out_data_t data);

/**
*
* @brief Write Camera unit test pattern
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_tp_cu(void* api, video_out_data_t data);

/**
*
* @brief Write sensor test pattern
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_tp_s(void* api, video_out_data_t data);

DLL_PUBLIC void bwv_api_video_out_toggle_calibration_rect(void* api, uint8_t mode);

/**
*
* @brief Write contrast low value
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_contrast_low(void* api, video_out_data_t data);

/**
*
* @brief Write contrast high value
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_contrast_high(void* api, video_out_data_t data);

/**
*
* @brief Write subtract direction
*
* @param data Video output data structure with relevant members filled
*/
DLL_PUBLIC void bwv_api_video_out_write_subtract_dir(void* api, video_out_data_t data);

DLL_PUBLIC void bwv_api_video_out_write_sys_fps(void* api, uint8_t fps);
DLL_PUBLIC void bwv_api_video_out_write_full_image(void* api, video_out_data_t data);
DLL_PUBLIC void bwv_api_video_out_write_md_status(void* api, video_out_data_t data);

/** @}*/

//                _____    _____ 
//       /\      / ____|  / ____|
//      /  \    | |      | (___  
//     / /\ \   | |       \___ \ 
//    / ____ \  | |____   ____) |
//   /_/    \_\  \_____| |_____/ 
//                               
//     

/**
 *
 **************************************************************
 * \defgroup acs ACS module
 *
 * @brief ACS control functions
 *
 * This module provides control functions for work with Camera unit's ACS module
 *
 *
 * @{
 */

 /**
 *
  *  @struct acs_data_t
  *  @brief ACS configuration data structure
  *
  * This is the common structure for ACS related commands.
 */
typedef struct _PACKED_ {
	unsigned char gamma;					/**< Gamma control*/
	unsigned char gain;						/**< Gain control*/	
	unsigned char illuminator;				/**< Illuminator control*/	
	unsigned char clahe;                    /**< CLAHE enable*/	
	unsigned char ignore_saturated;         /**< Ignore saturated pixels*/	
	unsigned char desired_intensity;        /**< Desired image intensity*/	
	unsigned char current_intensity;        /**< Current image intensity*/	
	unsigned char pixel_percentage;         /**< Pixel percentage*/	
	unsigned short roi_width;               /**< ROI width*/	
	unsigned short roi_height;              /**< ROI height*/	
	unsigned char clip_limit;               /**< CLAHE clip limit*/	
	unsigned char clip_limit_alpha;         /**< CLAHE clip limit alpha*/	
	unsigned char min_clip_limit;           /**< Minimum CLAHE clip limit  */	
	unsigned char mid_clip_limit;           /**< Medium CLAHE clip limit */	
	unsigned char max_clip_limit;           /**< MAximum CLAHE clip limit */	
	unsigned short expcl_thr;               /**< */	
	unsigned char ent_th_max;               /**< */	
	unsigned char ent_th_min;               /**< */	
	unsigned short itotal_pulse_count;      /**< */	
	unsigned short imax_pulses;             /**< */	
	unsigned short min_exp;                 /**< Minimum exposure*/	
	unsigned short max_exp;                 /**< MAximum exposure*/	
	unsigned short gtotal_pulse_count;      /**< */	
	unsigned char alpha;                    /**< */	
	unsigned short gmax_pulses;             /**< */	
	unsigned char weather_mode;             /**< */	
	unsigned short first_line;           	/**< First image line*/	
} acs_data_t;

/**
*
* @brief ACS configuration data callback signature
*
* @param data ACS configuration data
* @param param User data
*/
typedef void (*acs_callback_t)(acs_data_t data, void *param);

/**
*
* @brief ACS status callback signature
*
* @param data ACS status. 0x00 - disabled, 0x01 - enabled
* @param param User data
*/
typedef void (*acs_callback_status_t)(unsigned char data, void* param);

/**
*
* @brief Add API callback for read ACS data command (acs_read_data)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_acs_add_callback_data(void* api, acs_callback_t callback, void* param);

/**
*
* @brief Add API callback for read ACS status command (acs_read_status)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_acs_add_callback_status(void* api, acs_callback_status_t callback, void* param);

/**
*
* @brief Read ACS configuration data
*
*/
DLL_PUBLIC void bwv_api_acs_read_data(void* api);

/**
*
* @brief Read ACS status
*
*/
DLL_PUBLIC void bwv_api_acs_read_status(void* api);

/**
*
* @brief Turn ACS on
*
*/
DLL_PUBLIC void bwv_api_acs_on(void* api);

/**
*
* @brief Turn ACS off
*
*/
DLL_PUBLIC void bwv_api_acs_off(void* api);

/**
*
* @brief Write ACS configuration data
*
* @param data Configuration data
* 
*/
DLL_PUBLIC void bwv_api_acs_write_data(void* api, acs_data_t data);

/** @}*/

//    ______   _____    _   _ 
//   |  ____| |  __ \  | \ | |
//   | |__    | |__) | |  \| |
//   |  __|   |  ___/  | . ` |
//   | |      | |      | |\  |
//   |_|      |_|      |_| \_|
//                            
//                            

/**
 *
 **************************************************************
 * \defgroup fpn FPN module
 *
 * @brief FPN control functions
 *
 * This module provides control functions for work with Camera unit's FPN module
 *
 *
 * @{
 */
#define NUC_INIT 		0x02	
#define NUC_INIT_DONE 	0x03	
#define NUC_DARK_DONE 	0x07	
#define NUC_BURN_DONE 	0x00	

 /**
  *
   *  @struct fpn_data_t
   *  @brief FPN/TRN status
   *
  */
typedef struct _PACKED_ {
	unsigned char fpn_enabled; 		/**< FPN enable. 0x00 - disabled, 0x01 - enabled*/
	unsigned char trn_enabled;		/**< TRN enable. 0x00 - disabled, 0x01 - enabled*/
} fpn_data_t;

/**
*
* @brief FPN status callback signature
*
* @param data FPN status data
* @param param User data
*/
typedef void (*fpn_callback_t)(fpn_data_t data, void* param);

/**
*
* @brief FPN calibration callback signature
*
* @param data FPN calibration state
* @param param User data
*/
typedef void (*fpn_calibrate_callback_t)(unsigned char data, void* param);

/**
*
* @brief Add API callback for read FPN status (fpn_read_fpn_status)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_fpn_add_callback_fpn_status(void* api, fpn_callback_t callback, void* param);

/**
*
* @brief Add API callback for read TRN status (fpn_read_trn_status)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_fpn_add_callback_trn_status(void* api, fpn_callback_t callback, void* param);

/**
*
* @brief Add API callback for FPN calibration
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_fpn_add_callback_calibration(void* api, fpn_calibrate_callback_t callback, void* param);

/**
*
* @brief Add API callback for FPN calibration progress
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_fpn_add_callback_calibration_progress(void* api, fpn_calibrate_callback_t callback, void* param);

/**
*
* @brief Add API callback for write FPN calibration status
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*/
DLL_PUBLIC int bwv_api_fpn_add_callback_calibration_write_status(void* api, fpn_calibrate_callback_t callback, void* param);

/**
*
* @brief Read FPN status
*
*/
DLL_PUBLIC void bwv_api_fpn_read_fpn_status(void* api);

/**
*
* @brief Read TRN status
*
*/
DLL_PUBLIC void bwv_api_fpn_read_trn_status(void* api);

/**
*
* @brief Read FPN calibration status
*
*/
DLL_PUBLIC void bwv_api_fpn_calibrate_read(void* api);

/**
*
* @brief Write FPN status
* 
* @param data FPN status
*
*/
DLL_PUBLIC void bwv_api_fpn_write_fpn_status(void* api, fpn_data_t data);

/**
*
* @brief Write TRN status
*
* @param data TRN status
*
*/
DLL_PUBLIC void bwv_api_fpn_write_trn_status(void* api, fpn_data_t data);

/**
*
* @brief Initialize FPN calibration process
*
*/
DLL_PUBLIC void bwv_api_fpn_calibrate_init(void* api);

/**
*
* @brief Confirm sensor was covered 
*
*/
DLL_PUBLIC void bwv_api_fpn_calibrate_confirm_dark(void* api);

/**
*
* @brief Finish FPN calibration process
*
*/
DLL_PUBLIC void bwv_api_fpn_calibrate_done(void* api);

/**
*
* @brief Read FPN calibration progress
*
*/
DLL_PUBLIC void bwv_api_fpn_read_calibration_progress(void* api);

/**
*
* @brief Read FPN calibration status of write to serial flash operation
*
*/
DLL_PUBLIC void bwv_api_fpn_read_calibration_write_status(void* api);

DLL_PUBLIC int bwv_api_c2db9f85d3b2df3c154e02d3a6e72a84(void* api, void* param1, void* param2);
DLL_PUBLIC void bwv_api_f21264ee3db297be48823ebca5bf9391(void* api);

/** @}*/

//    _    _   _____    _____  
//   | |  | | |  __ \  |  __ \ 
//   | |__| | | |  | | | |__) |
//   |  __  | | |  | | |  _  / 
//   | |  | | | |__| | | | \ \ 
//   |_|  |_| |_____/  |_|  \_\
//                             
//                             

typedef struct _PACKED_ {
	unsigned char enabled;				
	unsigned short alpha;                  
	unsigned char saturation;              
	unsigned char exposure;                
	unsigned int threshold_low;            
	unsigned int threshold_high;           
} hdr_data_t;

typedef void (*hdr_callback_t)(hdr_data_t data, void *param);

DLL_PUBLIC int bwv_api_hdr_add_callback_enabled(void* api, hdr_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_hdr_add_callback_alpha(void* api, hdr_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_hdr_add_callback_saturation(void* api, hdr_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_hdr_add_callback_exposure(void* api, hdr_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_hdr_add_callback_threshold_low(void* api, hdr_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_hdr_add_callback_threshold_high(void* api, hdr_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_hdr_read_enabled(void* api);
DLL_PUBLIC void bwv_api_hdr_read_alpha(void* api);
DLL_PUBLIC void bwv_api_hdr_read_saturation(void* api);
DLL_PUBLIC void bwv_api_hdr_read_exposure(void* api);
DLL_PUBLIC void bwv_api_hdr_read_threshold_low(void* api);
DLL_PUBLIC void bwv_api_hdr_read_threshold_high(void* api);
DLL_PUBLIC void bwv_api_hdr_write_enabled(void* api, hdr_data_t data);
DLL_PUBLIC void bwv_api_hdr_write_alpha(void* api, hdr_data_t data);
DLL_PUBLIC void bwv_api_hdr_write_saturation(void* api, hdr_data_t data);
DLL_PUBLIC void bwv_api_hdr_write_exposure(void* api, hdr_data_t data);
DLL_PUBLIC void bwv_api_hdr_write_threshold_low(void* api, hdr_data_t data);
DLL_PUBLIC void bwv_api_hdr_write_threshold_high(void* api, hdr_data_t data);

//    _____                               _         
//   |  __ \                             | |        
//   | |__) |  _ __    ___   ___    ___  | |_   ___ 
//   |  ___/  | '__|  / _ \ / __|  / _ \ | __| / __|
//   | |      | |    |  __/ \__ \ |  __/ | |_  \__ \
//   |_|      |_|     \___| |___/  \___|  \__| |___/
//


/**
 *
 **************************************************************
 * \defgroup presets FPN module
 *
 * @brief FPN control functions
 *
 * This module provides control functions for work with Camera unit's Presets module
 *
 *
 * @{
 */

#define RANGE_SLICING_MODE_MANUAL_MAX_TABLE_ITEMS 12


PACK_START


/**
 *
  *  @struct Preset_GeneralParamsStruct
  *  @brief General preset parameters data
  *
 */
typedef struct _PACKED_ {
	unsigned char  		Version;				/**< Preset version*/
	unsigned short		Tlaser2Light;			/**< Tlaser to Tlight time*/
	unsigned short		T1PulseDuration;		/**< Tx1 Pulse duration*/
	unsigned short		T1T2Delta;				/**< TX1 to TX2 time delta*/
	unsigned char		SpeedThreshold;			/**< Speed threshold for illuminator activation*/
	unsigned short		MaxEnergy;				/**< Maximum gating pulses energy allowed*/
	unsigned short		LaserDurationMinutes;	/**< Maximum illuminator override time duration*/
	unsigned char		LaserOnStart;			/**< */
	unsigned char		OSDState;				/**< HDMI OSD enable/disable*/
	unsigned char		RandomCycleDuration;	/**< Random gating cycle duration*/
} Preset_GeneralParamsStruct;

/**
 *
  *  @struct Preset_SeriesStruct
  *  @brief Preset structure that holds gating parameters for one series
  *
 */
typedef struct _PACKED_ {
	unsigned char 	state;						/**< Series state. (series enabled/disabled, illuminator enabled/disabled*/
	unsigned short 	passivePulses;				/**< Number of passive pulses*/
	unsigned short 	activePulses;				/**< Number of active pulses*/
	unsigned short 	tcycle;						/**< Tcycle time (multiplied by 100)*/
	unsigned short	tlaser;						/**< TLaser time (multiplied by 100)*/
	signed short 	tclose;						/**< Tclose time (multiplied by 100)*/
	unsigned short 	tgate;						/**< Tgate time  (multiplied by 100)*/
} Preset_SeriesStruct;

/**
 *
  *  @struct Preset_TypeStruct
  *  @brief Preset type structure. Holds description of 4 series
  *
 */
typedef struct _PACKED_ {
	Preset_SeriesStruct	Series[4];				/**< Array of Preset_SeriesStruct*/
} Preset_TypeStruct;

/**
 *
  *  @struct Preset_GateStruct
  *  @brief Preset gating description structure. Holds description of 4 types
  *
 */
typedef struct _PACKED_ {
	Preset_TypeStruct	Type[4];				/**< Array of Preset_TypeStruct*/
} Preset_GateStruct;


/**
 *
  *  @struct Preset_SubGateStruct
  *  @brief Preset subgating description structure
  *
 */
typedef struct _PACKED_ {
	Preset_GateStruct	GateControlTable[26];	/**< Subgating table*/
	unsigned char		ACS_calcOnType;			/**< ACS type*/
	unsigned char		ACS_calcOnSubType;		/**< ACS subtype*/
	unsigned char 		ACS_EnMask[25];			/**< ACS enabled types */
	unsigned char		First;					/**< First subgating table to work on*/
	unsigned char		Last;					/**< Last subgating table to work on*/
	unsigned char		SubgatingEnable;		/**< Subgating state. 0x00 - disabled, 0x01 - enabled*/
	unsigned char       TypesPerFrame;			/**< Number of types per 33ms time frame. 1,2 or 4*/
} Preset_SubGateStruct;

/**
 *
  *  @struct Preset_ContrastStruct
  *  @brief Preset contrast description structure
  *
 */
typedef struct _PACKED_ {
	unsigned char	ContrastSource;		/**< Contrast input type*/
	unsigned short High;				/**< Contrast high value*/
	unsigned short Low;					/**< Contrast low value*/
} Preset_ContrastStruct;

/**
 *
  *  @struct Preset_VideoOutStruct
  *  @brief Preset Video output description structure
  *
 */
typedef struct _PACKED_ {
	unsigned char						HDMI0;				/**< HDMI0 output type*/
	unsigned char						HDMI1;				/**< HDMI1 output type*/
	unsigned char						Analog;				/**< Analog video output type*/
	unsigned char						CameraLink;			/**< CameraLink/USB3 output types. Bit 8 is Image pixel format: (0 - 10BPP, 1 - 8BPP) */
	unsigned char  						ClaheSource;		/**< CLAHE input source*/
	Preset_ContrastStruct				Contrast;			/**< Contrast data*/
	unsigned char						Sub;				/**< Subtract direction*/
	unsigned char						HDMI0Res;			/**< HDMI0 resolution*/
	unsigned char						HDMI1Res;			/**< HDMI1 resolution*/
} Preset_VideoOutStruct;

/**
 *
  *  @struct Preset_ACSStruct
  *  @brief Preset ACS description structure
  *
 */
typedef struct _PACKED_ {
	unsigned char				ACSStatus;					/**< ACS status*/
	unsigned char				DesiredIntensity;			/**< Desired image intensity*/
	unsigned char				PixelPersentage;			/**< Pixel percentage*/
	unsigned short				ROIFirsLine;				/**< ROI first line*/
	unsigned short				ROIWidth;					/**< ROI width*/
	unsigned short				ROIHeight;					/**< ROI height*/
	unsigned short				maxLaserPulses;				/**< Maximum laser pulses*/
	unsigned short				maxGainPulses;				/**< Maximum gain pulses*/
	unsigned short				exposureThrMinLaser;		/**< Minimum exposure threshold*/
	unsigned short			 	exposureThrMaxLaser;		/**< Maximum exposure threshold*/
	unsigned short				minCL;						/**< Minimum CLAHE clip limit*/
	unsigned short				midCL;						/**< Medium CLAHE clip limit*/
	unsigned short				maxCL;						/**< Maximum CLAHE clip limit*/
} Preset_ACSStruct;


/**
 *
  *  @struct Preset_RangeSliceStruct
  *  @brief Preset Range slicing description structure
  * 
  * Preset is set of parameters which camera unit uses to initialize itself and Control box/Communication box if connected
  *
 */
typedef struct _PACKED_ {
	unsigned char demo_mode;													/**< Demonstration mode. 0x0 - enabled, 0x1 - disabled*/
	unsigned short	Tgate;														/**< Tgate time (multiplied by 100)*/
	unsigned short	Tlaser;														/**< Tlaser time (multiplied by 100)*/
	unsigned short	Tclose[RANGE_SLICING_MODE_MANUAL_MAX_TABLE_ITEMS];			/**< Tclose time array (multiplied by 100)*/
	unsigned short Pulses[RANGE_SLICING_MODE_MANUAL_MAX_TABLE_ITEMS];			/**< Number of pulses array */
	unsigned short ContrastHigh[RANGE_SLICING_MODE_MANUAL_MAX_TABLE_ITEMS];		/**< Contrast high value array*/
	unsigned short delay[RANGE_SLICING_MODE_MANUAL_MAX_TABLE_ITEMS];			/**< Delay time array*/
} Preset_RangeSliceStruct;

typedef struct _PACKED_ {
	unsigned char              active;
	Preset_SeriesStruct        SenseHighActive;
	Preset_SeriesStruct        SenseHighPassive;
	Preset_SeriesStruct        SenseLowActive;
	Preset_SeriesStruct        SenseLowPassive;
	unsigned char              HistPersentage;
	unsigned short             ROIStartRow;
	unsigned short             ROIEndRow;
	unsigned short             ROIStartCol;
	unsigned short             ROIEndCol;
	unsigned int               a;
	unsigned int               b;
	unsigned int               s0;
	unsigned char              v1;
	unsigned char              v2;
	unsigned char              v3;
	unsigned char              v4;
	Preset_TypeStruct        	LowVisibilityGate;
	Preset_TypeStruct        	MediumVisibilityGate;
	Preset_TypeStruct        	HighVisibilityGate;
} Preset_BadWeatherStruct;

/**
 *
  *  @struct Preset_Struct
  *  @brief Preset data structure
  *
 */
typedef struct _PACKED_ {
	unsigned char				PresetValid;			/**< Preset valid flag 0x55*/
	unsigned char				PresetName[32];			/**< Preset name*/
	Preset_GeneralParamsStruct	General;				/**< General preset parameters*/
	Preset_VideoOutStruct		VideoOut;				/**< Video output parameters*/
	Preset_ACSStruct			ACS;					/**< ACS parameters*/
	Preset_RangeSliceStruct		RangeSlice;				/**< Range slicing parameter*/
	Preset_SubGateStruct		SubGating;				/**< Subgating parameters*/
	Preset_BadWeatherStruct     BadWeather;				
} Preset_Struct;
PACK_END

/**
*
* @brief API callback signature for read active preset command (presets_read_active)
*
* @param active Active preset number
* @param total Total number of presets
* @param param User data
* 
*/
typedef void (*preset_callback_active_t)(unsigned char active, unsigned char total, void *param);

/**
*
* @brief API callback signature for read preset name command (presets_read_name)
*
* @param preset_number Preset number
* @param name Preset name
* @param param User data
*
*/
typedef void (*preset_callback_name_t)(unsigned char preset_number, unsigned char name[32], void* param);

/**
*
* @brief API callback signature for read preset command (presets_read_data)
*
* @param data Preset data structure
* @param param User data
*
*/
typedef void (*preset_callback_data_t)(Preset_Struct data, void* param);

/**
*
* @brief API callback signature preset read/write progress (presets_read_data/presets_write_data)
*
* @param total Total percentage
* @param done Current percentage
* @param param User data
*
*/
typedef void (*preset_callback_progress_t)(unsigned char total, unsigned char done, void* param);


typedef void (*preset_callback_current_t)(unsigned char current, void* param);

/**
*
* @brief API callback signature preset write status progress (presets_read_data/presets_write_data)
*
* @param status Status of operation
* @param param User data
*
*/
typedef void (*preset_callback_status_t)(unsigned char status, void* param);


/**
*
* @brief Add API callback for reading active preset (presets_read_active)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_active(void* api, preset_callback_active_t callback, void* param);
DLL_PUBLIC int bwv_api_presets_add_callback_current(void* api, preset_callback_current_t callback, void* param);

/**
*
* @brief Add API callback for reading preset name (presets_read_name)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_name(void* api, preset_callback_name_t callback, void* param);

/**
*
* @brief Add API callback for reading preset data (presets_read_data)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_data(void* api, preset_callback_data_t callback, void* param);

/**
*
* @brief Add API callback for receiving preset read/write progress update
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_progress(void* api, preset_callback_progress_t callback, void* param);

/**
*
* @brief Add API callback for acknowledge of preset erase operation (presets_erase_all)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_erase(void* api, preset_callback_status_t callback, void* param);

/**
*
* @brief Add API callback for acknowledge of preset erase operation (presets_erase_all)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_write(void* api, preset_callback_status_t callback, void* param);

/**
*
* @brief Add API callback for reading preset version (presets_read_version)
*
* @param callback Callback function
* @param param User data
* @return 0 on success, -1 on failure
*
*/
DLL_PUBLIC int bwv_api_presets_add_callback_version(void* api, preset_callback_status_t callback, void* param);

/**
*
* @brief Read active Camera unit preset
*
*
*/
DLL_PUBLIC void bwv_api_presets_read_active(void* api);
DLL_PUBLIC void bwv_api_presets_read_current(void* api);

/**
*
* @brief Read Camera unit preset version
*
*/
DLL_PUBLIC void bwv_api_presets_read_version(void* api);

/**
*
* @brief Read preset name
*
* @param preset_number Preset number
*
*/
DLL_PUBLIC void bwv_api_presets_read_name(void* api, unsigned char preset_number);

/**
*
* @brief Read preset data
*
* @param preset_number Preset number
*
*/
DLL_PUBLIC void bwv_api_presets_read_data(void* api, unsigned char preset_number);

/**
*
* @brief Erase all presets in Camera unit
*
*/
DLL_PUBLIC void bwv_api_presets_erase_all(void* api);

/**
*
* @brief Read current preset write status
*
*/
DLL_PUBLIC void bwv_api_presets_get_write_status(void* api);

/**
*
* @brief Write active preset to Camera unit
* @param preset_number Preset number
*
*/
DLL_PUBLIC void bwv_api_presets_write_active(void* api, unsigned char preset_number);
DLL_PUBLIC void bwv_api_presets_write_current(void* api, unsigned char preset_number);

/**
*
* @brief Write preset data to Camera unit
* @param preset_number Preset number
* @param data Preset data
*
*/
DLL_PUBLIC void bwv_api_presets_write_data(void* api, unsigned char preset_number, Preset_Struct data);

/**
*
* @brief Save preset data to file
* @param filename File name
* @param data Preset data
*
*/
DLL_PUBLIC void bwv_api_presets_save_to_file(char* filename, Preset_Struct data);

/**
*
* @brief Load preset from file
* @param filename File name
* @param data Preset data
* @return Number of bytes read. HAve to me equal to size of Preset_Struct
*/
DLL_PUBLIC int bwv_api_presets_load_from_file(char* filename, Preset_Struct* data);

/**
*
* @brief Get version of API preset module. Have to be equal to presets version in Camera unit
* @return Version number
*
*/
DLL_PUBLIC unsigned char bwv_api_presets_get_version();

/**
*
* @brief Cancel ongoing preset operations (read/write)
*
*/

DLL_PUBLIC void bwv_api_presets_cancel(void* api);

/** @}*/
//    _____
//   |  __ \                                        
//   | |  | |   ___   _ __ ___     ___              
//   | |  | |  / _ \ | '_ ` _ \   / _ \             
//   | |__| | |  __/ | | | | | | | (_) |            
//   |_____/   \___| |_| |_| |_|  \___/             
//                                                  
// 

#define RANGE_SLICING_MODE_AUTO						1		
#define RANGE_SLICING_MODE_MANUAL					2	

typedef struct _PACKED_ {
	unsigned char index;						
	unsigned short external_pulses;             
	unsigned short contrast;                    
	double tclose;                              
	double tgate;                               
	double tlaser;                              
} range_slicing_custom_table_item_t;

typedef struct _PACKED_ {
	unsigned char mode;							
	unsigned char auto_mode_started;		    
	unsigned char auto_mode_speed;		        
	unsigned char auto_mode_reverse;     		
} range_slicing_data_t;

typedef void (*range_slicing_callback_t)(range_slicing_data_t data, void *param);
typedef void (*range_slicing_table_callback_t)(range_slicing_custom_table_item_t data, void* param);

typedef struct _PACKED_ {
	unsigned char enabled;
	unsigned short threshold;
} retro_emphasis_data_t;

typedef void (*retro_emphasis_callback_t)(retro_emphasis_data_t data, void* param);

DLL_PUBLIC int bwv_api_range_slicing_add_callback_mode(void* api, range_slicing_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_range_slicing_add_callback_speed(void* api, range_slicing_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_range_slicing_add_callback_start_stop(void* api, range_slicing_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_range_slicing_add_callback_reverse(void* api, range_slicing_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_range_slicing_add_callback_table(void* api, range_slicing_table_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_range_slicing_read_mode(void* api);
DLL_PUBLIC void bwv_api_range_slicing_read_speed(void* api);
DLL_PUBLIC void bwv_api_range_slicing_read_start_stop(void* api);
DLL_PUBLIC void bwv_api_range_slicing_read_reverse(void* api);
DLL_PUBLIC void bwv_api_range_slicing_read_manual_mode_table(void* api);
DLL_PUBLIC void bwv_api_range_slicing_write_mode(void* api, range_slicing_data_t data);
DLL_PUBLIC void bwv_api_range_slicing_auto_mode(void* api);
DLL_PUBLIC void bwv_api_range_slicing_toggle_start_stop(void* api);
DLL_PUBLIC void bwv_api_range_slicing_write_reverse(void* api, range_slicing_data_t data);
DLL_PUBLIC void bwv_api_range_slicing_write_speed(void* api, range_slicing_data_t data);
DLL_PUBLIC void bwv_api_range_slicing_disable(void* api);
DLL_PUBLIC void bwv_api_range_slicing_write_manual_mode_parameters(void* api, range_slicing_custom_table_item_t parameters);
DLL_PUBLIC void bwv_api_range_slicing_burn_manual_mode_table(void* api, range_slicing_custom_table_item_t items[RANGE_SLICING_MODE_MANUAL_MAX_TABLE_ITEMS], uint8_t item_count, double tlaser, double tgate);
DLL_PUBLIC int  bwv_api_retro_emphasis_add_callback_enabled(void* api, retro_emphasis_callback_t callback, void* param);
DLL_PUBLIC int  bwv_api_retro_emphasis_add_callback_threshold(void* api, retro_emphasis_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_retro_emphasis_read_enabled(void* api);
DLL_PUBLIC void bwv_api_retro_emphasis_read_threshold(void* api);
DLL_PUBLIC void bwv_api_retro_emphasis_write_enabled(void* api, retro_emphasis_data_t data);
DLL_PUBLIC void bwv_api_retro_emphasis_write_threshold(void* api, retro_emphasis_data_t data);


//   __          __                 _     _                   
//   \ \        / /                | |   | |                  
//    \ \  /\  / /    ___    __ _  | |_  | |__     ___   _ __ 
//     \ \/  \/ /    / _ \  / _` | | __| | '_ \   / _ \ | '__|
//      \  /\  /    |  __/ | (_| | | |_  | | | | |  __/ | |   
//       \/  \/      \___|  \__,_|  \__| |_| |_|  \___| |_|   
//                                                            
//                                                            

typedef struct _PACKED_ {
	unsigned char control;
	unsigned char state;
	unsigned char yc_high;
	unsigned char yc_low;
	unsigned char v1;
	unsigned char v2;
	unsigned char v3;
	unsigned char v4;
	double a;
	double b;
	double s0;
	double slope;
	double visibility;
} weather_data_t;

typedef void (*weather_callback_t)(weather_data_t data, void *param);

DLL_PUBLIC int bwv_api_weather_add_callback_control(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_state(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_yc_high(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_yc_low(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_v1(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_v2(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_v3(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_v4(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_a(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_b(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_s0(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_slope(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC int bwv_api_weather_add_callback_visibility(void* api, weather_callback_t callback, void* param);
DLL_PUBLIC void bwv_api_weather_read_control(void* api);
DLL_PUBLIC void bwv_api_weather_read_state(void* api);
DLL_PUBLIC void bwv_api_weather_read_yc_high(void* api);
DLL_PUBLIC void bwv_api_weather_read_yc_low(void* api);
DLL_PUBLIC void bwv_api_weather_read_v1(void* api);
DLL_PUBLIC void bwv_api_weather_read_v2(void* api);
DLL_PUBLIC void bwv_api_weather_read_v3(void* api);
DLL_PUBLIC void bwv_api_weather_read_v4(void* api);
DLL_PUBLIC void bwv_api_weather_read_a(void* api);
DLL_PUBLIC void bwv_api_weather_read_b(void* api);
DLL_PUBLIC void bwv_api_weather_read_s0(void* api);
DLL_PUBLIC void bwv_api_weather_read_slope(void* api);
DLL_PUBLIC void bwv_api_weather_read_visibility(void* api);
DLL_PUBLIC void bwv_api_weather_on(void* api);
DLL_PUBLIC void bwv_api_weather_off(void* api);
DLL_PUBLIC void bwv_api_weather_write_control(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_v1(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_v2(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_v3(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_v4(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_a(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_b(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_s0(void* api, weather_data_t data);
DLL_PUBLIC void bwv_api_weather_write_mode(void* api, uint8_t data);

//    _    _               _           _          
//   | |  | |             | |         | |         
//   | |  | |  _ __     __| |   __ _  | |_    ___ 
//   | |  | | | '_ \   / _` |  / _` | | __|  / _ \
//   | |__| | | |_) | | (_| | | (_| | | |_  |  __/
//    \____/  | .__/   \__,_|  \__,_|  \__|  \___|
//            | |                                 
//            |_|                                 

/**
 *
 **************************************************************
 * \defgroup fwupdate Firmware update module
 *
 * @brief Function for sending firmware update to Camera unit/Control box/Communication box
 *
 * This module provides control functions for work with Camera unit's FPN module
 * 
 * @{
 */

 /**
  *  @struct update_data_t
  *  @brief Update process progress information
 */
typedef struct _PACKED_ {
	unsigned char device;				/**< Device id being updated (Camera unit/Control box/Communication box)*/
	unsigned char overal_progress;		/**< Current progress of firmware update operation*/
	unsigned char overall_total;		/**< Overall total percentage of firmware update operation*/
	unsigned int current_progress;		/**< Current progress of current update stage*/
	unsigned int current_total;			/**< Total progress of current update stage*/
} update_data_t;

/**
 * @brief API callback signature for update progress indication
 *
 * @param data Update information data
 * @param param User data
 */
typedef void (*update_progress_callback_t)(update_data_t data, void * param);

/**
 * @brief API callback signature for update done event
 *
 * @param local_error API error. 0x00 - no error
 * @param system_error System (Camera unit/Control box/Communication box) error. 0x00 - no error
 * @param param User data
 */
typedef void (*update_done_callback_t)(unsigned char local_error, unsigned char system_error, void* param);

/**
 * @brief Add APi callback for progress indication
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_update_add_callback_progress(void* api, update_progress_callback_t callback, void* param);

/**
 * @brief Add APi callback for update done event
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_update_add_callback_done(void* api, update_done_callback_t callback, void* param);

/**
 * @brief Start firmware update process
 *
 * @param filename FW file name
 * @param filename_len Length in bytes of file name
 * @return 0 on successful start, -1 on failure starting process
 */
DLL_PUBLIC int bwv_api_update(void* api, char* filename, int filename_len);

DLL_PUBLIC int a4e9b4247ad0d04592218be4465ebca9(const char* mcs, const char* bin);

/** @}*/

//     _____                   _                         
//    / ____|                 | |                        
//   | |        __ _   _ __   | |_   _   _   _ __    ___ 
//   | |       / _` | | '_ \  | __| | | | | | '__|  / _ \
//   | |____  | (_| | | |_) | | |_  | |_| | | |    |  __/
//    \_____|  \__,_| | .__/   \__|  \__,_| |_|     \___|
//                    | |                                
//                    |_|                                

/**
 *
 * \defgroup capture Capture module
 *
 * @brief Video capture functions
 * 
 * 
 * This module provides video capture related functions
 *
 * 
 * @{
 */

/**
 *  @enum capture_limit_type_t
 *  @brief Limited capture mode
*/
typedef enum {
	CM_UNLIMITED = 0,		/**< Unlimited capture*/
	CM_LIMIT_FRAMES,		/**< Capture is limited by number of frames*/
	CM_LIMIT_TIME,			/**< Capture is limited by time*/
	CM_LIMIT_FILESIZE		/**< Capture is limited by file size*/
} capture_limit_type_t;


/**
 *  @struct capture_data_t
 *  @brief Capture status/information structure
*/
typedef struct _PACKED_ {
	uint8_t frame_type;				/**< Current frame type*/
	uint32_t lost_frames;			/**< Number of lost frames*/
	int width;						/**< Frame width in pixels*/
	int height;						/**< Frame height in pixels*/
	unsigned short* rawimage;		/**< Raw frame data. Note - this is static pointer. Do not attempt to free. */
	bool is_8bpp;					/**< Image pixel format. True - 8BPP, False - 10BPP*/
	uint8_t fps;					/**< Current FPS value*/
	capture_limit_type_t limit_type;/**< Capture limit type*/
	uint32_t limit_value;			/**< Capture limit value*/
} capture_data_t;

/**
 *  @struct capture_finish_info_t
 *  @brief Capture finish information
*/
typedef struct _PACKED_ {
	bool success;					/**< True when capture was finished by user, or by limit value, false if error occurred*/
	uint32_t number_of_frames;		/**< Total number of frames captured*/
} capture_finish_info_t;

typedef struct {
	uint32_t index;	
	uint32_t hw_frame_counter;
	uint64_t sw_timestamp;
	uint32_t timestamp;
	uint16_t timestamp_ms;
	uint8_t  bpp;
	uint8_t  fps;

	uint8_t frame_type_number;
	uint8_t id;
	uint8_t version_maj;
	uint8_t version_min;
	uint8_t frame_type;

	uint8_t contrast_en;
	uint16_t contrast_max;
	uint16_t contrast_min;

	uint8_t fpn_en;
	uint8_t trn_en;

	uint16_t params[8];

	Preset_SeriesStruct gating[4];
} recording_frame_info_t;

/**
 * @brief API callback for capture start event
 *
 * @param width Frame width
 * @param height Frame height
 * @param param User data
 */
typedef void (*capture_callback_start_t)(int width, int height, void* param);

/**
 * @brief API callback for frame capture event
 *
 * @param data Capture status/information structure
 * @param param User data
 * 
 */
typedef void (*capture_callback_data_t)(capture_data_t data, void* param);

/**
 * @brief API callback for capture end event
 *
 * @param param User data
 */
typedef void (*capture_callback_end_t)(void* param);

/**
 * @brief API callback for capture finished event. Called when limited capture is done
 *
 * @param info Capture finish information
 * @param param User data
 */
typedef void (*capture_callback_capture_finished_t)(capture_finish_info_t info, void* param);

/**
 * @brief Add API callback for capture start event
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_add_callback_start(void* api, capture_callback_start_t callback, void* param);

/**
 * @brief Add API callback for capture end event
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_add_callback_end(void* api, capture_callback_end_t callback, void* param);

/**
 * @brief Add API callback for preview failure event
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_add_callback_error(void* api, capture_callback_end_t callback, void* param);
/**
 * @brief Add API callback for capture finished event
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_add_callback_capture_finished(void* api, capture_callback_capture_finished_t callback, void* param);

/**
 * @brief Add API callback for capture frame event
 *
 * @param callback Callback function
 * @param param User data
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_add_callback_data(void* api, capture_callback_data_t callback, void* param);

/**
 * @brief Remove API callback for capture start event
 *
 * @param callback Callback function
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_remove_callback_start(void* api, capture_callback_start_t callback);

/**
 * @brief Remove API callback for capture end event
 *
 * @param callback Callback function
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_remove_callback_end(void* api, capture_callback_end_t callback);

/**
 * @brief Remove API callback for capture frame event
 *
 * @param callback Callback function
 * @return 0 on success, -1 on failure
 */

DLL_PUBLIC int bwv_api_capture_remove_callback_data(void* api, capture_callback_data_t callback);

/**
 * @brief Remove API callback for capture finished event
 *
 * @param callback Callback function
 * @return 0 on success, -1 on failure
 */

DLL_PUBLIC int bwv_api_capture_remove_callback_capture_finished(void* api, capture_callback_capture_finished_t callback);


/**
 * @brief Initialize video channel and start image preview
 *
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_start_preview(void* api);

/**
 * @brief Stop image preview
 * 
 */
DLL_PUBLIC void bwv_api_capture_stop_preview(void* api);

/**
 * @brief Start simple capture to file
 *
 * @param filename target filename
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_start_capture(void* api, const char* filename);

/**
 * @brief Start limited capture
 *
 * @param filename target filename
 * @param type type of limited capture
 * @param value limit value
 * @return 0 on success, -1 on failure
 */
DLL_PUBLIC int bwv_api_capture_start_limited_capture(void* api, const char* filename, capture_limit_type_t type, uint32_t value);

DLL_PUBLIC unsigned long long bwv_api_get_timestamp();
DLL_PUBLIC void bwv_api_timestamp2time(unsigned long long timestamp, unsigned short* year, unsigned short* month, unsigned short* day, unsigned short* hour, unsigned short* minute, unsigned short* sec, unsigned short* msec);

/**
 * @brief Stop capture
 *
 */
DLL_PUBLIC void bwv_api_capture_stop_capture(void* api);

DLL_PUBLIC int bwv_api_capture_get_time(void* api);
DLL_PUBLIC int bwv_api_capture_get_end_time(void* api);
DLL_PUBLIC bool bwv_api_capture_is_capturing(void* api);
DLL_PUBLIC unsigned int bwv_api_capture_get_lost_frames(void* api);
DLL_PUBLIC void bwv_api_capture_reset_lost_frames(void* api);

DLL_PUBLIC int bwv_api_capture_recording_open(char* filename, void** handle);
DLL_PUBLIC int bwv_api_capture_recording_get_frame_size(void* handle, int* width, int* height);
DLL_PUBLIC int bwv_api_capture_recording_get_frame_count(void* handle);
DLL_PUBLIC int bwv_api_capture_recording_get_frame(void* handle, int index, uint16_t* data, recording_frame_info_t* info);
DLL_PUBLIC void bwv_api_capture_recording_close(void* handle);

/** @}*/

#ifdef __cplusplus
}
#endif


#endif
