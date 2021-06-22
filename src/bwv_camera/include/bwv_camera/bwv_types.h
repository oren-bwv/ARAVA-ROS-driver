#ifndef __BWV_TYPES_H__
#define __BWV_TYPES_H__


#include "bwv_api.h"        // device_t

#define FRAME_WIDTH 800
#define FRAME_HEIGHT 482

extern uint64_t g_metadataTimer;

/** @file */

/// Configuration class.
///
/// holds dynamic reconfigure parameters.

typedef struct Config
{
	/// minRange - minimum visible range in meters, available ranges 12 - 350[m]
	unsigned short minRange;

	/// maxRange - maximum visible range in meters, available ranges 12 - 350[m]
	unsigned short maxRange;

	/// workingMode - availabe working modes -\n
	///		\t 0 LRN - default mode (Left, Right, None illumination) \n
	///		\t 1 BNN (Both, None, None illumination) \n
	///		\t 2 Full - ACS
	unsigned char workingMode;

	///	illuminationPower - power of illumination, range 0 - 130[%]
	unsigned char illuminationPower;

	/// tlaser - laser time duration - range 12 - 70 which represents 0.12 - 0.70 [usec]
	unsigned char tlaser;

	///  curvature - not implemented
	unsigned char curvature;
} config_t;


/// context struct.
///
/// describes the related members available to other classes
typedef struct ContextStruct
{
	/// reference to the master API instance
    void* apiMaster;

    /// reference to the slave API instance
    void* apiSlave;

    /// connection master device
    device_t devMaster;

    /// connection slave device
    device_t devSlave;

    /// local configuration parameters
    /// @see Config
    config_t config;

} context_t;

enum masterOrSlave
{
    MASTER, /**< is master */
    SLAVE 	/**< is slave */
};

/// row number 0 offsets
enum MetadataOffsetFirstRow
{
	TIMESTAMP 		 = 8,	/**< 8 */
	HW_FRAME_COUNTER = 20	/**< 20 */
};

/// row number 1 offsets
enum MetadataOffsetSecondRow
{
	ARAVA_ID 		= 0,	/**< 0 */

	MASTER_SLAVE 	= 200,	/**< 200 */
	X_START 		= 201,	/**< 201 */
	X_END 			= 202,	/**< 202 */
	POWER 			= 203,	/**< 203 */
	TLASER 			= 204,	/**< 204 */
	WORKING_MODE 	= 205,	/**< 205 */
	ILLUMINATOR 	= 206,	/**< 206 */
	ROAD_CURVATURE 	= 207	/**< 207 */
};

#endif // __BWV_TYPES_H__
