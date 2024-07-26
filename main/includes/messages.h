


























#ifndef MESSAGES_H
#define MESSAGES_H
#include "sdr_structs.h"
//struct CamMeasurement;
//struct ImuMeasurement;
#define CCSDS_APID_BASE	(0x0)	//!< The CCSDS APID Base number for our receiver

/*! @ingroup MESSAGES
 *  @brief Enum the packet ID #s, DO NOT REORDER */
enum CCSDS_MESSAGES_IDS
{
	FIRST_M_ID,
	FIRST_PERIODIC_M_ID, //xin: messages between FIRST_PERIODIC_M_ID and LAST_PERIODIC_M_ID can be telemetry.cpp::setMessageRate()
//	BOARD_HEALTH_M_ID,
//	TASK_HEALTH_M_ID,
	SPS_M_ID,
//	TOT_M_ID,
//	PPS_M_ID,
//	CLOCK_M_ID,
//	CHANNEL_M_ID,
//	SV_POSITION_M_ID,
//	MEASUREMENT_M_ID,
//	PSEUDORANGE_M_ID,
	SV_PREDICTION_M_ID,
	LAST_PERIODIC_M_ID,
//	EKF_STATE_M_ID,
//	EKF_COVARIANCE_M_ID,
//	EKF_RESIDUAL_M_ID,
//	COMMAND_M_ID,
//	COMMAND_ACK_M_ID,
//	EPHEMERIS_M_ID,
//	ALMANAC_M_ID,
//	EPHEMERIS_STATUS_M_ID,
//	SV_SELECT_STATUS_M_ID,
//	EEPROM_M_ID,
//	EEPROM_CHKSUM_M_ID,
//	MEMORY_M_ID,
//	MEMORY_CHKSUM_M_ID,
	LAST_M_ID
};


/*! @ingroup MESSAGES
 *  @brief Packet dumped to telemetry and to disk to keep track of each channel */
typedef struct CCSDS_Packet_Header
{

	uint16 pid;	//!< Packet ID
	uint16 psc;	//!< Packet sequence control
	uint16 pdl; //!< Packet data length

} CCSDS_Packet_Header;


/*! @ingroup MESSAGES
 *  @brief Decoded header
 */
typedef struct CCSDS_Decoded_Header
{
	uint32 id;
	uint32 type;
	uint32 tic;
	uint32 length;
} CCSDS_Decoded_Header;

/*! @ingroup MESSAGES
 *  @brief Packet format
 */
typedef struct CCSDS_Packet
{
	uint32 preamble;			//!< Preamble, should ALWAYS be 0xAAAAAAAA
	CCSDS_Packet_Header header; //!< CCSDS header
//	uint8 payload[502];			//!< Payload, up to 502 bytes
//	uint8 payload[8182];		//!< Payload, up to 2048*4-10=8182 bytes //xin 20220422
//	uint8 payload[2038];		//!< Payload, up to upper(150(pts)*4(bytes)*2(x&y))-10=2048-10=2038 bytes //xin 20220422
	uint8 payload[4086];		//!< Payload, up to upper(150(pts)*4(bytes)*2(x&y)*2cams)-10=4096-10=4086 bytes //xin 20220530
} CCSDS_Packet;
















/*! @ingroup MESSAGES
 *  @brief The predicted state of an SV via the almanac
 */
typedef struct SV_Prediction_M
{

//	double time;				//!< GPS time of predicition
//	double elev;				//!< Predicted elev
//	double azim;				//!< Predicted azim
//	double v_elev;				//!< Elevation of vehicle relative to SV
//	double v_azim;				//!< Azimuth of vehicle relative to SV
//	double delay;				//!< Predicted delay (seconds)
//	double doppler;				//!< Predicted Doppler (Hz)
//	double doppler_rate;			//!< Predicted Doppler rate (Hz/sec)
	int32 sv;					//!< SV number
//	int32 visible;				//!< Should the SV be visible?
//	int32 mode;					//!< Cold/warm/hot
	int32 tracked;				//!< Is it being tracked?
	int32 predicted;			//!< Has it been predicted?
//	CamMeasurement mCamMeas; //xin
//	ImuMeasurement mImuMeas; //xin
	int64_t miTimestamp_Cam[2]; //xin
	char 	msImage[2][128];    //xin
//	std::vector<cv::Point2f> mvCur_un_pts;
	size_t	mvIds[2][150];//xin: 2023-5-30
	float   mfCur_un_pts_x[2][150];//float   mfCur_un_pts_x[2048];
	float   mfCur_un_pts_y[2][150];//float   mfCur_un_pts_y[2048];
	uint32	muCnt_un_pts[2]; // the maximum number of feature points in a single frame is 150(MAX_CNT), so uint32 should be enough, even for future use 
} SV_Prediction_M;




































/*! @ingroup MESSAGES
 *  @brief Raw PVT navigation solution
*/
typedef struct SPS_M
 {

/*	double x;			//!< ECEF x coordinate (meters)
	double y;			//!< ECEF y coordinate (meters)
	double z;			//!< ECEF z coordinate (meters)
	double vx;			//!< ECEF x velocity (meters/sec)
	double vy;			//!< ECEF x velocity (meters/sec)
	double vz;			//!< ECEF x velocity (meters/sec)
	double time;		//!< time in seconds
	double clock_bias;	//!< clock bias in seconds
	double clock_rate;  //!< clock rate in meters/second
	double latitude;	//!< latitude in decimal radians
	double longitude;	//!< longitude in decimal radians
	double altitude;	//!< height in meters
	double gdop;		//!< geometric dilution of precision
	uint32 nsvs;		//!< This is a mask, not a number!
	uint32 converged;	//!< declare convergence
	uint32 iterations;			//!< Iterations
	uint32 stale_ticks;			//!< count the number of ticks since the last good sltn
	uint32 converged_ticks;		//!< count number of converged ticks
	uint32 chanmap[MAX_CHANNELS]; //!< Map of channels->sv
*/	uint32 tic;			//!< Corresponds to this receiver tic

} SPS_M;



















































































































































































































































































































































































/*! @ingroup MESSAGES
 *  @brief Holds all the relevant messages in a single container */
typedef struct Message_Struct
{
	/* Data gets stored here! */
/*	Board_Health_M 		board_health;					//!< Board health message
	Task_Health_M		task_health;					//!< Task health message*/
	SPS_M				sps;							//!< SPS message
/*	TOT_M				tot;							//!< UTC information
	PPS_M				pps;							//!< PPS status
	Clock_M				clock;							//!< Clock message
	EKF_State_M			ekf_state;						//!< EKF state
	EKF_Covariance_M	ekf_covariance;					//!< EKF covariance
	EKF_Residual_M		ekf_residual;					//!< EKF residual
	Channel_M 			channel[MAX_CHANNELS+1]; 		//!< Channel health message, last element is used as a buffer
	SV_Position_M		sv_positions[MAX_CHANNELS+1];	//!< SV Positions, last element is used as a buffer
	Measurement_M 		measurements[MAX_CHANNELS+1];	//!< Measurements, last element is used as a buffer
	Pseudorange_M 		pseudoranges[MAX_CHANNELS+1];	//!< Pseudoranges, last element is used as a buffer
	Command_Ack_M		command_ack;					//!< Command acknowledgement
	Ephemeris_M			ephemerides[MAX_SV+1];			//!< Ephemeris message, last element is used as a buffer
	Almanac_M			almanacs[MAX_SV+1];				//!< Almanac message, last element is used as a buffer
	Ephemeris_Status_M	ephemeris_status;				//!< Status of ephemeris
	SV_Select_Status_M	sv_select_status;				//!< SV Select status
*/	SV_Prediction_M		sv_predictions[MAX_SV+1];		//!< SV state prediction
/*	EEPROM_M			eeprom;							//!< EEPROM dump
	EEPROM_Chksum_M		eeprom_chksum;					//!< EEPROM checksum
	Memory_M			memory;							//!< Memory dump
	Memory_Chksum_M		memory_chksum;					//!< Memory checksum*/
} Message_Struct;


/*! @ingroup MESSAGES
 *  @brief Union containing all the individual messages */
typedef union Message_Union//typedef union Message_Union//TODO TODO: considering (1) boost::variant (2) a dedicated named pipe for high-bandwidth data, e.g. images, in addition to the original message union, i.e. Message_Union
{
/*	Board_Health_M		board_health;
	Task_Health_M		task_health;
*/	SPS_M				sps;
/*	TOT_M				tot;
	PPS_M				pps;
	Clock_M				clock;
	EKF_State_M			ekf_state;
	EKF_Covariance_M	ekf_covariance;
	EKF_Residual_M		ekf_residual;
	Channel_M			channel;
	SV_Position_M		sv_position;
	Measurement_M		measurement;
	Pseudorange_M		pseudorange;
	//Command_M			command;
	Command_Ack_M		command_ack;
	Ephemeris_M			ephemeris;
	Almanac_M			almanac;
	Ephemeris_Status_M	ephemeris_status;
	SV_Select_Status_M	sv_select_status;
*/	SV_Prediction_M		sv_prediction;
/*	EEPROM_M			eeprom;
	EEPROM_Chksum_M		eeprom_chksum;
	Memory_M			memory;
	Memory_Chksum_M		memory_chksum;
*/
} Message_Union;

#endif /* MESSAGES_H */
