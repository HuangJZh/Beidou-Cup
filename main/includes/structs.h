


























#ifndef STRUCTS_H_
#define STRUCTS_H_













/*! @ingroup STRUCTS
 *  @brief Tell the PVT how many measurements are in the pipe */
typedef struct Preamble_2_PVT_S
{
	uint32  tic_measurement;
//xin//not given a value in the original code//	uint32 	pps_accum;

} Preamble_2_PVT_S;













































































/*! @ingroup STRUCTS
 *	@brief Get the result of an acquisition */
typedef struct Acq_Command_S
{

	/* Command sent to acquisition */
	int32   sv;						//!< PRN number
	int32   type;					//!< ACQ_STRONG=0, ACQ_MEDIUM=1, ACQ_WEAK=2, ACQ_FINE=3

	int     success;				//!< Did the acq say the SV was detected?

	int32   count;					//!< 1 ms ticks


	int64_t miTimestamp_Cam[2]; //xin
	char 	msImage[2][128];    //xin
	size_t	mvIds[2][150];// xin: 2023-5-30
	float   mfCur_un_pts_x[2][150];
	float   mfCur_un_pts_y[2][150];
	uint32	muCnt_un_pts[2]; // the maximum number of feature points in a single frame is 150(MAX_CNT), so uint32 should be enough, even for future use
} Acq_Command_S;


























































/*! @ingroup STRUCTS
 *  @brief Dump info from the PVT to the Telemetry */
typedef struct PVT_2_TLM_S
{
	SPS_M 	sps;
/*	Clock_M clock;
	Pseudorange_M pseudoranges[MAX_CHANNELS];
	Measurement_M measurements[MAX_CHANNELS];
	SV_Position_M sv_positions[MAX_CHANNELS];
	TOT_M tot;*/
} PVT_2_TLM_S;






















/*! @ingroup STRUCTS
 *  @brief Dump SV Select state to the telemetry */
typedef SV_Prediction_M SVS_2_TLM_S;







#endif /* STRUCTS_H_ */
