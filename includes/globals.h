


























#ifdef GLOBALS_HERE
	#define EXTERN
#else
	#define EXTERN extern
#endif

#ifndef GLOBALS_H_
#define GLOBALS_H_

/* Part 1, Threaded and Semi-threaded objects */
/*----------------------------------------------------------------------------------------------*/
//EXTERN class Keyboard		*pKeyboard;						//!< Handle user input
EXTERN class FIFO_IMU		*pFIFO_IMU;						//!< Get data and pass it into the receiver
EXTERN class FIFO_Cam		*pFIFO_Cam;						//!< Get data and pass it into the receiver
EXTERN class FIFO_Fea		*pFIFO_Fea;						//!< Get data and pass it into the receiver
EXTERN class PVT			*pPVT;							//!< Do the PVT solution
//EXTERN class Ephemeris	*pEphemeris;					//!< Extract the ephemeris
EXTERN class Acquisition	*pAcquisition;					//!< Perform acquisitions
EXTERN class Correlator		*pCorrelator;					//!< Correlator
//EXTERN class Channel		*pChannels[MAX_CHANNELS];		//!< Channels (uses correlations to close the loops)
EXTERN class SV_Select		*pSV_Select;					//!< Contains the channels and drives the channel objects
EXTERN class Telemetry		*pTelemetry;					//!< Simple ncurses interface
//EXTERN class Commando		*pCommando;						//!< Process and execute commands
EXTERN class Source_IMU		*pSource_IMU;					//!< Get the GPS data from somewhere
EXTERN class Source_Cam		*pSource_Cam;					//!< Get the GPS data from somewhere
//EXTERN class Patience		*pPatience;						//!< Watchdog for GPS Source
/*----------------------------------------------------------------------------------------------*/


/* Part 2, Pipes */
/*----------------------------------------------------------------------------------------------*/
/* Interplay between acquisition and tracking */
EXTERN int SVS_2_COR_P[2];						//!< \ingroup PIPES Send an acquisition result to the correlator to start a channel
EXTERN int FIFO_IMU_2_ACQ_P[2];		//!< \ingroup PIPES Send sensor data (IMU) to frontend
EXTERN int FIFO_CAM_2_ACQ_P[2];		//!< \ingroup PIPES Send sensor data (Cam) to frontend


EXTERN int PVT_2_TLM_P[2];						//!< \ingroup PIPES Output PVT state to Telemetry
EXTERN int SVS_2_TLM_P[2];						//!< \ingroup PIPES Output predicted SV states to Telemetry


EXTERN int ACQ_2_SVS_P[2];						//!< \ingroup PIPES Request an acquisition because some of the channels are empty



EXTERN int SVS_2_ACQ_P[2];						//!< \ingroup PIPES Request an acquisition because some of the channels are empty

EXTERN int ISRP_2_PVT_P[2];						//!< \ingroup PIPES Output measurement preamble to PVT
EXTERN int FIFO_FEA_2_COR_P[2];
EXTERN int ACQ_2_FIFO_FEA_P[2];
/*----------------------------------------------------------------------------------------------*/


/* Part 3, Anything else */
/*----------------------------------------------------------------------------------------------*/
EXTERN int64_t grun;				//!< Keep all the threads active (technically, this should be mutex protected, but eh, who cares? )
EXTERN Options_S gopt;				//!< Global receiver options
EXTERN struct timeval starttime;	//!< Get receiver start time
EXTERN int64_t source_success_IMU;	//!< Was the source created successfully?
EXTERN int64_t source_success_Cam;	//!< Was the source created successfully?
/*----------------------------------------------------------------------------------------------*/


#endif /* GLOBALS_H */
