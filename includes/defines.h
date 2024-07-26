


























#ifndef DEFINES_H_
#define DEFINES_H_

/* For unnamed pipes */
/*----------------------------------------------------------------------------------------------*/
#define READ	(0)									//!< Read handle
#define WRITE	(1)									//!< Write handle
//#define TRUE	(1)									//!< True?
//#define FALSE	(0)									//!< False!
/*----------------------------------------------------------------------------------------------*/

/* First make some type prototypes */
/*----------------------------------------------------------------------------------------------*/
typedef unsigned char		uint8;					//!< Unsigned byte
typedef unsigned short		uint16;					//!< Unsigned word
//typedef unsigned long		uint32;					//!< Unsigned double word
//xin: defined in /opt/ros/.../opencv-3.3.1-dev//typedef unsigned long long	uint64;					//!< Unsigned quadruple word
typedef signed char			int8;					//!< Signed byte
typedef signed short		int16;					//!< Signed word
//typedef signed long		int32;					//!< Signed double word
//xin: defined in /opt/ros/.../opencv-3.3.1-dev//typedef signed long long	int64;					//!< Signed quadruple word
typedef signed int			int32;
typedef unsigned int		uint32;
/*----------------------------------------------------------------------------------------------*/


















































































/* PVT Stuff */
/*----------------------------------------------------------------------------------------------*/


























#define SAMPS_MS				(1)//(2048)						//!< All incoming signals are resampled to this sampling frequency
/*----------------------------------------------------------------------------------------------*/

/* printing Stuff xin 0517*/
/*----------------------------------------------------------------------------------------------*/
//#define DEBUG_GETMEASUREMENTS_

#endif //DEFINES_H_
