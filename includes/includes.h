
























/*----------------------------------------------------------------------------------------------*/
// Revised by Xin Zhang 2013

/* Include standard headers, OS stuff */
/*----------------------------------------------------------------------------------------------*/
#include <unistd.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
/*----------------------------------------------------------------------------------------------*/

#include <string>
#include <Eigen/Dense>
#include <gtsam/navigation/ImuBias.h>
#include <vector>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory> //std::shared_ptr
/* Herein Lies Many Important File, note their order is important! */
/*----------------------------------------------------------------------------------------------*/

#include "config.h"			//!< Configure receiver
//#include "signaldef.h"		//!< Define attributes of input data
#include "defines.h"			//!< Defines from IS-GPS-200D and some other things
//#include "macros.h"			//!< Macros
#include "messages.h"			//!< Defines output telemetry//2023-6-20: including sdr_structs.h
//CCSDS//#include "commands.h"			//!< Defines command input
#include "structs.h"			//!< Structs used for interprocess communication

//#include "sdr_structs.h"		//!< Structs used for interprocess communication
#include "cameras.h"
#include "calibration.h"

#include "protos.h"				//!< Functions & thread prototypes
//#include "simd.h"				//!< Include the SIMD functionality
#include "globals.h"			//!< Global objects live here
#include "threaded_object.h"	//!< Base class for threaded object
//#include "se3.hpp"
//#include "faim_assert.h"
/*----------------------------------------------------------------------------------------------*/
