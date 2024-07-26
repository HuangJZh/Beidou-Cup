



























/*----------------------------------------------------------------------------------------------*/
#include "includes.h"
#include "fifo_imu.h"
#include "fifo_cam.h"
#include "fifo_fea.h"
//#include "Keyboard.h"
//#include "channel.h"			//!< Tracking channels
#include "correlator.h"			//!< Correlator
#include "acquisition.h"		//!< Acquisition
#include "pvt.h"				//!< PVT solution
//#include "ephemeris.h"		//!< Ephemeris decode
#include "telemetry.h"
//#include "commando.h"
#include "sv_select.h"			//!< Drives acquisition/reacquisition process
#include "source_imu.h"
#include "source_cam.h"
//#include "patience.h"
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
/*! First stop all threads */
void Thread_Shutdown(void)
{

	/* Start the keyboard thread to handle user input from stdio */
//	pKeyboard->Stop();

	/* Stop the WatchDog */
//	pPatience->Stop();

	/* Stop the FIFO */
	pFIFO_Cam->Stop();
	pFIFO_IMU->Stop();
	/* Stop the telemetry */
	pTelemetry->Stop();

	/* Uh-oh */
	pPVT->Stop();

	/* Stop the correlator */
	pCorrelator->Stop();

	/* Stop the acquistion */
	pAcquisition->Stop();


	pFIFO_Fea->Stop();





	/* Stop the command interface */
//	pCommando->Stop();

	/* Stop the tracking */
	pSV_Select->Stop();

}
/*----------------------------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------------------------*/
/*! Shutdown all pipes */
void Pipes_Shutdown(void)
{

	close(SVS_2_COR_P[READ]);
	close(FIFO_CAM_2_ACQ_P[READ]);
	close(FIFO_IMU_2_ACQ_P[READ]);

	close(PVT_2_TLM_P[READ]);
	close(SVS_2_TLM_P[READ]);
		
	
	close(ACQ_2_SVS_P[READ]);
	
	
	
	close(SVS_2_ACQ_P[READ]);
	
	close(ISRP_2_PVT_P[READ]);
	close(FIFO_FEA_2_COR_P[READ]);
	close(ACQ_2_FIFO_FEA_P[READ]);

	
	close(SVS_2_COR_P[WRITE]);
	close(FIFO_CAM_2_ACQ_P[WRITE]);
	close(FIFO_IMU_2_ACQ_P[WRITE]);

	close(PVT_2_TLM_P[WRITE]);
	close(SVS_2_TLM_P[WRITE]);


	close(ACQ_2_SVS_P[WRITE]);



	close(SVS_2_ACQ_P[WRITE]);

	close(ISRP_2_PVT_P[WRITE]);
	close(FIFO_FEA_2_COR_P[WRITE]);
	close(ACQ_2_FIFO_FEA_P[WRITE]);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Object_Shutdown(void)
{
//	delete pCorrelator;

	





	delete pFIFO_Fea;
//	delete pKeyboard;
	delete pAcquisition;

	delete pFIFO_Cam;
	delete pFIFO_IMU;
	delete pSV_Select;
	delete pTelemetry;
	delete pPVT;
	delete pCorrelator;

//	delete pCommando;
//	delete pPatience;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*! Close out any hardware */
void Hardware_Shutdown(void)
{



}
/*----------------------------------------------------------------------------------------------*/
