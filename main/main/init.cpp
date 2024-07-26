



























/*----------------------------------------------------------------------------------------------*/
#include "includes.h"
#include "fifo_imu.h"				//!< Circular buffer for Importing IF data
#include "fifo_cam.h"				//!< Circular buffer for Importing IF data
#include "fifo_fea.h"				//!< Circular buffer for Importing IF data
//#include "keyboard.h"			//!< Handle user input via keyboard
//#include "channel.h"			//!< Tracking channels
#include "correlator.h"		//!< Correlator
#include "acquisition.h"		//!< Acquisition
#include "pvt.h"				//!< PVT solution
//#include "ephemeris.h"		//!< Ephemeris decode
#include "telemetry.h"		//!< Serial/GUI telemetry
//#include "commando.h"			//!< Command interface
#include "sv_select.h"		//!< Drives acquisition/reacquisition process
#include "source_imu.h"		//!< Get IMU data
#include "source_cam.h"		//!< Get Cam data
//#include "patience.h"
/*----------------------------------------------------------------------------------------------*/

/*! Print out command arguments to std_out */
/*----------------------------------------------------------------------------------------------*/
void usage(char *_str)
{
	fprintf(stdout,"\n");














	fflush(stdout);
	exit(1);
}
/*----------------------------------------------------------------------------------------------*/


/*! Print out command arguments to std_out */
/*----------------------------------------------------------------------------------------------*/
void echo_options()
{






















}
/*----------------------------------------------------------------------------------------------*/


/*! Parse command line arguments to setup functionality */
/*----------------------------------------------------------------------------------------------*/
void Parse_Arguments(int argc, char** argv)
{
	gopt.msInput_dataset_path = "/home/xin/data/Euroc_MAV/machine_hall/MH_03";
//	gopt.msInput_dataset_path = "/home/xin/data/Euroc_MAV/machine_hall/MH_03_backup";
	gopt.msImuName = "imu0";
//	gopt.msCamName = "cam0";// for mono
	gopt.msCamName = "cam";// for stereo
	gopt.source = -1;//SOURCE_IMU; currently UNUSED
	gopt.verbose = 1;
	//gopt.mvCamNames = {"cam0", "cam1"};//for stereo


	// test: 20220323
	
	std::cout<<"Self-defined types in bytes (using sizeof()):"<<std::endl
		     <<"uint8 : "<<"should be 1, now"<<sizeof(uint8)<<std::endl
		     <<"uint16: "<<"should be 2, now"<<sizeof(uint16)<<std::endl
		     //<<"uint32: "<<sizeof(uint32)<<std::endl
		     <<"uint64: "<<"should be 8, now"<<sizeof(uint64)<<std::endl
		     <<"int8  : "<<"should be 1, now"<<sizeof(int8)<<std::endl
		     <<"int16 : "<<"should be 2, now"<<sizeof(int16)<<std::endl
		     //<<"int32 : "<<sizeof(int32)<<std::endl
		     <<"int64 : "<<"should be 8, now"<<sizeof(int64)<<std::endl
		     <<"int32 : "<<"should be 4, now"<<sizeof(int32)<<std::endl
		     <<"uint32: "<<"should be 4, now"<<sizeof(uint32)<<std::endl;
	std::cout<<"Built-in types FOR union in bytes (using sizeof()):"<<std::endl
		     <<"float for cv2::Point2f("<<sizeof(cv::Point2f)<<" bytes).x or .y: "<<"should be 4, now"<<sizeof(float)<<std::endl;





















































































//	echo_options();

}
/*----------------------------------------------------------------------------------------------*/


/*! Initialize any hardware (for realtime mode) */
/*----------------------------------------------------------------------------------------------*/
int64_t Hardware_Init(void)
{

















































	return(1);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*! Initialize all threaded objects and global variables */
int64_t Object_Init(void)
{
//	size_t lcv;

	/* Get start of receiver */
//	gettimeofday(&starttime, NULL);

	/* Create Keyboard object to handle user input */
//	pKeyboard = new Keyboard();

	/* Now do the hard work? */
	pAcquisition = new Acquisition();



	/* Drive the acquisition process */
	pSV_Select = new SV_Select;


	/* Output info to the GUI */
	pTelemetry = new Telemetry();
	
	/* Serial or named pipe */
//	pTelemetry->SetType(gopt.tlm_type);

	/* execute user commands */
//	pCommando = new Commando();

	/* Form a nav solution */
	pPVT = new PVT();




	/* Get data from either the USRP/GN3S/disk */
	pFIFO_IMU = new FIFO_IMU;
	pFIFO_Cam = new FIFO_Cam;
	pFIFO_Fea = new FIFO_Fea;
	if(source_success_IMU != 0 || source_success_Cam != 0)
		return(-1);

	/* Startup Watchdog */
//	pPatience = new Patience;

	pCorrelator = new Correlator();

//	if(gopt.verbose)
	{
		fprintf(stdout,"Cleared Object Init\n");
		fflush(stdout);
	}

	return(1);

}
/*----------------------------------------------------------------------------------------------*/


/*! Initialize all pipes */
/*----------------------------------------------------------------------------------------------*/
int64_t Pipes_Init(void)
{

	int ret_val = 0;
	/* Create all of the pipes */
	ret_val = pipe((int *)FIFO_IMU_2_ACQ_P); //ACQ now takes over all processing before backend
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): FIFO_IMU_2_ACQ_P failed"<<std::endl;exit(-1);}

	ret_val = pipe((int *)FIFO_CAM_2_ACQ_P); //ACQ now takes over all processing before backend
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): FIFO_CAM_2_ACQ_P failed"<<std::endl;exit(-1);}

	ret_val = pipe((int *)ACQ_2_SVS_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): ACQ_2_SVS_P failed"<<std::endl;exit(-1);}
	
	ret_val = pipe((int *)SVS_2_ACQ_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): SVS_2_ACQ_P failed"<<std::endl;exit(-1);}
	
	ret_val = pipe((int *)SVS_2_TLM_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): SVS_2_TLM_P failed"<<std::endl;exit(-1);}
	
	ret_val = pipe((int *)PVT_2_TLM_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): PVT_2_TLM_P failed"<<std::endl;exit(-1);}
	
	ret_val = pipe((int *)ISRP_2_PVT_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): ISRP_2_PVT_P failed"<<std::endl;exit(-1);}

	ret_val = pipe((int *)ACQ_2_FIFO_FEA_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): ACQ_2_FIFO_FEA_P failed"<<std::endl;exit(-1);}

	ret_val = pipe((int *)FIFO_FEA_2_COR_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): FIFO_FEA_2_COR_P failed"<<std::endl;exit(-1);}
	
	ret_val = pipe((int *)SVS_2_COR_P);
	if (ret_val == -1) {std::cout<<"init.cpp: Pipes_Init(): SVS_2_COR_P failed"<<std::endl;exit(-1);}

	/* Setup some of the non-blocking pipes */
	fcntl(FIFO_IMU_2_ACQ_P[WRITE], F_SETFL, O_NONBLOCK);
//	fcntl(FIFO_IMU_2_ACQ_P[READ], F_SETFL, O_NONBLOCK);
	fcntl(FIFO_CAM_2_ACQ_P[WRITE], F_SETFL, O_NONBLOCK);
//	fcntl(FIFO_CAM_2_ACQ_P[READ], F_SETFL, O_NONBLOCK);
	fcntl(SVS_2_TLM_P[WRITE], F_SETFL, O_NONBLOCK);
	fcntl(SVS_2_TLM_P[READ], F_SETFL, O_NONBLOCK);
//	fcntl(PVT_2_TLM_P[WRITE],  F_SETFL, O_NONBLOCK);
	fcntl(PVT_2_TLM_P[READ],  F_SETFL, O_NONBLOCK);
//	fcntl(SVS_2_COR_P[WRITE],  F_SETFL, O_NONBLOCK);
	fcntl(SVS_2_COR_P[READ],  F_SETFL, O_NONBLOCK);
	
	fcntl(ACQ_2_FIFO_FEA_P[WRITE],  F_SETFL, O_NONBLOCK);//xin 20230513
	fcntl(FIFO_FEA_2_COR_P[WRITE], F_SETFL, O_NONBLOCK);


//	if (gopt.verbose)
	{
		fprintf(stdout,"Cleared Pipes Init\n");
		fflush(stdout);
	}

	return(1);

}
/*----------------------------------------------------------------------------------------------*/


/*! Finally start up the threads */
/*----------------------------------------------------------------------------------------------*/
int64_t Thread_Init(void)
{
//	size_t lcv;

	/* Set the global run flag to true */
	grun = 0x1;

	/* Start the keyboard thread to handle user input from stdio */
//	pKeyboard->Start();

	/* Startup the PVT sltn */
	pPVT->Start();

	/* Start up the correlators */
	pCorrelator->Start();

	/* Start up the acquistion */
	pAcquisition->Start();






	/* Start up the command interface */
//	pCommando->Start();

	/* Start the SV select thread */
	pSV_Select->Start();

	/* Last thing to do */
	pTelemetry->Start();

	/* Start up the FIFO */
	pFIFO_IMU->Start();
	pFIFO_Cam->Start();
	pFIFO_Fea->Start();
	sleep(1);

	/* Start up the Watchdog */
//	pPatience->Start();

	return(1);

}
/*----------------------------------------------------------------------------------------------*/
