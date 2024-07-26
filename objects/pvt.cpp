























/*----------------------------------------------------------------------------------------------*/

#include "pvt.h"

/*----------------------------------------------------------------------------------------------*/
void *PVT_Thread(void *_arg)
{

	while(grun)
	{
		pPVT->Import();
		pPVT->Navigate();
		pPVT->Export();
	}

	pthread_exit(0);//xin
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void PVT::Start()
{

	Start_Thread(PVT_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"PVT thread started\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
PVT::~PVT()
{
//	if(gopt.verbose)
		fprintf(stdout,"Destructing PVT\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
PVT::PVT():Threaded_Object("PVT333TASK")
{

	object_mem = this;
	size = sizeof(PVT);

	Reset();

//	master_nav.stale_ticks = STALE_SPS_VALUE; //TODO TODO

//	if(gopt.verbose)
		fprintf(stdout,"Creating PVT\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void PVT::Import()
{
	ssize_t bread;




	/* Get number of channels coming off the pipe */
	bread = read(ISRP_2_PVT_P[READ], &preamble, sizeof(Preamble_2_PVT_S)); if (bread != sizeof(Preamble_2_PVT_S)) std::cout<<"PVT::Import():read(ISRP_2_PVT_P[READ], &preamble, sizeof(Preamble_2_PVT_S)) failed: "<<strerror(errno)<<std::endl;
	Lock();































































	/* Make sure the pipes are EMPTY */
//	PipeCheck();

	IncStartTic();

}
/*----------------------------------------------------------------------------------------------*/













































/*----------------------------------------------------------------------------------------------*/
void PVT::Export()
{	
	ssize_t bwrite;//xin	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//	std::cout<<"(((((((((((((((((((((((((((((((( pvt.cpp: Export() ))))))))))))))))))))))))))))))))"<<std::endl;
	/* Note, this is some shady shit done to save SRAM !*/
	memcpy(&tlm_s.sps, 			&master_nav, 		sizeof(SPS_M));
	//std::cout<<" PVT: _master_nav.tic                    = "<<master_nav.tic<<" <><<><><><><>"<<std::endl;
	






	bwrite = write(PVT_2_TLM_P[WRITE], &tlm_s, sizeof(PVT_2_TLM_S)); if (bwrite != sizeof(PVT_2_TLM_S)) std::cout<<" PVT::Export():write(PVT_2_TLM_P[WRITE], &tlm_s, sizeof(PVT_2_TLM_S)) failed: "<<strerror(errno)<<std::endl;




	
	Unlock();

	IncStopTic();

	IncExecTic();
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void PVT::Navigate()
{
	
	
	
	
	/* Always tag nav sltn with current tic */
	master_nav.tic = preamble.tic_measurement;








































































}
/*----------------------------------------------------------------------------------------------*/


//xin: line 1289 !!!!!!!!!!1
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
void PVT::Reset()
{



	/* Reset to center of the earth	 */
//	memset(&master_clock,0x0,sizeof(Clock_M));
	memset(&master_nav,0x0,sizeof(SPS_M));
//	memset(&temp_nav,0x0,sizeof(SPS_M));

	/* Reset Each Channel */
//	for(lcv = 0; lcv < MAX_CHANNELS; lcv++)
//		ResetChannel(lcv);

//	master_nav.stale_ticks = STALE_SPS_VALUE; TODO TODO

}
/*----------------------------------------------------------------------------------------------*/
