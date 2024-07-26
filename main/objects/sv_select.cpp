/*----------------------------------------------------------------------------------------------*/
























/*----------------------------------------------------------------------------------------------*/

#include "sv_select.h"

/*----------------------------------------------------------------------------------------------*/
void *SV_Select_Thread(void *_arg)
{
	while(grun)
	{
		pSV_Select->Import();
	}

	pthread_exit(0);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void SV_Select::Start()
{
	/* With new priority specified */
	Start_Thread(SV_Select_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"SV_Select thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
SV_Select::SV_Select():Threaded_Object("SVS333TASK")
{

	strong_sv = 0;
	type = ACQ_TYPE_STRONG;

		fprintf(stdout,"Creating SV Select\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
SV_Select::~SV_Select()
{
//	if(gopt.verbose)
		fprintf(stdout,"Destructing SV Select\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void SV_Select::Import()
{
	{
		Lock();
		Acquire();
		Unlock();
	}
//	    std::cout<<"sv_select.cpp:Import()"<<std::endl;	
	IncExecTic();

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void SV_Select::Export(int _sv)//void SV_Select::Export(int32 _sv)
{
	ssize_t bwrite;

	sv_prediction[_sv].sv = _sv;

	/* Dump prediction to SV Select */
//	std::cout<<"sv_select.cpp[5]:   before SVS_2_TLM_P "<<std::endl;
	bwrite = write(SVS_2_TLM_P[WRITE], &sv_prediction[_sv], sizeof(SVS_2_TLM_S));
//	std::cout<<"sv_select.cpp[6]:  after SVS_2_TLM_P "<<std::endl;
	if (bwrite == -1) std::cout<<"SV_Select::Export():write(SVS_2_TLM_P[WRITE], &sv_prediction[_sv], sizeof(SVS_2_TLM_S)) failed: "<<strerror(errno)<<std::endl;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void SV_Select::Acquire()
{
	ssize_t bread, bwrite;//xin

	int32 current_sv;
	int32 doacq;

	current_sv = strong_sv;

	// Update SV's predicted state
	doacq = SetupRequest(current_sv);

	// Do something with acquisition
//	if(doacq && (already == 666) && (chan != 666))i
	if (doacq)
	{
		// Send to the acquisition thread
//	    std::cout<<"sv_select.cpp[1]:  before SVS_2_ACQ_P "<<std::endl;
		bwrite = write(SVS_2_ACQ_P[WRITE], &command, sizeof(Acq_Command_S)); if (bwrite == -1) std::cout<<"SV_Select::Acquire():write(SVS_2_ACQ_P[WRITE], &command, sizeof(Acq_Command_S)) failed: "<<strerror(errno)<<std::endl;
//	    std::cout<<"sv_select.cpp[2]:  after  SVS_2_ACQ_P "<<std::endl;

		// Wait for acq to return, do stuff depending on the state //xin: read img data from acq to svs; then svs to tlm, for visualization only
//	    std::cout<<"sv_select.cpp[3]:  before  ACQ_2_SVS_P "<<std::endl;
		bread = read(ACQ_2_SVS_P[READ], &command, sizeof(Acq_Command_S)); if (bread == -1) std::cout<<"SV_Select::Acquire():read(ACQ_2_SVS_P[READ], &command, sizeof(Acq_Command_S)) failed: "<<strerror(errno)<<std::endl;
//	    std::cout<<"sv_select.cpp[4]:  after  ACQ_2_SVS_P "<<std::endl;
	    for (size_t camID = 0; camID < 2; camID++)
		{
			memcpy(&sv_prediction[current_sv].miTimestamp_Cam[camID],	&command.miTimestamp_Cam[camID],	sizeof(int64_t));
			memcpy(&sv_prediction[current_sv].msImage[camID][0],		&command.msImage[camID][0],			128*sizeof(char));
			memcpy(&sv_prediction[current_sv].mfCur_un_pts_x[camID][0], &command.mfCur_un_pts_x[camID][0],	command.muCnt_un_pts[camID] * sizeof(float));
			memcpy(&sv_prediction[current_sv].mfCur_un_pts_y[camID][0], &command.mfCur_un_pts_y[camID][0],	command.muCnt_un_pts[camID] * sizeof(float));
			memcpy(&sv_prediction[current_sv].muCnt_un_pts[camID],		&command.muCnt_un_pts[camID],		sizeof(uint32));
//xin 20230803			std::cout<<"SV_Select: muCnt_un_pts["<<camID<<"]="<<command.muCnt_un_pts[camID]<<std::endl;
			memcpy(&sv_prediction[current_sv].mvIds[camID][0],			&command.mvIds[camID][0],			command.muCnt_un_pts[camID] * sizeof(size_t));
		}
/*		//for the moment (as of 20220323), correlator.cpp and channel.cpp are deliberately shut off.
		if(command.success)
		{
			bwrite = write(SVS_2_COR_P[WRITE], &command, sizeof(Acq_Command_S));
			if (bwrite == -1) std::cout<<"SV_Select::Acquire():write(SVS_2_COR_P[WRITE], &command, sizeof(Acq_Command_S)) failed: "<<strerror(errno)<<std::endl;
		}
*/	}

	// Dump state info
	Export(current_sv);

	// Increment SV
	UpdateState();

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
size_t SV_Select::SetupRequest(int32 _sv)
{

	size_t return_val;


	IncStartTic();

	return_val = true;

	/* Initialize acquisition command parameters */

	command.type 		= type;
	command.sv 			= _sv;
	command.success		= false;













	SV_Predict(_sv);

	IncStopTic();

	return(return_val);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void SV_Select::UpdateState()
{
	if(type == ACQ_TYPE_STRONG)
		strong_sv = (strong_sv + 1) % MAX_SV;
}
/*----------------------------------------------------------------------------------------------*/




/*----------------------------------------------------------------------------------------------*/
void SV_Select::SV_Predict(int _sv)
{
	SV_Prediction_M *ppred;

	ppred = &sv_prediction[_sv];
	ppred->sv 	= _sv;
	ppred->predicted = true;
}
/*----------------------------------------------------------------------------------------------*/
