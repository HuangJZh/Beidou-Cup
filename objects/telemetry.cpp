
























/*----------------------------------------------------------------------------------------------*/

#include "telemetry.h"

/*----------------------------------------------------------------------------------------------*/
void lost_gui_pipe(int _sig)
{
	pTelemetry->ClosePipe();
	fprintf(stderr,"GUI disconnected\n");
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
void *Telemetry_Thread(void *_arg)
{

	while(grun)
	{
		pTelemetry->Import();
		pTelemetry->Export();
		usleep(10000);
	}

	pthread_exit(0);//xin
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Telemetry::Start()
{
	Start_Thread(Telemetry_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"Telemetry thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
Telemetry::Telemetry():Threaded_Object("TLM333TASK")
{

	object_mem = this;
	size = sizeof(Telemetry);

	npipe_open = false;
	npipe[READ] = npipe[WRITE] = -1;
//	pheader = (uint8 *)&command_header;
//	pcommand = (uint8 *)&command_body;
	pchecksumr = (uint8 *)&checksumr;

	signal(SIGPIPE, lost_gui_pipe);

	remove("/tmp/GPS2GUI");
	fifo[WRITE] = mkfifo("/tmp/GPS2GUI", S_IRWXU|S_IRWXG|S_IRWXO|S_IROTH|S_IWOTH);
	if(fifo[WRITE] == -1)
		fprintf(stderr,"Error creating the named pipe /tmp/GPS2GUI\n");
	chmod("/tmp/GPS2GUI", S_IRWXU|S_IRWXG|S_IRWXO|S_IROTH|S_IWOTH);


	remove("/tmp/GUI2GPS");
	fifo[READ] = mkfifo("/tmp/GUI2GPS", S_IRWXU|S_IRWXG|S_IRWXO|S_IROTH|S_IWOTH);
	if(fifo[READ] == -1)
		fprintf(stderr,"Error creating the named pipe /tmp/GUI2GPS\n");
	chmod("/tmp/GUI2GPS", S_IRWXU|S_IRWXG|S_IRWXO|S_IROTH|S_IWOTH);

/*	std::cout<<"sizeof(FIRST_M_ID)          = "<<0<<std::endl;
	std::cout<<"sizeof(FIRST_PERIODIC_M_ID) = "<<0<<std::endl;
	std::cout<<"sizeof(BOARD_HEALTH_M_ID)   = "<<sizeof(Board_Health_M)<<std::endl;
	std::cout<<"sizeof(Task_Health_M_ID)    = "<<sizeof(Task_Health_M)<<std::endl;
*/	std::cout<<"sizeof(SPS_M_ID)            = "<<sizeof(SPS_M)<<std::endl;
/*	std::cout<<"sizeof(TOT_M_ID)          	= "<<sizeof(TOT_M)<<std::endl;
	std::cout<<"sizeof(PPS_M_ID)            = "<<sizeof(PPS_M)<<std::endl;
	std::cout<<"sizeof(Clock_M_ID)          = "<<sizeof(Clock_M)<<std::endl;
	std::cout<<"sizeof(Channel_M_ID)        = "<<sizeof(Channel_M)<<std::endl;
	std::cout<<"sizeof(SV_Position_M_ID)    = "<<sizeof(SV_Position_M)<<std::endl;
	std::cout<<"sizeof(Measurement_M_ID)    = "<<sizeof(Measurement_M)<<std::endl;
	std::cout<<"sizeof(Pseudorange_M_ID)    = "<<sizeof(Pseudorange_M)<<std::endl;
*/	std::cout<<"sizeof(SV_Prediction_M_ID)  = "<<sizeof(SV_Prediction_M)<<std::endl;
/*	std::cout<<"sizeof(LAST_PERIODIC_M_ID)  = "<<0<<std::endl;
	std::cout<<"sizeof(EKF_State_M_ID)      = "<<sizeof(EKF_State_M)<<std::endl;
	std::cout<<"sizeof(EKF_Covariance_M_ID) = "<<sizeof(EKF_Covariance_M)<<std::endl;
	std::cout<<"sizeof(EKF_Residual_M_ID)   = "<<sizeof(EKF_Residual_M)<<std::endl;
	std::cout<<"sizeof(COMMAND_M_ID)        = "<<0<<std::endl;
	std::cout<<"sizeof(Command_Ack_M_ID)    = "<<sizeof(Command_Ack_M)<<std::endl;
	std::cout<<"sizeof(Ephemeris_M_ID)      = "<<sizeof(Ephemeris_M)<<std::endl;
	std::cout<<"sizeof(Almanac_M_ID)        = "<<sizeof(Almanac_M)<<std::endl;
	std::cout<<"sizeof(Ephemeris_Status_M_ID)= "<<sizeof(Ephemeris_Status_M)<<std::endl;
	std::cout<<"sizeof(SV_Select_Status_M_ID)= "<<sizeof(SV_Select_Status_M)<<std::endl;
	std::cout<<"sizeof(EEPROM_M_ID)         = "<<sizeof(EEPROM_M)<<std::endl;
	std::cout<<"sizeof(EEPROM_Chksum_M_ID)  = "<<sizeof(EEPROM_Chksum_M)<<std::endl;
	std::cout<<"sizeof(Memory_M_ID)         = "<<sizeof(Memory_M)<<std::endl;
	std::cout<<"sizeof(Memory_Chksum_M_ID)  = "<<sizeof(Memory_Chksum_M)<<std::endl;
	std::cout<<"sizeof(LAST_PERIODIC_M_ID)  = "<<0<<std::endl;
*/
	/* Build the function pointer array */
/*	msg_handlers[FIRST_M_ID] 				= NULL;
	msg_handlers[FIRST_PERIODIC_M_ID]		= NULL;
	msg_handlers[BOARD_HEALTH_M_ID] 		= &Telemetry::SendBoardHealth;
	msg_handlers[TASK_HEALTH_M_ID] 			= &Telemetry::SendTaskHealth;
*/	msg_handlers[SPS_M_ID] 					= &Telemetry::SendSPS;
/*	msg_handlers[TOT_M_ID] 					= &Telemetry::SendTOT;
	msg_handlers[PPS_M_ID] 					= &Telemetry::SendPPS;
	msg_handlers[CLOCK_M_ID] 				= &Telemetry::SendClock;
	msg_handlers[CHANNEL_M_ID] 				= &Telemetry::SendChannelHealth;
	msg_handlers[SV_POSITION_M_ID] 			= &Telemetry::SendSVPositions;
	msg_handlers[MEASUREMENT_M_ID] 			= &Telemetry::SendMeasurements;
	msg_handlers[PSEUDORANGE_M_ID] 			= &Telemetry::SendPseudoranges;
*/	msg_handlers[SV_PREDICTION_M_ID] 		= &Telemetry::SendSVPredictions;
/*	msg_handlers[LAST_PERIODIC_M_ID]		= NULL;
	msg_handlers[EKF_STATE_M_ID] 			= &Telemetry::SendEKFState;
	msg_handlers[EKF_COVARIANCE_M_ID] 		= &Telemetry::SendEKFCovariance;
	msg_handlers[EKF_RESIDUAL_M_ID] 		= &Telemetry::SendEKFResidual;
	msg_handlers[COMMAND_M_ID] 				= NULL;
	msg_handlers[COMMAND_ACK_M_ID] 			= NULL;
	msg_handlers[EPHEMERIS_M_ID] 			= NULL;
	msg_handlers[ALMANAC_M_ID] 				= NULL;
	msg_handlers[EPHEMERIS_STATUS_M_ID]		= NULL;
	msg_handlers[SV_SELECT_STATUS_M_ID]		= NULL;
	msg_handlers[EEPROM_M_ID]				= NULL;
	msg_handlers[EEPROM_CHKSUM_M_ID] 		= NULL;
	msg_handlers[MEMORY_M_ID] 				= NULL;
	msg_handlers[MEMORY_CHKSUM_M_ID] 		= NULL;
	msg_handlers[LAST_M_ID] 				= NULL;
*/
	/* Build the message rates */
/*	msg_rates[FIRST_M_ID] 				= 0;
	msg_rates[FIRST_PERIODIC_M_ID]		= 0;
	msg_rates[BOARD_HEALTH_M_ID] 		= 1;
	msg_rates[TASK_HEALTH_M_ID] 		= 1;
*/	msg_rates[SPS_M_ID] 				= 1;
/*	msg_rates[TOT_M_ID] 				= 1;
	msg_rates[PPS_M_ID] 				= 1;
	msg_rates[CLOCK_M_ID] 				= 1;
	msg_rates[CHANNEL_M_ID] 			= 1;
	msg_rates[SV_POSITION_M_ID] 		= 0;
	msg_rates[MEASUREMENT_M_ID] 		= 0;
	msg_rates[PSEUDORANGE_M_ID] 		= 1;
*/	msg_rates[SV_PREDICTION_M_ID] 		= 1;
/*	msg_rates[LAST_PERIODIC_M_ID]		= 0;
	msg_rates[EKF_STATE_M_ID] 			= 1;
	msg_rates[EKF_COVARIANCE_M_ID] 		= 1;
	msg_rates[EKF_RESIDUAL_M_ID] 		= 1;
	msg_rates[COMMAND_M_ID] 			= 0;
	msg_rates[COMMAND_ACK_M_ID] 		= 0;
	msg_rates[EPHEMERIS_M_ID] 			= 0;
	msg_rates[ALMANAC_M_ID] 			= 0;
	msg_rates[EPHEMERIS_STATUS_M_ID]	= 0;
	msg_rates[SV_SELECT_STATUS_M_ID]	= 0;
	msg_rates[EEPROM_M_ID]				= 0;
	msg_rates[EEPROM_CHKSUM_M_ID] 		= 0;
	msg_rates[MEMORY_M_ID] 				= 0;
	msg_rates[MEMORY_CHKSUM_M_ID] 		= 0;
	msg_rates[LAST_M_ID] 				= 0;

	if(gopt.verbose)*/
		fprintf(stdout,"Creating Telemetry\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
Telemetry::~Telemetry()
{

	close(npipe[READ]);
	close(npipe[WRITE]);
	remove("/tmp/GPS2GUI");
	remove("/tmp/GUI2GPS");

//	if(gopt.verbose)
		fprintf(stdout,"Destructing Telemetry\n");

}
/*----------------------------------------------------------------------------------------------*/


















































/*----------------------------------------------------------------------------------------------*/
void Telemetry::OpenPipe()
{

	npipe[READ] = npipe[WRITE] = -1;
	npipe[READ] = open("/tmp/GUI2GPS", O_RDONLY | O_NONBLOCK);
	npipe[WRITE] = open("/tmp/GPS2GUI", O_WRONLY | O_NONBLOCK);

	if((npipe[READ] != -1) && (npipe[WRITE] != -1))
	{
		fcntl(npipe[READ] , F_SETFL, O_NONBLOCK);
		fcntl(npipe[WRITE] , F_SETFL, O_NONBLOCK);
		npipe_open = true;
		fprintf(stdout,"GUI connected\n");
	}
	else
	{
		npipe_open = false;
		close(npipe[READ]);
		close(npipe[WRITE]);
		npipe[READ] = -1;
		npipe[WRITE] = -1;
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Telemetry::ClosePipe()
{
	npipe_open = false;
	close(npipe[READ]);
	close(npipe[WRITE]);
	npipe[READ] = -1;
	npipe[WRITE] = -1;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Telemetry::Import()
{

	/* Wait for data from the PVT */
	ImportPVT();

	/* Get data from EKF */
//	ImportEKF();

	/* Get data from serial port */
//	ImportSerial();

	/* Bent pipe anything from Commando */
//	ImportCommando();

	/* Increment execution counter */
	IncExecTic();

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Telemetry::ImportPVT()
{

	uint32 bread, sv;

	bread = read(PVT_2_TLM_P[READ], &pvt_s, sizeof(PVT_2_TLM_S));
//	std::cout<<"<><<><><><><> Telemetry1: pvt_s.sps.tic                = "<<pvt_s.sps.tic<<" <><<><><><><>"<<std::endl;
	if(bread == sizeof(PVT_2_TLM_S))
	{
//		std::cout<<"telemetry.cpp: bread = read(PVT_2_TLM_P[READ], &pvt_s, sizeof(PVT_2_TLM_S));  SUCCESS "<<std::endl;
		export_messages = true;
		if(npipe_open == false)
		{
//			if(tlm_type == TELEM_NAMED_PIPE)
			{
				OpenPipe();
			}
/*			else
			{
				OpenSerial();
			}*/
		}
	}//else std::cout<<"telemetry.cpp: bread = read(PVT_2_TLM_P[READ], &pvt_s, sizeof(PVT_2_TLM_S));  FAILED: "<<strerror(errno)<<std::endl;

	bread = read(SVS_2_TLM_P[READ], &svs_s, sizeof(SVS_2_TLM_S));
//	std::cout<<"<><<><><><><> Telemetry2: svs_s.miTimestamp_Cam        = "<<svs_s.miTimestamp_Cam<<" <><<><><><><>"<<std::endl;
	if(bread == sizeof(SVS_2_TLM_S))
	{
		sv = svs_s.sv;
		if((sv >= 0) && (sv < MAX_SV))
		{
			memcpy(&sv_predictions[sv], &svs_s, sizeof(SVS_2_TLM_S));
			//std::cout<<"<><<><><><><> Telemetry3: sv_predictions[sv].miTimestamp_Cam = "<<sv_predictions[sv].miTimestamp_Cam<<" <><<><><><><>"<<std::endl<<std::endl;
//			std::cout<<"<><<><><><><> Telemetry4: sv_predictions[sv].msImage         = "<<sv_predictions[sv].msImage<<" <><<><><><><>"<<std::endl;
		}
	}//else std::cout<<"telemetry.cpp: bread = read(SVS_2_TLM_P[READ], &pvt_s, sizeof(SVS_2_TLM_S));  FAILED: "<<strerror(errno)<<std::endl;
}
/*----------------------------------------------------------------------------------------------*/





























































































































































































/*----------------------------------------------------------------------------------------------*/
void Telemetry::Export()
{

	int32 lcv;

	/* Dump EKF ASAP */
/*	if(export_ekf == true)
	{
		(this->*msg_handlers[EKF_STATE_M_ID])();
		(this->*msg_handlers[EKF_COVARIANCE_M_ID])();
		(this->*msg_handlers[EKF_RESIDUAL_M_ID])();
		export_ekf = false;
	}
*/
	/* Now transmit the normal once/pvt stuff */
	if(export_messages == true)
	{
		export_tic++;

		/* Get the start of execution */
		IncStartTic();

		/* Dump info */
		for(lcv = FIRST_PERIODIC_M_ID; lcv < LAST_PERIODIC_M_ID; lcv++)
		{
			if((msg_handlers[lcv] != NULL) && (msg_rates[lcv] != 0))
			{
				if((export_tic % msg_rates[lcv]) == 0)
				{
					(this->*msg_handlers[lcv])();
				}
			}
		}

		export_messages = false;

		/* Get the stop of execution */
		IncStopTic();

		last_start_tic = start_tic;
		last_stop_tic = stop_tic;
	}

}
/*----------------------------------------------------------------------------------------------*/





































































































































































/*----------------------------------------------------------------------------------------------*/
void Telemetry::SendSPS()
{

	/* Form the packet */
	FormCCSDSPacketHeader(&packet_header, SPS_M_ID, 0, sizeof(SPS_M), 0, packet_tic++);

	/* Emit the packet */
	EmitCCSDSPacket((void *)&pvt_s.sps, sizeof(SPS_M));

}
/*----------------------------------------------------------------------------------------------*/


























































































/*----------------------------------------------------------------------------------------------*/
void Telemetry::SendSVPredictions()
{

	int32 sv = svs_s.sv; 

//xin: 20230514	//sv = export_tic % MAX_SV; //TODO: this might be a glitch when adding feature tracking in this code

	/* Form the packet */
	FormCCSDSPacketHeader(&packet_header, SV_PREDICTION_M_ID, 0, sizeof(SV_Prediction_M), 0, packet_tic++);

	/* Emit the packet */
	//sv_predictions[sv].sv = sv;
	EmitCCSDSPacket((void *)&sv_predictions[sv], sizeof(SV_Prediction_M)); 
			//std::cout<<"<><<><><><><> Telemetry5: sv_predictions[sv].miTimestamp_Cam = "<<sv_predictions[sv].miTimestamp_Cam<<" <><<><><><><>"<<std::endl;
			//std::cout<<"<><<><><><><> Telemetry6: sv_predictions[sv].msImage         = "<<sv_predictions[sv].msImage<<" <><<><><><><>"<<std::endl;
			//std::cout<<"<><<><><><><> Telemetry7: sv_predictions[sv].muCnt_un_pts[CAM0] = "<<sv_predictions[sv].muCnt_un_pts[CAM0]<<" <><<><><><><>"<<std::endl;
}
/*----------------------------------------------------------------------------------------------*/



































































































/*----------------------------------------------------------------------------------------------*/
void Telemetry::EmitCCSDSPacket(void *_buff, int32 _len)
{
	ssize_t bwrite;//xin
	int32 lcv;
	uint8 *sbuff;
	uint32 pre = 0xAAAAAAAA;
	uint32 post = 0xBBBBBBBB;
	uint32 checksum;

	sbuff = &packet_body[0];

	/* Assemble into a complete packet */
	memcpy(sbuff, &pre, 4); 				sbuff += 4;		//!< Prefix
	memcpy(sbuff, &packet_header, 6); 		sbuff += 6;		//!< CCSDS Header
	memcpy(sbuff, _buff, _len); 			sbuff += _len;	//!< Payload
	checksum = adler(&packet_body[4], 6 + _len);
	memcpy(sbuff, &checksum, 4); 			sbuff += 4;		//!< Checksum
	memcpy(sbuff, &post, 4); 				sbuff += 4;		//!< Postfix

	/* Write the sync header to the UART */
	if(npipe_open)
	{
		/* Now write the body to the UART */
		sbuff = (uint8 *)&packet_body[0];
		for(lcv = 0; lcv < _len + 18; lcv++)
		{
			bwrite = write(npipe[WRITE], sbuff, sizeof(uint8)); if (bwrite != sizeof(uint8)) std::cout<<"Telemetry::EmitCCSDSPacket():write(npipe[WRITE], sbuff, sizeof(uint8)) failed: "<<strerror(errno)<<std::endl;
			sbuff++;
		}
	}

}
/*----------------------------------------------------------------------------------------------*/
