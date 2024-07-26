





















#include "gui_serial.h"


/*----------------------------------------------------------------------------------------------*/
#define MOD_ADLER 65521
/* Data is input buffer, len is in bytes! */
uint32 adler(uint8 *data, int32 len)
{

    uint32 a, b;
//	int32 tlen;

    a = 1; b = 0;

    if(len < 5552)
    {
        do
        {
            a += *data++;
            b += a;
        } while (--len);

        a %= MOD_ADLER;
        b %= MOD_ADLER;
    }

    return (b << 16) | a;

}
/*----------------------------------------------------------------------------------------------*/

static uint32 SIZEOF_M[LAST_M_ID + 1] =
{
	0,
	0,
/*	sizeof(Board_Health_M),
	sizeof(Task_Health_M),
*/	sizeof(SPS_M),
/*	sizeof(TOT_M),
	sizeof(PPS_M),
	sizeof(Clock_M),
	sizeof(Channel_M),
	sizeof(SV_Position_M),
	sizeof(Measurement_M),
	sizeof(Pseudorange_M),
*/	sizeof(SV_Prediction_M),
	0,
/*	sizeof(EKF_State_M),
	sizeof(EKF_Covariance_M),
	sizeof(EKF_Residual_M),
	0,
	sizeof(Command_Ack_M),
	sizeof(Ephemeris_M),
	sizeof(Almanac_M),
	sizeof(Ephemeris_Status_M),
	sizeof(SV_Select_Status_M),
	sizeof(EEPROM_M),
	sizeof(EEPROM_Chksum_M),
	sizeof(Memory_M),
	sizeof(Memory_Chksum_M),
*/	0
};

GUI_Serial *aGUI_Serial;

/*----------------------------------------------------------------------------------------------*/
/* Form the CCSDS packet header with the given _apid, sequence flag, and packet length, and command bit */
void FormCCSDSPacketHeader(CCSDS_Packet_Header *_p, uint32 _apid, uint32 _sf, uint32 _pl, uint32 _cm, uint32 _tic)
{

	_p->pid = ((_cm & 0x1) << 12) + ((_apid + CCSDS_APID_BASE) & 0x7FF);//_p->pid = ((_cm & 0x1) << 12) + (_apid + CCSDS_APID_BASE) & 0x7FF;
	_p->psc = (_sf & 0x3) + ((_tic & 0x3FFF) << 2);
	_p->pdl = _pl & 0xFFFF;

}

/* Decode a command into its components */
void DecodeCCSDSPacketHeader(CCSDS_Decoded_Header *_d, CCSDS_Packet_Header *_p)
{

	_d->id 		= _p->pid - CCSDS_APID_BASE;
	_d->type 	= _p->psc & 0x3;
	_d->tic		= (_p->psc >> 2) & 0x3FFF;
	_d->length 	= _p->pdl & 0xFFFF;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int grun;
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void *GUI_Serial_Thread(void *_arg)
{

	aGUI_Serial = (GUI_Serial *)_arg;

	while(grun)
	{
		aGUI_Serial->Import();
		aGUI_Serial->Export();
	}

	pthread_exit(0);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void lost_client(int _sig)
{
	aGUI_Serial->closePipe();
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::Start()
{
	Start_Thread(GUI_Serial_Thread, this);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
GUI_Serial::GUI_Serial()
{

	grun = 1;
	execution_tic 	= 0;
	start_tic 		= 0;
	stop_tic 		= 0;
	message_sync 	= 0;
//	command_ready 	= 0;
//	command_sent	= 1;
//	command_ack 	= 1;
//	command_count	= 666;
	byte_count 		= 0;
	serial 			= 0;
	npipe_open 		= false;
	npipe[READ] 	= -1;
	npipe[WRITE] 	= -1;
//	lfile 			= NULL;
//	gfile 			= NULL;
//	robsfile 		= NULL;
//	rephemfile 		= NULL;
	packet_count[LAST_M_ID] = 0;
	decoded_packet.tic = decoded_packet.id = decoded_packet.length = 0;
	memset(&packet_count[0], 0x0, (LAST_M_ID+1)*sizeof(int));
//	memset(&command_body, 0x0, sizeof(Command_Union));
//	memset(&command_header, 0x0, sizeof(CCSDS_Packet_Header));
//	memset(&decoded_command, 0x0, sizeof(CCSDS_Decoded_Header));
//	memset(&filepath[0], 0x0, 1024*sizeof(char));
//	memset(&log_flag[0], 0x0, LAST_M_ID*sizeof(char));
	signal(SIGPIPE, lost_client);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
GUI_Serial::~GUI_Serial()
{
//	if(lfile)
//		fclose(lfile);

	if(npipe[READ])
		close(npipe[READ]);

	if(npipe[WRITE])
		close(npipe[WRITE]);

	grun = false;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*void GUI_Serial::openSerial()
{
	int32 spipe;
	struct termios tty;

	spipe = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(spipe < 0)
    {
		npipe[READ] = -1;
		npipe[WRITE] = -1;
		npipe_open = false;
    	return;
    }

    memset(&tty, 0x0, sizeof(tty));		//!< Initialize the port settings structure to all zeros
    tty.c_cflag =  B57600 | CS8 | CLOCAL | CREAD | CRTSCTS;	//!< 8N1
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;      			//!< 0 means use-vtime
    tty.c_cc[VTIME] = 1;      			//!< Time to wait until exiting read (tenths of a second)

    tcflush(spipe, TCIFLUSH);				//!< Flush old data
    tcsetattr(spipe, TCSANOW, &tty);		//!< Apply new settings
    fcntl(spipe, F_SETFL, FASYNC);
	fcntl(spipe, F_SETFL, O_NONBLOCK);		//!< Nonblocking reads and writes

	// Alias the serial port
	npipe[READ] = npipe[WRITE] = spipe;
	npipe_open = true;
}*/
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::openPipe()
{
	npipe[READ] = npipe[WRITE] = -1;
	npipe[READ] = open("/tmp/GPS2GUI", O_RDONLY | O_NONBLOCK);
	npipe[WRITE] = open("/tmp/GUI2GPS", O_WRONLY | O_NONBLOCK);

	if((npipe[READ] != -1) && (npipe[WRITE] != -1))
	{
		fcntl(npipe[READ] , F_SETFL, O_NONBLOCK);
		fcntl(npipe[WRITE] , F_SETFL, O_NONBLOCK);
		npipe_open = true;
	}
	else
	{
		close(npipe[READ]);
		close(npipe[WRITE]);
		npipe_open = false;
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::closePipe()
{
	close(npipe[READ]);
	close(npipe[WRITE]);
	npipe_open = false;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::setPipe(bool _status)
{
	npipe_open = _status;
	if(_status == false)
	{
		close(npipe[READ]);
		close(npipe[WRITE]);
		npipe[READ] = -1;
		npipe[WRITE] = -1;
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::setIO(int32 _serial)
{

	if(npipe_open)
	{
		npipe_open = false;
		close(npipe[READ]);
		close(npipe[WRITE]);
		npipe[READ] = -1;
		npipe[WRITE] = -1;
	}

	serial = _serial;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::Import()
{

	if(npipe_open == true)
	{
		readGPS();
	}
	else
	{
/*		if(serial)
			openSerial();
		else*/
			openPipe();
	}

	execution_tic++;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::Export()
{

	/* Open the pipe */
	if(npipe_open == true)
		writeGPS();

}
/*----------------------------------------------------------------------------------------------*/































































/*----------------------------------------------------------------------------------------------*/
int GUI_Serial::Read(void *_b, int32 _bytes)
{

	int32 nbytes, bread, k;
	uint8 *buff;

	k = 0; nbytes = 0; bread = 0;
	buff = (uint8 *)_b;

	while((nbytes < _bytes) && grun && npipe_open)
	{
		bread = read(npipe[READ], buff, _bytes - nbytes);

		if(bread > -1)
		{
			nbytes += bread;
			buff += bread;
		}

		if(k++ > 5000)
		{
			closePipe();
			return(0);
		}

		usleep(100);

	}

	//endian_swap(_b, _bytes, _bytes <= 6);

	return(nbytes);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int GUI_Serial::Write(void *_b, int32 _bytes)
{

	uint32 bwrote;

	if(npipe_open)
		bwrote = write(npipe[WRITE], _b, _bytes);
	else
		bwrote = 0;

	return(bwrote);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::readGPS()
{
	uint32 checksumc, checksumr;
//	int32 nbytes, bread, k;
	int32 chan;
	uint8 v;
	Message_Struct *dst = &messages;
	Message_Union *src = &message_body;
	
	if(message_sync)
	{
		/* Check to make sure the first 4 bytes are "0xAAAAAAAA" */
		byte_count += Read(&syncword, sizeof(uint32));
		wxPrintf("\n<><<><><><><> gui_serial: syncword                                     = 0x%X\n", syncword);
		if(syncword == 0xAAAAAAAA)
		{
			message_sync++;
		}
		else
		{
			message_sync = 0;
			packet_count[LAST_M_ID]++;
			return;
		}

		/* If so get the packet header */
		byte_count += Read(&packet_header, sizeof(CCSDS_Packet_Header));
		DecodeCCSDSPacketHeader(&decoded_packet, &packet_header);
		wxPrintf("<><<><><><><> gui_serial: decoded_packet.id                            = %u\n", decoded_packet.id);
		wxPrintf("<><<><><><><> gui_serial: decoded_packet.type                          = %u\n", decoded_packet.type);
		wxPrintf("<><<><><><><> gui_serial: decoded_packet.tic                           = %u\n", decoded_packet.tic);
		wxPrintf("<><<><><><><> gui_serial: decoded_packet.length                        = %u\n", decoded_packet.length);
		/* Catch some failures */
//		if(decoded_packet.length < 0 || decoded_packet.length > 512)
//		if(decoded_packet.length < 0 || decoded_packet.length > 8182) //xin 20220422
//		if(decoded_packet.length < 0 || decoded_packet.length > 2048) //xin 20220427
//		if(decoded_packet.length < 0 || decoded_packet.length > 4096) //xin 20230530
//		if(decoded_packet.length < 0 || decoded_packet.length > 8192) //xin 20220602
//		if(decoded_packet.length < 0 || decoded_packet.length > 5096) //xin 20220603
		if(decoded_packet.length < 0 || decoded_packet.length > 8192) //xin 20220603
		{
			message_sync = 0;
			packet_count[LAST_M_ID]++;wxPrintf("Bailing out!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11");
			return;
		}

		/* Check to make sure ID is ok */
		if(decoded_packet.id < FIRST_M_ID || decoded_packet.id > LAST_M_ID)
		{
			message_sync = 0;
			packet_count[LAST_M_ID]++;
			return;
		}

		if((decoded_packet.id == FIRST_PERIODIC_M_ID) || (decoded_packet.id == LAST_PERIODIC_M_ID))
		{
			message_sync = 0;
			packet_count[LAST_M_ID]++;
			return;
		}

		/* Make sure the packet length matches the structure size */
		if(CheckPacket(&decoded_packet) == false)
		{
			message_sync = 0;
			packet_count[LAST_M_ID]++;
			return;
		}

		/* Read in the message */
		Read(src, decoded_packet.length);
		wxPrintf("<><<><><><><> gui_serial: readGPS()                                    = <><<><><><><>\n");

		/* Assemble into a complete packet for the checksum */
		memcpy(&packet_body[0], &packet_header, 6);				//!< CCSDS Header //TODO:packet_body: not only for commands ??
		memcpy(&packet_body[6], src, decoded_packet.length);	//!< Body
		checksumc = adler(&packet_body[0], 6 + decoded_packet.length);

		/* Read in the checksum */
		byte_count += Read(&checksumr, sizeof(uint32));

		/* Now verify the checksum */
		if(checksumc != checksumr)
		{   wxPrintf("<><<><><><><> gui_serial: checksumc != checksumr                       \n");
			message_sync = 0;
			packet_count[LAST_M_ID]++;
			return;
		}
		wxPrintf("<><<><><><><> gui_serial: checksumc == checksumr                       \n");

		/* Read in the postword */
		byte_count += Read(&postword, sizeof(uint32));
		wxPrintf("<><<><><><><> gui_serial: postword                                     = 0x%X\n", postword);
		if(postword != 0xBBBBBBBB)
		{
			message_sync = 0;
			packet_count[LAST_M_ID]++;
			return;
		}

		Lock();

		/* For the bytes/sec calculation */
		byte_count += decoded_packet.length;

		packet_count[decoded_packet.id]++;
		
		/* Now copy in the body */
		switch(decoded_packet.id)
		{
/*			case BOARD_HEALTH_M_ID:
				memcpy(&dst->board_health, &src->board_health, sizeof(Board_Health_M));
				if(log_flag[BOARD_HEALTH_M_ID]) printBoard();
				break;
			case TASK_HEALTH_M_ID:
				memcpy(&dst->task_health, &src->task_health, sizeof(Task_Health_M));
				if(log_flag[TASK_HEALTH_M_ID]) printTask();
				break;
			case CHANNEL_M_ID:
				chan = src->channel.chan;
				if((chan >= 0) && (chan < MAX_CHANNELS))
				{
					memcpy(&dst->channel[chan], &src->channel, sizeof(Channel_M));
					if(log_flag[CHANNEL_M_ID]) printChan(chan);
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}
				break;*/
			case SPS_M_ID:
//				if(log_flag[LAST_M_ID + 1]) printRinexObs();
//				FixDoubles((void *)&src->sps, 13);
				memcpy(&dst->sps, &src->sps, sizeof(SPS_M));
				wxPrintf("<><<><><><><> gui_serial: src->sps.tic                            = %d\n", src->sps.tic);//TODO 20220410: switch clause fails
//				if(log_flag[SPS_M_ID]) printPVT();
//				if(log_flag[LAST_M_ID]) printGoogleEarth();
				break;
/*			case CLOCK_M_ID:
				FixDoubles((void *)&src->clock, 6);
				memcpy(&dst->clock, &src->clock, sizeof(Clock_M));
				if(log_flag[CLOCK_M_ID]) printClock();
				break;
			case TOT_M_ID:
				FixDoubles((void *)&src->tot, 1);
				memcpy(&dst->tot, &src->tot, sizeof(TOT_M));
				break;
			case PPS_M_ID:
				FixDoubles((void *)&src->pps, 4);
				memcpy(&dst->pps, &src->pps, sizeof(PPS_M));
				if(log_flag[PPS_M_ID]) printPPS();
				break;
			case SV_POSITION_M_ID:
				FixDoubles((void *)&src->sv_position, 12);
				chan = src->sv_position.chan;
				if((chan >= 0) && (chan < MAX_CHANNELS))
				{
					memcpy(&dst->sv_positions[chan], &src->sv_position, sizeof(SV_Position_M));
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}
				break;
			case EKF_STATE_M_ID:
				FixDoubles((void *)&src->ekf_state, 11);
				memcpy(&dst->ekf_state, &src->ekf_state, sizeof(EKF_State_M));
				if(log_flag[EKF_STATE_M_ID]) printEKFState();
				break;
			case EKF_RESIDUAL_M_ID:
				FixDoubles((void *)&src->ekf_residual, MAX_CHANNELS);
				memcpy(&dst->ekf_residual, &src->ekf_residual, sizeof(EKF_Residual_M));
				if(log_flag[EKF_RESIDUAL_M_ID]) printEKFResidual();
				break;
			case EKF_COVARIANCE_M_ID:
				FixDoubles((void *)&src->ekf_covariance, 10);
				memcpy(&dst->ekf_covariance, &src->ekf_covariance, sizeof(EKF_Covariance_M));
				if(log_flag[EKF_COVARIANCE_M_ID]) printEKFCovariance();
				break;
			case MEASUREMENT_M_ID:
				chan = src->measurement.chan;
				if((chan >= 0) && (chan < MAX_CHANNELS))
				{
					memcpy(&dst->measurements[chan], &src->measurement, sizeof(Measurement_M));
					if(log_flag[MEASUREMENT_M_ID]) printMeas(chan);
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}
				break;
			case PSEUDORANGE_M_ID:
				FixDoubles((void *)&src->pseudorange, 6);
				chan = src->pseudorange.chan;
				if((chan >= 0) && (chan < MAX_CHANNELS))
				{
					memcpy(&dst->pseudoranges[chan], &src->pseudorange, sizeof(Pseudorange_M));
					if(log_flag[PSEUDORANGE_M_ID]) printPseudo(chan);
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}
				break;
			case EPHEMERIS_M_ID:
				FixDoubles((void *)&src->ephemeris, 24);
				chan = src->ephemeris.sv;
				if((chan >= 0) && (chan < MAX_SV))
				{
					memcpy(&dst->ephemerides[chan], &src->ephemeris, sizeof(Ephemeris_M));
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}
				break;
			case ALMANAC_M_ID:
				FixDoubles((void *)&src->almanac, 10);
				chan = src->almanac.sv;
				if((chan >= 0) && (chan < MAX_SV))
				{
					memcpy(&dst->almanacs[chan], &src->almanac, sizeof(Almanac_M));
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}
				break;
			case EPHEMERIS_STATUS_M_ID:
				memcpy(&dst->ephemeris_status, &src->ephemeris_status, sizeof(Ephemeris_Status_M));
				break;
			case SV_SELECT_STATUS_M_ID:
				memcpy(&dst->sv_select_status, &src->sv_select_status, sizeof(SV_Select_Status_M));
				break;*/
			case SV_PREDICTION_M_ID:
//				FixDoubles((void *)&src->sv_prediction, 8);
				wxPrintf("<><<><><><><> gui_serial: sizeof(src->sv_prediction)              = %ld\n", sizeof(src->sv_prediction));
				chan = src->sv_prediction.sv;
				memcpy(&dst->sv_predictions[MAX_SV], &src->sv_prediction, sizeof(SV_Prediction_M));
				if((chan >= 0) && (chan < MAX_SV))
				{
					memcpy(&dst->sv_predictions[chan], &src->sv_prediction, sizeof(SV_Prediction_M));
//					if(log_flag[SV_PREDICTION_M_ID]) printSVPred(chan);
				}
				else
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
				}

				for (size_t camID = 0; camID < 2; camID++)
				{
//					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.mCamMeas.mvTimestamp = %ld\n", src->sv_prediction.mCamMeas.mvTimestamp);//TODO 20220410: switch clause fails
//					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.miTimestamp_Cam      = %lld\n", src->sv_prediction.miTimestamp_Cam);
					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.miTimestamp_Cam      = %x     (Hex)\n", src->sv_prediction.miTimestamp_Cam[camID]);
					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.msImage              = %s\n", src->sv_prediction.msImage[camID]);
//					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.sv                   = %d\n", src->sv_prediction.sv);
//					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.tracked              = %d\n", src->sv_prediction.tracked);
//					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.predicted            = %d\n", src->sv_prediction.predicted);
					wxPrintf("<><<><><><><> gui_serial: src->sv_prediction.muCnt_un_pts         = %d\n\n", src->sv_prediction.muCnt_un_pts[camID]);
					wxPrintf("<><<><><><><> gui_serial: dst->sv_predictions[chan].muCnt_un_pts[camID] = %d\n\n", dst->sv_predictions[chan].muCnt_un_pts[camID]);
/*					for (size_t lcv = 0; lcv < src->sv_prediction.muCnt_un_pts; lcv++)
						wxPrintf("<><<><><><><> GUI_Serial: src->sv_prediction.(x,y)                = (%f, %f)\n", 
								src->sv_prediction.mfCur_un_pts_x[lcv],
								src->sv_prediction.mfCur_un_pts_y[lcv]);
*/				}
				break;
/*			case COMMAND_ACK_M_ID:
				memcpy(&dst->command_ack, &src->command_ack, sizeof(Command_Ack_M));
				command_ack = 1;
				break;*/
			default:
				message_sync = 0;
				packet_count[LAST_M_ID]++;
				break;
		}

		Unlock();

	}
	else //if(message_sync)//xin
	{
		v = 0;
		Read(&v, 1);

		syncword <<= 8;
		syncword += (uint32)v;

		if(syncword == 0xAAAAAAAA)
		{
			message_sync++;

			/* If so get the packet header */
			Read(&packet_header, sizeof(CCSDS_Packet_Header));
			DecodeCCSDSPacketHeader(&decoded_packet, &packet_header);

			if(CheckPacket(&decoded_packet) == true)
			{
				Read(&buff[0], decoded_packet.length);
				Read(&checksumr, sizeof(uint32));
				Read(&postword, sizeof(uint32));
				if(postword != 0xBBBBBBBB)
				{
					message_sync = 0;
					packet_count[LAST_M_ID]++;
					return;
				}
			}
			else
			{
				message_sync = 0;
				packet_count[LAST_M_ID]++;
				return;
			}
		}
	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
bool GUI_Serial::CheckPacket(CCSDS_Decoded_Header *_p)
{

	bool val = true;

	if((_p->id >= LAST_M_ID) || (_p->id <= FIRST_M_ID))
	{
		val = false;
	}
	else if(SIZEOF_M[_p->id] != _p->length)
	{
		val = false;
	}

	return(val);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::FixDoubles(void *_b, int32 _num)
{

//	int32 lcv;
//	double *buff = (double *)_b;

//	for(lcv = 0; lcv < _num; lcv++)
//		SWAP64(buff[lcv]);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Serial::writeGPS()
{
/*
	uint8 *sbuff;
	uint32 pre = 0xAAAAAAAA;
	uint32 post = 0xBBBBBBBB;
	uint32 checksum;

	Lock();




	sbuff = &packet_body[0];

	if(command_ready && (command_sent == 0))
	{
		// Assemble into a complete packet
		memcpy(sbuff, &pre, 4); 				sbuff += 4;		//!< Prefix
		memcpy(sbuff, &command_header, 6); 		sbuff += 6;		//!< CCSDS Header
		memcpy(sbuff, &command_body, decoded_command.length);	sbuff += decoded_command.length; //!< Payload
		checksum = adler(&packet_body[4], 6 + decoded_command.length);
		memcpy(sbuff, &checksum, 4); 			sbuff += 4;		//!< Checksum
		memcpy(sbuff, &post, 4); 				sbuff += 4;		//!< Postfix

		Write(&packet_body[0], decoded_command.length + 18);
		command_sent = 1;
	}

	Unlock();
*/
}
/*----------------------------------------------------------------------------------------------*/


