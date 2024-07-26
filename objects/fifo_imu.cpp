
























/*----------------------------------------------------------------------------------------------*/

#include "fifo_imu.h"
#include <iostream>
/*----------------------------------------------------------------------------------------------*/
void *FIFO_IMU_Thread(void *_arg)
{

	FIFO_IMU *aFIFO = pFIFO_IMU;

	/* This thread must be cancellable by the watchdog */
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	while(grun)
	{
		aFIFO->Import();
		aFIFO->IncExecTic();
	}

	pthread_exit(0);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_IMU::Start()
{

	Start_Thread(FIFO_IMU_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"FIFO_IMU thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
FIFO_IMU::FIFO_IMU():Threaded_Object("FIFIMUTASK")
{

	int64_t lcv;

	/* Create the buffer */
	buff = new ms_packet_imu[FIFO_IMU_DEPTH];
	memset(buff, 0x0, sizeof(ms_packet_imu)*FIFO_IMU_DEPTH);
	head = &buff[0];
	tail = &buff[0];
	miFront = 0;
	miBack = FIFO_IMU_DEPTH - 1;

	/* Create circular linked list */
	for (lcv = 0; lcv < FIFO_IMU_DEPTH-1; lcv++)
		buff[lcv].mpNext = &buff[lcv+1];

	buff[FIFO_IMU_DEPTH-1].mpNext = &buff[0]; //xin this is ONLY for circular buffer : head = tail

	miTic = miCount = miSize = 0;

	int ret_val;
	ret_val = sem_init(&mtSem_full, 0, 0); if (ret_val == -1) std::cout<<"FIFO_IMU::FIFO_IMU(): sem_init(&mtSem_full, 0, 0) failed: "<<strerror(errno)<<std::endl;
	ret_val = sem_init(&mtSem_empty, 0, FIFO_IMU_DEPTH); if (ret_val == -1) std::cout<<"FIFO_IMU::FIFO_IMU(): sem_init(&mtSem_empty, 0, FIFO_IMU_DEPTH) failed: "<<strerror(errno)<<std::endl;

	pSource_IMU = NULL;
//	fp_out = NULL;
	ResetSource();

//	if(gopt.verbose)
		fprintf(stdout, "Creating FIFO_IMU\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
FIFO_IMU::~FIFO_IMU()
{


	sem_destroy(&mtSem_full);
	sem_destroy(&mtSem_empty);

	delete [] buff;

	if(pSource_IMU != NULL)
		delete pSource_IMU;

//	if(gopt.verbose)
		fprintf(stdout,"Destructing FIFO_IMU\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_IMU::Import()
{

	IncStartTic();

	/* Read from the GPS source */
	if(pSource_IMU != NULL)
		pSource_IMU->Read(head);//pSource->Read(head);
/*	std::cout<<"FIFO_IMU::Import(): head->mpNext               = "<<head->mpNext<<std::endl;
	std::cout<<"FIFO_IMU::Import(): head->mData[0].mvTimestamp = "<<head->mData[0].mvTimestamp<<std::endl;
*///	std::cout<<"FIFO_IMU::Import(): head->mData[0].miTest      = "<<head->mData[0].miTest<<std::endl;
//	std::cout<<"FIFO_IMU::Import(): head->mData[0].msImage     = "<<head->mData[0].msImage<<std::endl;

	Enqueue();

	IncStopTic();

	miCount++;
//	fprintf(stdout,"miCount: %ld \n\n\n", miCount);//TODO: only one printer of miCount (either in fifo_imu.cpp or fifo_cam.cpp)!!
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_IMU::Enqueue()
{
	sem_wait(&mtSem_empty);

	head->miCount = miCount;
//	std::cout<<"FIFO_IMU::Import(): head->mData[0].miTest      = "<<head->mData[0].miTest<<std::endl;
	int data_size = sizeof(ms_packet_imu);

//	std::cout<<"-----------------------FIFO_IMU::Enqueue(): Hey I am here------------------"<<std::endl;
	/* Send a packet to the acquisition (nonblocking) */
	ssize_t ret_val = write(FIFO_IMU_2_ACQ_P[WRITE], head, data_size);
	if (ret_val != data_size)
	{
		//std::cout<<"FIFO_IMU::Enqueue(): write(FIFO_IMU_2_ACQ_P) failed: "<<strerror(errno)<<std::endl;
//xin 0502//		exit(-1);
	}

//	std::cout<<"***********************FIFO_IMU::Enqueue(): Hey I am here********************"<<std::endl;
	head = head->mpNext;
	miBack = (miBack + 1) % FIFO_IMU_DEPTH;

	sem_post(&mtSem_full);
	miSize++;//xin 0505
//	fprintf(stdout,"FIFO_IMU::Enqueue()\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_IMU::Dequeue(ms_packet_imu *p)
{

	sem_wait(&mtSem_full);

	memcpy(p, tail, sizeof(ms_packet_imu));
	tail = tail->mpNext;
	miFront = (miFront + 1) % FIFO_IMU_DEPTH;

	sem_post(&mtSem_empty);

	miSize--;//xin 0505
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_IMU::ResetSource()
{

	if(pSource_IMU != NULL)
	{
		delete pSource_IMU;
		pSource_IMU = new Source_IMU(); //pSource = new GPS_Source(&gopt);
	}
	else
	{
		pSource_IMU = new Source_IMU(); //pSource = new GPS_Source(&gopt);
	}

}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
bool FIFO_IMU::Empty()
{
	return (miSize == 0);
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
int64_t FIFO_IMU::Size()
{
	return miSize;
}
/*----------------------------------------------------------------------------------------------*/

ms_packet_imu* FIFO_IMU::Front()
{
	if (miSize == 0)
	{
		std::cout<<"FIFO_IMU:Front(): FIFO_IMU is EMPTY!"<<std::endl;
	}
	return &buff[miFront];
//	return tail;
}

ms_packet_imu* FIFO_IMU::Back()
{
	if (miSize == 0)
	{
		std::cout<<"FIFO_IMU:Back(): FIFO_IMU is EMPTY!"<<std::endl;
	}
	return &buff[miBack];
}
