
























/*----------------------------------------------------------------------------------------------*/

#include "fifo_cam.h"
#include <iostream>
/*----------------------------------------------------------------------------------------------*/
void *FIFO_Cam_Thread(void *_arg)
{

	FIFO_Cam *aFIFO = pFIFO_Cam;

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
void FIFO_Cam::Start()
{

	Start_Thread(FIFO_Cam_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"FIFO_Cam thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
FIFO_Cam::FIFO_Cam():Threaded_Object("FIFCAMTASK")
{

	int64_t lcv;

	/* Create the buffer */
	buff = new ms_packet_cam[FIFO_CAM_DEPTH];
	memset(buff, 0x0, sizeof(ms_packet_cam)*FIFO_CAM_DEPTH);
	head = &buff[0];
	tail = &buff[0];
	miFront = 0;
	miBack = FIFO_CAM_DEPTH - 1;

	/* Create circular linked list */
	for (lcv = 0; lcv < FIFO_CAM_DEPTH-1; lcv++)
		buff[lcv].mpNext = &buff[lcv+1];

	buff[FIFO_CAM_DEPTH-1].mpNext = &buff[0]; //xin this is ONLY for circular buffer : head = tail

	miTic = miCount = miSize = 0;

	int ret_val;
	ret_val = sem_init(&mtSem_full, 0, 0);if (ret_val == -1) std::cout<<"FIFO_Cam::FIFO_Cam(): sem_init(&mtSem_full, 0, 0) failed: "<<strerror(errno)<<std::endl;
	ret_val = sem_init(&mtSem_empty, 0, FIFO_CAM_DEPTH);if (ret_val == -1) std::cout<<"FIFO_Cam::FIFO_Cam(): sem_init(&mtSem_empty, 0, FIFO_CAM_DEPTH) failed: "<<strerror(errno)<<std::endl;

	pSource_Cam = NULL;
//	fp_out = NULL;
	ResetSource();

//	if(gopt.verbose)
		fprintf(stdout, "Creating FIFO_Cam\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
FIFO_Cam::~FIFO_Cam()
{


	sem_destroy(&mtSem_full);
	sem_destroy(&mtSem_empty);

	delete [] buff;

	if(pSource_Cam != NULL)
		delete pSource_Cam;

//	if(gopt.verbose)
		fprintf(stdout,"Destructing FIFO_Cam\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Cam::Import()
{

	IncStartTic();

	/* Read from the GPS source */
	if(pSource_Cam != NULL)
		pSource_Cam->Read(head);//pSource->Read(head);
/*	std::cout<<"FIFO_Cam::Import(): head->mpNext               = "<<head->mpNext<<std::endl;
	std::cout<<"FIFO_Cam::Import(): head->mData[0].mvTimestamp = "<<head->mData[0].mvTimestamp<<std::endl;
//	std::cout<<"FIFO_Cam::Import(): head->mData[0].miTest      = "<<head->mData[0].miTest<<std::endl;
	std::cout<<"FIFO_Cam::Import(): head->mData[0].msImage     = "<<head->mData[0].msImage<<std::endl;
*/

	Enqueue();
//	std::cout<<"-----------------------FIFO_Cam::Import(): head->miCount = "<<head->miCount<<" tail->miCount = "<<tail->miCount<<"------------------"<<std::endl;
	IncStopTic();

	miCount++;
//	fprintf(stdout,"miCount: %ld \n\n\n", miCount);//TODO: only one printer of miCount (either in fifo_imu.cpp or fifo_cam.cpp)!!
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Cam::Enqueue()
{

	sem_wait(&mtSem_empty);
//	int value_mtSem_empty;
//	sem_getvalue(&mtSem_empty, &value_mtSem_empty);
//	std::cout<<"FIFO_Cam::Enqueue(): mtSem_empty = "<<value_mtSem_empty<<std::endl;

	head->miCount = miCount;
//	std::cout<<"FIFO_Cam::Import(): head->miCount              = "<<head->miCount<<std::endl;
	int data_size = sizeof(ms_packet_cam);

//	std::cout<<"-----------------------FIFO_Cam::Enqueue(): Hey I am here------------------"<<std::endl;
	/* Send a packet to the acquisition (nonblocking) */
	ssize_t ret_val = write(FIFO_CAM_2_ACQ_P[WRITE], head, data_size);
/*	std::cout<<"The file descriptor of FIFO_CAM_2_ACQ_P[WRITE]: "<<FIFO_CAM_2_ACQ_P[WRITE]<<std::endl;
	std::cout<<"The file descriptor of FIFO_CAM_2_ACQ_P[READ]:  "<<FIFO_CAM_2_ACQ_P[READ]<<std::endl;
	std::cout<<"The file descriptor of FIFO_IMU_2_ACQ_P[WRITE]: "<<FIFO_IMU_2_ACQ_P[WRITE]<<std::endl;
	std::cout<<"The file descriptor of FIFO_IMU_2_ACQ_P[READ]:  "<<FIFO_IMU_2_ACQ_P[READ]<<std::endl;
	std::cout<<"The file descriptor of STDOUT_FILENO:           "<<STDOUT_FILENO<<std::endl;
	std::cout<<"The file descriptor of STDIN_FILENO:            "<<STDOUT_FILENO<<std::endl;*/
//xin 20230803	std::cout<<"FIFO_Cam::Enqueue(): data_size of ms_packet_cam "<<data_size<<std::endl;
//	std::cout<<"FIFO_Cam::Enqueue(): ret_val of the write operation = "<<ret_val<<std::endl;
	if (ret_val != data_size)
	{
		std::cout<<"FIFO_Cam::Enqueue(): write(FIFO_CAM_2_ACQ_P) failed: "<<strerror(errno)<<std::endl;
//xin//0502		exit(-1);
	}

//	std::cout<<"-----------------------FIFO_Cam::Enqueue(): head->miCount = "<<head->miCount<<" tail->miCount = "<<tail->miCount<<"------------------"<<std::endl;
	head = head->mpNext;
	miBack = (miBack + 1) % FIFO_CAM_DEPTH;

	sem_post(&mtSem_full);
	miSize++;//xin 0505
//	fprintf(stdout,"FIFO_Cam::Enqueue()\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Cam::Dequeue(ms_packet_cam *p)
{

	sem_wait(&mtSem_full);

	memcpy(p, tail, sizeof(ms_packet_cam));
	tail = tail->mpNext;
	miFront = (miFront + 1) % FIFO_CAM_DEPTH;
	
	sem_post(&mtSem_empty);

	miSize--;//xin 0505
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Cam::ResetSource()
{

	if(pSource_Cam != NULL)
	{
		delete pSource_Cam;
		pSource_Cam = new Source_Cam(); //pSource = new GPS_Source(&gopt);
	}
	else
	{
		pSource_Cam = new Source_Cam(); //pSource = new GPS_Source(&gopt);
	}

}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
bool FIFO_Cam::Empty()
{
	return (miSize == 0);
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
int64_t FIFO_Cam::Size()
{
	return miSize;
}
/*----------------------------------------------------------------------------------------------*/

ms_packet_cam* FIFO_Cam::Front()
{
	if (miSize == 0)
	{
		std::cout<<"FIFO_Cam:Front(): FIFO_Cam is EMPTY!"<<std::endl;
	}
	return &buff[miFront];
//	return tail;
}

ms_packet_cam* FIFO_Cam::Back()
{
	if (miSize == 0)
	{
		std::cout<<"FIFO_Cam:Back(): FIFO_Cam is EMPTY!"<<std::endl;
	}
	return &buff[miBack];
}
