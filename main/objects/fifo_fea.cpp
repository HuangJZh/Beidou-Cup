

























/*----------------------------------------------------------------------------------------------*/

#include "fifo_fea.h"
#include <iostream>
/*----------------------------------------------------------------------------------------------*/
void *FIFO_Fea_Thread(void *_arg)
{

	FIFO_Fea *aFIFO = pFIFO_Fea;

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
void FIFO_Fea::Start()
{

	Start_Thread(FIFO_Fea_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"FIFO_Fea thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
FIFO_Fea::FIFO_Fea():Threaded_Object("FIFFEATASK")
{

	int64_t lcv;

	/* Create the buffer */
	buff = new ms_packet_fea[FIFO_FEA_DEPTH];
	memset(buff, 0x0, sizeof(ms_packet_fea)*FIFO_FEA_DEPTH);
	head = &buff[0];
	tail = &buff[0];
	miFront = 0;
	miBack = FIFO_FEA_DEPTH - 1;

	/* Create circular linked list */
	for (lcv = 0; lcv < FIFO_FEA_DEPTH-1; lcv++)
		buff[lcv].mpNext = &buff[lcv+1];

	buff[FIFO_FEA_DEPTH-1].mpNext = &buff[0]; //xin this is ONLY for circular buffer : head = tail

	miTic = miCount = miSize = 0;

	int ret_val;
	ret_val = sem_init(&mtSem_full, 0, 0);if (ret_val == -1) std::cout<<"FIFO_Fea::FIFO_Fea(): sem_init(&mtSem_full, 0, 0) failed: "<<strerror(errno)<<std::endl;
	ret_val = sem_init(&mtSem_empty, 0, FIFO_FEA_DEPTH);if (ret_val == -1) exit(0);//std::cout<<"FIFO_Fea::FIFO_Fea(): sem_init(&mtSem_empty, 0, FIFO_FEA_DEPTH) failed: "<<strerror(errno)<<std::endl;

//xin 0505//	pSource_Cam = NULL;//pSource_Cam now taken over by a Read(head) operation reading inputs from pFIFO_Cam
//	fp_out = NULL;
	ResetSource();

//	if(gopt.verbose)
		fprintf(stdout, "Creating FIFO_Fea\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
FIFO_Fea::~FIFO_Fea()
{


	sem_destroy(&mtSem_full);
	sem_destroy(&mtSem_empty);

	delete [] buff;

//xin 0505//	if(pSource_Cam != NULL)
//xin 0505//		delete pSource_Cam;

//	if(gopt.verbose)
		fprintf(stdout,"Destructing FIFO_Fea\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Fea::Import()
{

	IncStartTic();

	/* Read from the GPS source */
/*//xin 0505//	if(pSource_Cam != NULL)
		pSource_Cam->Read(head);//pSource->Read(head);
*/	ssize_t bread;
	bread = read(ACQ_2_FIFO_FEA_P[READ], &(head->mData[0]), sizeof(FeaMeasurement)); if (bread != sizeof(FeaMeasurement)) std::cout<<"FIFO_Fea::Import():read(ACQ_2_FIFO_FEA_P[READ], &head, sizeof(ms_packet_fea)) failed: "<<strerror(errno)<<std::endl;

/*	std::cout<<"FIFO_Fea::Import(): head->mpNext               = "<<head->mpNext<<std::endl;
	std::cout<<"FIFO_Fea::Import(): head->mData[0].mvTimestamp = "<<head->mData[0].mvTimestamp<<std::endl;
//	std::cout<<"FIFO_Fea::Import(): head->mData[0].miTest      = "<<head->mData[0].miTest<<std::endl;
	std::cout<<"FIFO_Fea::Import(): head->mData[0].msImage     = "<<head->mData[0].msImage<<std::endl;
*/

	Enqueue();
//	std::cout<<"-----------------------FIFO_Fea::Import(): head->miCount = "<<head->miCount<<" tail->miCount = "<<tail->miCount<<"------------------"<<std::endl;
	IncStopTic();

	miCount++;
//	fprintf(stdout,"miCount: %ld \n\n\n", miCount);//TODO: only one printer of miCount (either in fifo_imu.cpp or fifo_cam.cpp)!!
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Fea::Enqueue()
{

	sem_wait(&mtSem_empty);
//	int value_mtSem_empty;
//	sem_getvalue(&mtSem_empty, &value_mtSem_empty);
//	std::cout<<"FIFO_Fea::Enqueue(): mtSem_empty = "<<value_mtSem_empty<<std::endl;

	head->miCount = miCount;
//	std::cout<<"FIFO_Fea::Import(): head->miCount              = "<<head->miCount<<std::endl;
//xin 20230709	int data_size = sizeof(ms_packet_fea);

//	std::cout<<"-----------------------FIFO_Fea::Enqueue(): Hey I am here------------------"<<std::endl;
	/* Send a packet to the acquisition (nonblocking) */
//xin 20230709	ssize_t ret_val = write(FIFO_FEA_2_COR_P[WRITE], head, data_size);
/*	std::cout<<"The file descriptor of FIFO_FEA_2_ACQ_P[WRITE]: "<<FIFO_FEA_2_ACQ_P[WRITE]<<std::endl;
	std::cout<<"The file descriptor of FIFO_FEA_2_ACQ_P[READ]:  "<<FIFO_FEA_2_ACQ_P[READ]<<std::endl;
	std::cout<<"The file descriptor of FIFO_IMU_2_ACQ_P[WRITE]: "<<FIFO_IMU_2_ACQ_P[WRITE]<<std::endl;
	std::cout<<"The file descriptor of FIFO_IMU_2_ACQ_P[READ]:  "<<FIFO_IMU_2_ACQ_P[READ]<<std::endl;
	std::cout<<"The file descriptor of STDOUT_FILENO:           "<<STDOUT_FILENO<<std::endl;
	std::cout<<"The file descriptor of STDIN_FILENO:            "<<STDOUT_FILENO<<std::endl;*/
//	std::cout<<"FIFO_Fea::Enqueue(): data_size of ms_pacekt_fea "<<data_size<<std::endl;
//	std::cout<<"FIFO_Fea::Enqueue(): ret_val of the write operation = "<<ret_val<<std::endl;
//xin 20230709	if (ret_val != data_size)
	{
		//std::cout<<"FIFO_Fea::Enqueue(): write(FIFO_FEA_2_COR_P) failed: "<<strerror(errno)<<std::endl;
//xin//0502		exit(-1);
	}

//	std::cout<<"-----------------------FIFO_Fea::Enqueue(): head->miCount = "<<head->miCount<<" tail->miCount = "<<tail->miCount<<"------------------"<<std::endl;
	head = head->mpNext;
	miBack = (miBack + 1) % FIFO_FEA_DEPTH;

	sem_post(&mtSem_full);
	miSize++;//xin 0505
//	fprintf(stdout,"FIFO_Fea::Enqueue()\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Fea::Dequeue(ms_packet_fea *p)
{

	sem_wait(&mtSem_full);

	memcpy(p, tail, sizeof(ms_packet_fea));
	tail = tail->mpNext;
	miFront = (miFront + 1) % FIFO_FEA_DEPTH;
	
	sem_post(&mtSem_empty);
	
	miSize--;//xin 0505
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void FIFO_Fea::ResetSource()
{
/*//xin 0505
	if(pSource_Cam != NULL)
	{
		delete pSource_Cam;
		pSource_Cam = new Source_Cam(); //pSource = new GPS_Source(&gopt);
	}
	else
	{
		pSource_Cam = new Source_Cam(); //pSource = new GPS_Source(&gopt);
	}
*/
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
bool FIFO_Fea::Empty()
{
	return (miSize == 0);
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
int64_t FIFO_Fea::Size()
{
	return miSize;
}
/*----------------------------------------------------------------------------------------------*/

ms_packet_fea* FIFO_Fea::Front()
{
	if (miSize == 0)
	{
		std::cout<<"FIFO_Fea:Front(): FIFO_Fea is EMPTY!"<<std::endl;
	}
	return &buff[miFront];
//	return tail;
}

ms_packet_fea* FIFO_Fea::Back()
{
	if (miSize == 0)
	{
		std::cout<<"FIFO_Fea:Back(): FIFO_Fea is EMPTY!"<<std::endl;
	}
	return &buff[miBack];
}
