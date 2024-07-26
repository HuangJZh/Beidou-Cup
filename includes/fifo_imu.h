


























#ifndef FIFO_IMU_H_
#define FIFO_IMU_H_

#include "includes.h"
#include "source_imu.h"

#define FIFO_IMU_DEPTH (2500)	//!< In ms
//#define FIFO_IMU_DEPTH (6000)	//!< In ms //xin: before 2023-9-29

/*! \ingroup CLASSES
 *
 */
class FIFO_IMU : public Threaded_Object
{

	private:

		sem_t mtSem_full;
		sem_t mtSem_empty;

//		ms_packet_imu *buff;	//!< 1 second buffer (in 1 ms packets)
		ms_packet_imu *head;	//!< Pointer to the head
		ms_packet_imu *tail;	//!< Pointer to the tail

//		int64_t miCount;		//!< Count the number of packets received
		int64_t miTic;		//!< Master receiver tic
//		FILE *fp_out;
		int64_t miSize;			//!< number of elements in FIFO//xin 0505
//		int64_t miBack;
//		int64_t miFront;
	public:

		FIFO_IMU();				//!< Create circular FIFO
		~FIFO_IMU();			//!< Destroy circular FIFO
		void Start();		//!< Start the thread
		void Import();		//!< Get data into the thread
//		void Export();		//!< Get data out of the thread

//		void Open();
		void Enqueue();
		void Dequeue(ms_packet_imu *p);
		void ResetSource();
		bool Empty();		//!< xin 0505
		int64_t Size();		//!< xin 0507
		ms_packet_imu* Back();
		ms_packet_imu* Front();
		int64_t miBack;
		int64_t miFront;
		ms_packet_imu *buff;	//!< 1 second buffer (in 1 ms packets)
		int64_t miCount;		//!< Count the number of packets received
};

#endif /* FIFO_IMU_H */
