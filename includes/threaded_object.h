


























#ifndef THREADED_OBJECT_H_
#define THREADED_OBJECT_H_

#include "includes.h"

#ifdef LINUX_OS
#include <pthread.h>
#include <time.h>
#endif

/*! @ingroup CLASSES
	@brief The Threaded_Object class provides the base functionality to monitor each tasks' status
	 and health. This should be OS transparent, and will depend on the #defines LINUX_OS or NUCLEUS_OS
	 to control compilation */
class Threaded_Object
{

	protected:

		/* Default object variables */
//		char				task_name[8];	//!< 8 character task name
		char				task_name[11];	//!< 8 character task name xin: example 'FIFCAMTASK' in fifo_cam.cpp
		uint64_t 			execution_tic;	//!< Execution counter
		uint64_t 			start_tic;		//!< ISR tic at start of function
		uint64_t			temp_start_tic;	//!< ISR tic at start of function (temp)
		uint64_t 			stop_tic;		//!< ISR tic at end of function
		uint64_t			size;			//!< Size of object
		uint64_t			stack;			//!< Number of bytes on stack
		//void 				*task_mem;		//!< Pointer to the task memory
		void				*object_mem;	//!< Pointer to the class memory

		#ifdef NUCLEUS_OS
			NU_TASK 		task;			//!< Nucleus task variable
			NU_SEMAPHORE 	mutex;			//!< Lock and unlock the object
		#endif

		#ifdef LINUX_OS
			pthread_t 		task;			//!< pthread task variable
			pthread_mutex_t	mutex;			//!< Lock and unlock the object
			struct timeval 	tv;				//!< To emulate 500 us tic
		#endif

	public:

		/* Default object methods */
		Threaded_Object(const char _task_name[8]);//!< Constructor
		~Threaded_Object();		//!< Destructor

		void Lock();			//!< Lock the object's mutex
		void Unlock();			//!< Unlock the object's mutex
		uint64_t Trylock();		//!< Trylock, locks the mutex and returns true, else returns false
		void Stop();			//!< Stop the thread

		uint64_t getExecTic();	//!< Get the execution counter
		uint64_t getStartTic();	//!< Get the 500 us ISR tic at start of function
		uint64_t getStopTic();	//!< Get the 500 us ISR tic at end of function
		uint64_t getSize();		//!< Get the size of the object
		//uint64_t getStack();	//!< Get the stack size
		//void *getTaskMem();		//!< Get the task memory
		void *getObjectMem();	//!< Get the class memory

		void IncExecTic();		//!< Increment execution tic
		void IncStartTic();		//!< Get the 500 us ISR tic
		void IncStopTic();		//!< Get the 500 us ISR tic
		//void setStack();		//!< Set the stack depth

		#ifdef LINUX_OS
			void Start_Thread(void *(*_start_routine)(void*), void *_arg);	//!< Start the thread
		#endif

		#ifdef NUCLEUS_OS
			void Start_Thread(void (*_start_routine)(UNSIGNED)(void*));	//!< Start the thread
		#endif

};

#endif /* Threaded_Object_H */
