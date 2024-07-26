



























/* These are all found in Main() */
/*----------------------------------------------------------------------------------------------*/
void Parse_Arguments(int, char**); //!< Parse command line arguments to setup functionality
int64_t Hardware_Init(void);              //!< Initialize any hardware (for realtime mode)
int64_t Object_Init(void);                //!< Initialize all threaded objects and global variables
int64_t Pipes_Init(void);               //!< Initialize all pipes
int64_t Thread_Init(void);                //!< Finally start up the threads
void Thread_Shutdown(void);							//!< First step to shutdown, stopping the threads
void Pipes_Shutdown(void);							//!< Close all the pipes
void Object_Shutdown(void);							//!< Delete/free all objects
void Hardware_Shutdown(void);						//!< Shutdown any hardware
/*----------------------------------------------------------------------------------------------*/





/* Found in Misc.cpp */
/*----------------------------------------------------------------------------------------------*/










void FormCCSDSPacketHeader(CCSDS_Packet_Header *_p, uint32 _apid, uint32 _sf, uint32 _pl, uint32 _cm, uint32 _tic);

uint32 adler(uint8 *data, int32 len);
double toSec(int64_t header_stamp);
/*----------------------------------------------------------------------------------------------*/
