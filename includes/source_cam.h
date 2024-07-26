


























#ifndef SOURCE_CAM_H_
#define SOURCE_CAM_H_


#include "includes.h"

/*
// Inertial containers.
using Timestamp = std::int64_t; //common/vio_types.h
using CamStamp = Timestamp;

typedef struct CamMeasurement
{
	CamStamp	mvTimestamp;
	std::string msImage;
} CamMeasurement;


 * Store a list of image names and provide functionalities to parse them.
 */
//using CameraImageLists = std::vector<std::pair<Timestamp, std::string>>;

/*! \ingroup CLASSES
 *
 */
class Source_Cam
{

	private:
		
		/* Options */
//		Options_S opt;			//!< Options

		/* Generic variables */
		int64_t source_type;	//!< Source type



		int64_t miMs_count;		//!< Count the numbers of ms processed





		/* Tag overflows */
//		time_t rawtime;
//		struct tm * timeinfo;






		/* Data buffers */;
		CamMeasurement cam_buff[1]; // for mono//simulated,
		// In fact, buffer ('buff[40932]' , 'gbuff[40919*2]', etc) is for
		// (1) reading hardware input,(2) and the input is a sequence of data.
		// However, here we only send img or ImuAccgyr one by one. But we retain
		// the definition of buffer for future upgrade.



















		/* File handles */
		std::ifstream misFinCam[2];
//		std::ifstream misFinCam; // go for stereo (2023-5-29)

	private:





		void Open_File_Cam();		//!< Open the file

		
		
		
		void Close_File_Cam();		//!< Close the file





		void Read_File_Cam(ms_packet_cam*);		//!< Read from a file



	public:

		Source_Cam();//Sensor_Source(Options_S *_opt);	//!< Create the GPS source with the proper hardware type
		~Source_Cam();					//!< Kill the object
		void Read(ms_packet_cam*);//void Read(ms_packet *_p);		//!< Read in a single ms of data
		
//		int64_t getOvrflw(){return(overflw);}

};

#endif /* SOURCE_CAM_H_ */
