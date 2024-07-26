



























#include "source_cam.h"


/*----------------------------------------------------------------------------------------------*/
Source_Cam::Source_Cam() //Source_Cam::Source_Cam(Options_S *_opt)
{/*
	memcpy(&opt, _opt, sizeof(Options_S));

	switch(opt.source)
	{
	case SOURCE_CAM:
		source_type = SOURCE_CAM;
		Open_File_CAM();
		break;
	case SOURCE_IMU:
		source_type = SOURCE_IMU;
		Open_File_IMU();
		break;
	default:
		source_type = SOURCE_CAM;
		Open_File_CAM();
		break;
	}*/
	Open_File_Cam();
//	overflw = soverflw = 0;

    /* Assign to base */
//	mvpBuffOut = &mvImu_measurements[0];//xin: unnecessary, both are 'm'ember variables
	miMs_count = 0;

//	if (opt.verbose)
		fprintf(stdout, "Creating Source_Cam\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
Source_Cam::~Source_Cam()
{

	Close_File_Cam();

//	if(opt.verbose)
		fprintf(stdout, "Destructing Source_Cam\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Source_Cam::Read(ms_packet_cam* _p) //void GPS_Source::Read(ms_packet *_p)
{

	Read_File_Cam(_p);//Read_USRP_V1(_p);

	miMs_count++; //xin: This was previously used as 'ms_count' by Read_Primo(), Read_GN3S(), and
//	Read(ms_packet*) to figure out whether the device is started. also used by Read_GN3S() to determine 
//	which ms of the 5ms long data to copy to the input buffer, i.e. head->data
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Source_Cam::Open_File_Cam()
{
	for (size_t camID = 0; camID < 2; camID++)
	{
		std::string filename_data = gopt.msInput_dataset_path + "/mav0/" + gopt.msCamName + std::to_string(camID) + "/data.csv";
		misFinCam[camID].open(filename_data.c_str());
		std::cout<<"The Cam file is opened: "<<filename_data<<std::endl;
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Source_Cam::Read_File_Cam(ms_packet_cam* _p)
{
	for (size_t camID = 0; camID < 2; camID++)// for stereo camera
	{
		if(!misFinCam[camID].is_open())
		{
			std::cout<< "std::ifstream: Camera input data stream "<<std::to_string(camID)<<" is not open !"<<std::endl;
			exit(-1);
		}

		std::string line;

		// Skip the first line, containing the header.
		if (miMs_count == 0)
		{
			if (!std::getline(misFinCam[camID], line))
			{
				std::cout<< "std::ifstream: Cam input data stream" << gopt.msCamName << std::to_string(camID) << " is EMPTY !"<<std::endl;
				exit(-1);
			}
		}
	//xin 0512//again, first line	else
		if (std::getline(misFinCam[camID], line))
		{
			// Read/store cam measurements, line by line.
			auto idx = line.find_first_of(',');
			Timestamp timestamp = std::stoll(line.substr(0, idx));
			std::string image_filename = gopt.msInput_dataset_path + "/mav0/" + gopt.msCamName + std::to_string(camID) + "/data/" + line.substr(0,idx)+".png";
//			std::cout<<"|||||||||||||||| Source_Cam: image_filename: "<<image_filename<<std::endl;

			cam_buff[0].mvTimestamp[camID] = timestamp;
			size_t a = sizeof(cam_buff[0].msImage[camID]);
			strncpy(cam_buff[0].msImage[camID], image_filename.c_str(), a);
			cam_buff[0].msImage[camID][a-1] = 0;

			{
		//		usleep(10000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct)//xin: before 2023-9-29
		//		usleep(50000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct)//xin: 2023-10-2, trial #2
		//		usleep(10000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct)//xin: 2023-10-2, trial #3
		//		usleep(10000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct)//xin: 2023-10-2, trial #3
				usleep(15000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct)//xin: 2023-10-3
				memcpy(_p->mData, (void *)&cam_buff[0], SAMPS_MS * sizeof(CamMeasurement));
		//		std::cout<<"|||||||||||||||| Source_Cam: p->mData.mvTimestamp[0]: "<<_p->mData[0].mvTimestamp[CAM0]<<std::endl;
		//		std::cout<<"|||||||||||||||| Source_Cam: p->mData.mvTimestamp[1]: "<<_p->mData[0].mvTimestamp[CAM1]<<std::endl;
				//std::cout<<"<><<><><><><> Source_Cam: _p->mData[0].mvTimestamp           = "<<_p->mData[0].mvTimestamp<<" <><<><><><><>"<<std::endl;
			}
		}else{
			fprintf(stdout, "End of File: Quitting Source_Cam\n");
//xin: 2023-10-2		grun = 0;
		}
			//std::cout<<std::endl<<"$$$$$$$$$$$$$$$$$$$ Source_Cam::Read_File_Cam() $$$$$$$$$$$$$$$$$$$$"<<std::endl;
	}
	

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Source_Cam::Close_File_Cam()
{
	for (size_t camID = 0; camID < 2; camID++)
		if (misFinCam[camID].is_open())
			misFinCam[camID].close();
	
//	if(opt.verbose)
		fprintf(stdout,"Close_File_CAM: Destructing File\n");
}
/*----------------------------------------------------------------------------------------------*/
