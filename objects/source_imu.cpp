



























#include "source_imu.h"


/*----------------------------------------------------------------------------------------------*/
Source_IMU::Source_IMU() //Source_IMU::Source_IMU(Options_S *_opt)
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
	Open_File_IMU();
//	overflw = soverflw = 0;

    /* Assign to base */
//	mvpBuffOut = &mvImu_measurements[0];//xin: unnecessary, both are 'm'ember variables
	miMs_count = 0;

//	if (opt.verbose)
		fprintf(stdout, "Creating Source_IMU\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
Source_IMU::~Source_IMU()
{

	Close_File_IMU();

//	if(opt.verbose)
		fprintf(stdout, "Destructing Source_IMU\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Source_IMU::Read(ms_packet_imu* _p) //void GPS_Source::Read(ms_packet *_p)
{

	Read_File_IMU(_p);

	miMs_count++; //xin: This was previously used as 'ms_count' by Read_Primo(), Read_GN3S(), and
//	Read(ms_packet*) to figure out whether the device is started. also used by Read_GN3S() to determine 
//	which ms of the 5ms long data to copy to the input buffer, i.e. head->data
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
void Source_IMU::Open_File_IMU()
{
	///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
    //#timestamp [ns],w_x [rad s^-1],w_y [rad s^-1],w_z [rad s^-1], a_x [m s^-2],a_y [m s^-2],a_z [m s^-2]
	std::string filename_data = gopt.msInput_dataset_path + "/mav0/" + gopt.msImuName + "/data.csv";
	misFinIMU.open(filename_data.c_str());
	std::cout<<"The IMU file is opened: "<<filename_data<<std::endl;
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
void Source_IMU::Read_File_IMU(ms_packet_imu* _p)
{
	if(!misFinIMU.is_open())
	{
		fprintf(stdout, "std::ifstream: IMU input data stream is not open !\n");
		source_success_IMU = -1;
		exit(-1);
	}

/*	if(misFinIMU.is_open())
		fprintf(stdout, "Source_IMU::Read_File_IMU(): input data stream 'misFinIMU' is open !!\n");
*/	
	std::string line;
	
	// Skip the first line, containing the header.
	if (miMs_count == 0)\
	{
		if (!std::getline(misFinIMU, line)) 
		{
			fprintf(stdout, "std::ifstream: IMU input data stream is not EMPTY !\n");
			source_success_IMU = -1;
			exit(-1);
		}
	}
//xin 0512//again, first line	else
	if (std::getline(misFinIMU, line))
	{
		// Read/store imu measurements, line by line.
		Timestamp timestamp = 0;
		gtsam::Vector6 gyr_acc_data;
		for (int i = 0; i < gyr_acc_data.size() + 1; i++)
		{
			int idx = line.find_first_of(',');
			if (i == 0)
				timestamp = std::stoll(line.substr(0, idx));
			else
				gyr_acc_data(i - 1) = std::stod(line.substr(0, idx));
			line = line.substr(idx + 1);
		}
		gtsam::Vector6 imu_accgyr;
		// Acceleration first!
		imu_accgyr << gyr_acc_data.tail(3), gyr_acc_data.head(3);

		//! Store imu measurements
//		mvImu_measurements.push_back(ImuMeasurement(timestamp, imu_accgyr));
		imu_buff[0].mvTimestamp = timestamp;
		imu_buff[0].mvAcc_gyr = imu_accgyr; //xin: copying Eigen matrix maybe questionable

//		usleep(10000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct) //xin: before 2023-9-29
//		usleep(5000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct) //xin: 2023-10-2, trial #2
//		usleep(20000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct) //xin: 2023-10-2, trial #3
//		usleep(10000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct) //xin: 2023-10-2, trial #3
		usleep(10000); // 1ms's clock: to be adjusted. The position (just before memcpy clause is correct) //xin: 2023-10-3
		memcpy(_p->mData, (void *)&imu_buff[0], SAMPS_MS * sizeof(ImuMeasurement));
	}else{
		fprintf(stdout, "End of File: Quitting Source_IMU\n");
//xin: 2023-10-3		grun = 0;
	}
//		std::cout<<std::endl<<"$$$$$$$$$$$$$$$$$$$ Source_IMU::Read_File_IMU() $$$$$$$$$$$$$$$$$$$$"<<std::endl;

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Source_IMU::Close_File_IMU()
{
	if (misFinIMU.is_open())
		misFinIMU.close();
	
//	if(opt.verbose)
		fprintf(stdout,"Close_File_IMU: Destructing File\n");
}
/*----------------------------------------------------------------------------------------------*/
