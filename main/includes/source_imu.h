


























#ifndef SOURCE_IMU_H_
#define SOURCE_IMU_H_


#include "includes.h"

/*
// Inertial containers.
using Timestamp = std::int64_t; //common/vio_types.h
using ImuStamp = Timestamp;
//using ImuStampS = Eigen::Matrix<std::int64_t, 1, Eigen::Dynamic>;
// order: acceleration data [m/s^2], angular velocities [rad/s].
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;
//using ImuAccGyrS = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using ImuBias = gtsam::imuBias::ConstantBias;

typedef struct ImuMeasurement
{
    ImuStamp 	mvTimestamp;
    ImuAccGyr	mvAcc_gyr;
} ImuMeasurement;

struct ImuMeasurement
{
    ImuMeasurement() = default;
    ImuMeasurement(const ImuStamp& timestamp, const ImuAccGyr& imu_data)
                  : mvTimestamp(timestamp), mvAcc_gyr(imu_data) {}
    ImuMeasurement(ImuStamp&& timestamp, ImuAccGyr&& imu_data)
                 : mvTimestamp(std::move(timestamp)), mvAcc_gyr(std::move(imu_data)) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuStamp mvTimestamp;
    ImuAccGyr mvAcc_gyr;
};
// Multiple Imu measurements, bundled in dynamic matrices.
struct ImuMeasurements
{
	public:
		ImuMeasurements() = default;
		ImuMeasurements(const ImuStampS& timestamps, const ImuAccGyrS& measurements)
			           : mvTimestamps(timestamps), mvAcc_gyr(measurements) {}
        ImuMeasurements(ImuStampS&& timestamps, ImuAccGyrS&& measurements)
                       : mvTimestamps(std::move(timestamps)), mvAcc_gyr(std::move(measurements)) {}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		ImuStampS  mvTimestamps;
        ImuAccGyrS mvAcc_gyr;
};
*/

/*! \ingroup CLASSES
 *
 */
class Source_IMU
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
		ImuMeasurement imu_buff[1];//gyr_acc_data




















		/* File handles */
		std::ifstream misFinIMU;

	private:





		void Open_File_IMU();					//!< Open the file

		
		
		
		void Close_File_IMU();					//!< Close the file




		void Read_File_IMU(ms_packet_imu*);		//!< Read from a file



	public:

		Source_IMU();				//Sensor_Source(Options_S *_opt);	//!< Create the GPS source with the proper hardware type
		~Source_IMU();				//!< Kill the object
		void Read(ms_packet_imu*);	//void Read(ms_packet *_p);		//!< Read in a single ms of data
		
//		int64_t getOvrflw(){return(overflw);}

};

#endif /* SOURCE_IMU_H_ */
