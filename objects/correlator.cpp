









/*----------------------------------------------------------------------------------------------*/

#include "correlator.h"

/*----------------------------------------------------------------------------------------------*/
void *Correlator_Thread(void *_arg)
{

	Correlator *aCorrelator = pCorrelator;

	while(grun)
	{/*//xin: 2023-10-3
		if (pFIFO_IMU->Empty() || pFIFO_Fea->Empty())
		{
			//usleep(1000);
			continue;//return measurements;
		}
        
		if (! (toSec((pFIFO_IMU->Back()->mData[0]).mvTimestamp) >
			   toSec((pFIFO_Fea->Front()->mData[0]).mvTimestamp[CAM0])))//xin(2023-6-1) using left cam (cam0). In EuRoC, the timestamps of left and right cams
																	  //are identical, actually. But this sentence might be a error source when the timestamps
																	  //of both cameras are not preprocessed (i.e. set identical for a tolerable range of difference)
		{
            //ROS_WARN("wait for imu, only should happen at the beginning");
			//usleep(1000);
			continue;//return measurements;
		}*/
		aCorrelator->Import();
		aCorrelator->Correlate();
		aCorrelator->IncExecTic();
	}
	pthread_exit(0);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Correlator::Start()
{
	
	/* With new priority specified */
	Start_Thread(Correlator_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"Correlator thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


template< bool B, class T = void >
using enable_if_t = typename std::enable_if<B,T>::type;
/*********** calibration.h  ***********/

namespace cereal
{
	/****************** thirdparty/eigen_io.h *******************/
	// NOTE: Serialization functions for non-basalt types (for now Eigen and Sophus)
	// are provided in a separate header to make them available to other libraries
	// depending on basalt-headers. Beware that it is not possible to have different
	// and incompatible definitions for these same types in other libraries or
	// executables that also include basalt-headers, as this would lead to undefined
	// behaviour. See
	// https://groups.google.com/d/topic/cerealcpp/WswQi_Sh-bw/discussion for a more
	// detailed discussion and possible workarounds.

	// For binary-archives, don't save a size tag for compact representation.
	template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
	//std::enable_if_t<(_Rows > 0) && (_Cols > 0) &&
	enable_if_t<(_Rows > 0) && (_Cols > 0) && !traits::is_text_archive<Archive>::value>
	serialize(Archive& archive,
			  Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
	{
		for (int i = 0; i < _Rows; i++)
			for (int j = 0; j < _Cols; j++)
				archive(m(i, j));
		// Note: if we can break binary compatibility, we might want to consider the
		// following. However, this would only work for compact Scalar types that can
		// be serialized / deserialized by memcopy, such as double.
		// archive(binary_data(m.data(), _Rows * _Cols * sizeof(_Scalar)));
	}

	// For text-archives, save size-tag even for constant size matrices, to ensure
	// that the serialization is more compact (e.g. for JSON with size tag is uses a
	// simple array, whereas without, it stores a list of pairs like ("value0", v0).
	template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
	//std::enable_if_t<(_Rows > 0) && (_Cols > 0) &&
	enable_if_t<(_Rows > 0) && (_Cols > 0) && traits::is_text_archive<Archive>::value>
	serialize(Archive& archive,
			  Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m)
	{
		size_type s = static_cast<size_type>(_Rows * _Cols);
		archive(make_size_tag(s));
		if (s != _Rows * _Cols)
			throw std::runtime_error("matrix has incorrect length");
		for (size_t i = 0; i < _Rows; i++)
			for (size_t j = 0; j < _Cols; j++)
				archive(m(i, j));
	}

	template <class Archive, class _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols>
	//std::enable_if_t<(_Cols > 0), void> save(
	enable_if_t<(_Cols > 0), void> save(	  Archive& archive,
										const Eigen::Matrix<_Scalar, Eigen::Dynamic, _Cols, _Options, _MaxRows, _MaxCols>& m)
	{
		archive(make_size_tag(static_cast<size_type>(m.size())));
		for (int i = 0; i < m.rows(); i++)
			for (int j = 0; j < _Cols; j++)
				archive(m(i, j));
	}

	template <class Archive, class _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols>
	//std::enable_if_t<(_Cols > 0), void> load(
	enable_if_t<(_Cols > 0), void> load(Archive& archive,
										Eigen::Matrix<_Scalar, Eigen::Dynamic, _Cols, _Options, _MaxRows, _MaxCols>& m)
	{
		size_type size;
		archive(make_size_tag(size));
		m.resize(Eigen::Index(size) / _Cols, _Cols);
		for (int i = 0; i < m.rows(); i++)
			for (int j = 0; j < _Cols; j++)
			  archive(m(i, j));
	}

	template <class Archive, class _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols>
	//std::enable_if_t<(_Rows > 0), void> save(
	enable_if_t<(_Rows > 0), void> save(	  Archive& archive,
										const Eigen::Matrix<_Scalar, _Rows, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& m)
	{
		archive(make_size_tag(static_cast<size_type>(m.size())));
		for (int i = 0; i < _Rows; i++)
			for (int j = 0; j < m.cols(); j++)
				archive(m(i, j));
	}

	template <class Archive, class _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols>
	//std::enable_if_t<(_Rows > 0), void> load(
	enable_if_t<(_Rows > 0), void> load(Archive& archive,
										Eigen::Matrix<_Scalar, _Rows, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& m)
	{
		size_type size;
		archive(make_size_tag(size));
		m.resize(_Rows, Eigen::Index(size) / _Rows);
		for (int i = 0; i < _Rows; i++)
			for (int j = 0; j < m.cols(); j++)
				archive(m(i, j));
	}

	template <class Archive, class _Scalar, int _Options, int _MaxRows, int _MaxCols>
	void save(		Archive& archive,
			  const Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& m)
	{
		archive(make_size_tag(static_cast<size_type>(m.rows())));
		archive(make_size_tag(static_cast<size_type>(m.cols())));
		for (int i = 0; i < m.rows(); i++)
			for (int j = 0; j < m.cols(); j++)
				archive(m(i, j));
	}

	template <class Archive, class _Scalar, int _Options, int _MaxRows, int _MaxCols>
	void load(Archive& archive,
			  Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols>& m)
	{
		size_type rows;
		size_type cols;
		archive(make_size_tag(rows));
		archive(make_size_tag(cols));
		m.resize(rows, cols);
		for (int i = 0; i < m.rows(); i++)
			for (int j = 0; j < m.cols(); j++)
				archive(m(i, j));
	}

	template <class Archive>
	void serialize(Archive& ar, Sophus::SE3d& p) {
		std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>.SE3d: I am here<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	  ar(cereal::make_nvp("px", p.translation()[0]),
		 cereal::make_nvp("py", p.translation()[1]),
		 cereal::make_nvp("pz", p.translation()[2]),
		 cereal::make_nvp("qx", p.so3().data()[0]),
		 cereal::make_nvp("qy", p.so3().data()[1]),
		 cereal::make_nvp("qz", p.so3().data()[2]),
		 cereal::make_nvp("qw", p.so3().data()[3]));
	}

	template <class Archive>
	void serialize(Archive& ar, Sophus::Sim3d& p) {
	  ar(cereal::make_nvp("px", p.translation()[0]),
		 cereal::make_nvp("py", p.translation()[1]),
		 cereal::make_nvp("pz", p.translation()[2]),
		 cereal::make_nvp("qx", p.rxso3().data()[0]),
		 cereal::make_nvp("qy", p.rxso3().data()[1]),
		 cereal::make_nvp("qz", p.rxso3().data()[2]),
		 cereal::make_nvp("qw", p.rxso3().data()[3]));
	}

	/***************  rbasalt/thirdparty/.../headers_serialization.h ********************/
	template <class Archive, class Scalar>
	inline void save(Archive& ar, const DoubleSphereCamera<Scalar>& cam)
	{
		ar(cereal::make_nvp("fx", cam.getParam()[0]),
			cereal::make_nvp("fy", cam.getParam()[1]),
			cereal::make_nvp("cx", cam.getParam()[2]),
			cereal::make_nvp("cy", cam.getParam()[3]),
			cereal::make_nvp("xi", cam.getParam()[4]),
			cereal::make_nvp("alpha", cam.getParam()[5]));
	}


	template <class Archive, class Scalar>
	inline void load(Archive& ar, DoubleSphereCamera<Scalar>& cam)
	{
		Eigen::Matrix<Scalar, 6, 1> intr;
		std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>.DoubleSphereCamera: I am here<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
		ar(cereal::make_nvp("fx", intr[0]),
			cereal::make_nvp("fy", intr[1]),
			cereal::make_nvp("cx", intr[2]),
			cereal::make_nvp("cy", intr[3]),
		 	cereal::make_nvp("xi", intr[4]),
			cereal::make_nvp("alpha", intr[5]));

		cam = DoubleSphereCamera<Scalar>(intr);
	}

	/***************  rbasalt/thirdparty/.../headers_serialization.h ********************/
	template <class Archive, class Scalar>
	inline void serialize(Archive& ar, Calibration<Scalar>& cam)
	{
		std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>.Calibration: I am here<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
		ar(cereal::make_nvp("T_imu_cam", cam.mT_i_c),
			cereal::make_nvp("intrinsics", cam.mIntrinsics),
			cereal::make_nvp("resolution", cam.mResolution),
			cereal::make_nvp("calib_accel_bias", cam.mCalib_accel_bias.getParam()),
			cereal::make_nvp("calib_gyro_bias", cam.mCalib_gyro_bias.getParam()),
			cereal::make_nvp("imu_update_rate", cam.mImu_update_rate),
			cereal::make_nvp("accel_noise_std", cam.mAccel_noise_std),
			cereal::make_nvp("gyro_noise_std", cam.mGyro_noise_std),
			cereal::make_nvp("accel_bias_std", cam.mAccel_bias_std),
			cereal::make_nvp("gyro_bias_std", cam.mGyro_bias_std),
			cereal::make_nvp("cam_time_offset_ns", cam.miCam_time_offset_ns));
//				cereal::make_nvp("vignette", cam.vignette));
	}

}//namespace cereal


/*----------------------------------------------------------------------------------------------*/
void Correlator::load_data(const std::string& calib_path)
{
	std::cout<<"Correlator: load_data()"<<std::endl;
	std::ifstream os(calib_path,
					 std::ios::binary);

	if (os.is_open())
	{
		cereal::JSONInputArchive archive(os);
		std::cout<<"Correlator: load_data(): before archive(mCalib)"<<std::endl;
		archive(mCalib);
		std::cout<<"Correlator: load_data(): after archive(mCalib)"<<std::endl;
		std::cout << "Loaded camera with " << mCalib.mIntrinsics.size() << " cameras" << std::endl;
	} else {
		std::cerr << "could not load camera calibration " << calib_path << std::endl;
		std::abort();
	}
	os.close();//2023-6-24
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
Correlator::Correlator():Threaded_Object("COR333TASK")
{
	
	
	std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;	
	object_mem = this;
	size = sizeof(Correlator);
	packet_count_fea = 0;
	measurement_tic = 0;
	
	Vioconfig config;
//	Calibration<double> calib; //instantiated at line 140 of vio.cpp
	load_data("/home/xin/FAIM/params/euroc_ds_calib.json");


	mbInitialized = false; // keypoint_vio.cpp
//	mT_w_i_init;//Set in TakeMeasurements:if (!mbInitialized)
//	miLast_state_t_ns = ?;//Set in TakeMeasurements:if (!mbInitialized)
//	mFrame_states.?;//Mapped in TakeMeasurements:if (!mbInitialized) 
//	mFrame_poses.?;// TODO TODO TODO uninitialized. set at line 761 in marginalize() of keypoint_vio.cpp 
//	mImu_meas.?;//Mapped in TakeMeasurements:if (!mbInitialized)
//	mPrev_opt_flow_res_test.?;//TODO: keypoint_vio.cpp::measure(): prev_opt_flow_res[opt_flow_meas->t_ns] = opt_flow_meas
//	mLmdb.?;// TODO TODO TODO uninitialized. set at multiple lines in keypoint_vio.cpp: addObservation() vs addLandmark()
	mbTake_kf = true; // keypoint_vio.cpp
	miFrames_after_kf = 0;// keypoint_vio.cpp
	mConfig = config;
//	mKf_ids.?;// TODO TODO TODO uninitialized. set at line 328 in keypoint_vio.cpp: kf_ids.emplace(last_state_t_ns)
//	mNum_points_kf.?;//TODO TODO TODO uninitialized. set at line 413 in keypoint_vio.cpp: num_points_kf[opt_flow_meas->t_ns] =  num_points_added
	mLast_processed_t_ns = 0; // vio_estimator.h
	mbOpt_started = false; // keypoint_vio.cpp
	mdHuber_thresh = config.mdVio_obs_huber_thresh; // keypoint_vio.cpp
	mGyro_bias_weight = mCalib.mGyro_bias_std.array().square().inverse();// keypoint_vio.cpp
	mAccel_bias_weight = mCalib.mAccel_bias_std.array().square().inverse();// keypoint_vio.cpp
	mdLambda = config.mdVio_lm_lambda_min; // keypoint_vio.cpp
	mdMin_lambda = config.mdVio_lm_lambda_min; // keypoint_vio.cpp
	mdMax_lambda = config.mdVio_lm_lambda_max; // keypoint_vio.cpp
	mdLambda_vee = 2; // keypoint_vio.cpp
//	mMarg_order.?; //Mapped in TakeMeasurements:if (!mbInitialized)

	// Setup marginalization, keypoint_vio.coo
	mMarg_H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
	mMarg_b.setZero(POSE_VEL_BIAS_SIZE);
	// prior on position
	mMarg_H.diagonal().head<3>().setConstant(config.mdVio_init_pose_weight);
	// prior on yaw
	mMarg_H(5, 5) = config.mdVio_init_pose_weight;
	// small prior to avoid jumps in bias
	mMarg_H.diagonal().segment<3>(9).array() = config.mdVio_init_ba_weight;
	mMarg_H.diagonal().segment<3>(12).array() = config.mdVio_init_bg_weight;
	std::cout << "marg_H\n" << mMarg_H << std::endl;


	
	mdObs_std_dev = config.mdVio_obs_std_dev;// keypoint_vio.cpp
	mImu_data_queue.set_capacity(300);// vio_estimator.h TODO: 300 ok?
	mOut_marg_queue = nullptr;//vio_estimator.h

	mvBg.setZero();
	mvBa.setZero();
//	mpPrev_frame = nullptr;
	
	mbIs_header_written = false;
	std::string filename_data = "/home/xin/FAIM/output/EuRoC_MAV/machine_hall/MH_03/traj_vio.csv";
	mosTraj_vio_csv.open(filename_data.c_str(), std::ofstream::out | std::ofstream::app);
	mosTraj_vio_csv.precision(20);
	std::cout<<"The output_traj.csv is opened: "<<filename_data<<std::endl;

//	if(gopt.verbose)
		fprintf(stdout,"Creating Correlator\n");

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
Correlator::~Correlator()
{


	if (mOut_marg_queue)
		mOut_marg_queue->push(nullptr);

	if (mosTraj_vio_csv.is_open())
		mosTraj_vio_csv.close();

//	if(gopt.verbose)
		fprintf(stdout,"Destructing Correlator\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Correlator::getMeasurements()//TODO: this function is basically useless as of 2023-6-24
{
#ifdef DEBUG_GETMEASUREMENTS_
	std::cout<<"------ getMeasurements(): pFIFO_Fea->Size() = "<<pFIFO_Fea->Size()<<std::endl;
	std::cout<<"------ getMeasurements(): pFIFO_IMU->Size() = "<<pFIFO_IMU->Size()<<std::endl;
//	std::cout<<"------ getMeasurements(): pFIFO_Cam->Size() = "<<pFIFO_Cam->Size()<<std::endl;
#endif







}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Correlator::Import()
{
	/* Read a packet in */
//xin 20230709	int32 bread = read(FIFO_FEA_2_COR_P[READ], &packet_fea, sizeof(ms_packet_fea)); if (bread != sizeof(ms_packet_fea)) std::cout<<"Correlator::Import():read(FIFO_FEA_2_COR_P[READ], &packet_fea, sizeof(ms_packet_fea)) failed: "<<strerror(errno)<<std::endl;


	/* We have a new packet! */
	packet_count_fea++;//TODO: separate packet_count for imu and fea ??
//xin 20230810	std::cout<<std::endl;std::cout<<"correlator.cpp:import(): packet_count_fea: "<<packet_count_fea<<std::endl;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Correlator::Correlate()
{






	IncStartTic();

	if((packet_count_fea % MEASUREMENT_INT) == 0) //xin 0505//if((packet_count % MEASUREMENT_INT) == 0)
	{
		TakeMeasurements();
	}













































































	IncStopTic();

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Correlator::TakeMeasurements()
{
	ssize_t bwrite;//xin










	measurement_tic++;

//	std::cout<<std::endl<<std::endl;
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl;
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH "<<"correlator.cpp:TakeMeasurements(): measurement_tic: "<<measurement_tic<<" HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl;
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl<<std::endl;

	IntegratedImuMeasurement::Ptr meas;

	const Eigen::Vector3d accel_cov = mCalib.dicrete_time_accel_noise_std().array().square();
	const Eigen::Vector3d gyro_cov = mCalib.dicrete_time_gyro_noise_std().array().square();


	// 1. For each Dequeue operation of pFIFO_IMU: pFIFO_IMU->Dequeue(&packet_imu), rbasalt use equivalent operation of imu_data_queue.pop(data);
	// 		Here I use 'packet_imu.mData[0]' in each occurance of '*data' (data is a shared pointer!)
	// 		Note: DO NOT claim a shared pointer like 'ImuMeasurement::Ptr curr_imu' and then assign it different values using '='
	pFIFO_Fea->Dequeue(&packet_fea);
	pFIFO_Cam->Dequeue(&packet_cam);//Since pFIFO_Cam and pFIFO_Fea are synced, they are always dequeued simultaneously
	static int64_t prev_frame_t_ns = -1;
	static FeaMeasurement::Ptr curr_frame = (FeaMeasurement::Ptr)&packet_fea.mData[0];
	

	if (!mbInitialized)
	{
		// 2. when data is not yet dequeued into 'packet_imu', use 'pFIFO_IMU->Front()->mData[0]' INSTEAD OF 'packet_imu->mData[0]' to extract mvTimestamp!!!
		pFIFO_IMU->Dequeue(&packet_imu);
		while (packet_imu.mData[0].mvTimestamp < curr_frame->mvTimestamp[CAM0])
		{
			std::cout<<"Correlator::TakeMeasurements(): if (!mbInitialized): while loop: packet_imu.mData[0].mvTimestamp = "<<packet_imu.mData[0].mvTimestamp
																					  <<", curr_frame->mvTimestamp[CAM0] = "<<curr_frame->mvTimestamp[CAM0]<<std::endl;
			pFIFO_IMU->Dequeue(&packet_imu);

			packet_imu.mData[0].mvAcc_gyr.head<3>() = mCalib.mCalib_accel_bias.getCalibrated(packet_imu.mData[0].mvAcc_gyr.head<3>());
			packet_imu.mData[0].mvAcc_gyr.tail<3>() = mCalib.mCalib_gyro_bias.getCalibrated(packet_imu.mData[0].mvAcc_gyr.tail<3>());
			std::cout << "Skipping IMU data.." <<std::endl;
		}

		Eigen::Vector3d vel_w_i_init;
		vel_w_i_init.setZero();

		mT_w_i_init.setQuaternion(Eigen::Quaterniond::FromTwoVectors(packet_imu.mData[0].mvAcc_gyr.head<3>(),
																	 Eigen::Vector3d::UnitZ()));

		miLast_state_t_ns = curr_frame->mvTimestamp[CAM0];
		mImu_meas[miLast_state_t_ns] = IntegratedImuMeasurement(miLast_state_t_ns,
																mvBg,
																mvBa);
		mFrame_states[miLast_state_t_ns] = PoseVelBiasStateWithLin(miLast_state_t_ns,
																   mT_w_i_init,
																   vel_w_i_init,
																   mvBg,
																   mvBa,
																   true);

		mMarg_order.mAbs_order_map[miLast_state_t_ns] = std::make_pair(0, POSE_VEL_BIAS_SIZE);
		mMarg_order.miTotal_size = POSE_VEL_BIAS_SIZE;
		mMarg_order.miItems = 1;

		std::cout << "Setting up filter: t_ns " << miLast_state_t_ns << std::endl;
		std::cout << "T_w_i\n" << mT_w_i_init.matrix() << std::endl;
		std::cout << "vel_w_i " << vel_w_i_init.transpose() << std::endl;

		mbInitialized = true;
	}//if (!mbInitialized)

	if (prev_frame_t_ns > 0)
	{
		std::cout<<"Correlator::TakeMeasurements(): if (mpPrev_frame)"<<std::endl;
		std::cout<<"Correlator::TakeMeasurements(): if (mpPrev_frame): BEFORE 1st while loop:  data->t_ns = "<<(packet_imu.mData[0]).mvTimestamp
																					<<", curr_frame->t_ns = "<<curr_frame->mvTimestamp[CAM0]
																					<<", prev_frame->t_ns = "<<prev_frame_t_ns<<std::endl;
		// preintegrate measurements

		auto last_state = mFrame_states.at(miLast_state_t_ns);

		meas.reset(new IntegratedImuMeasurement(prev_frame_t_ns,
												last_state.getState().mBias_gyro,
												last_state.getState().mBias_accel));

		while (packet_imu.mData[0].mvTimestamp <= prev_frame_t_ns)
		{
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): BEFORE meas->integrate(): 0"<<std::endl;
			pFIFO_IMU->Dequeue(&packet_imu);
			
			packet_imu.mData[0].mvAcc_gyr.head<3>() = mCalib.mCalib_accel_bias.getCalibrated(packet_imu.mData[0].mvAcc_gyr.head<3>());
			packet_imu.mData[0].mvAcc_gyr.tail<3>() = mCalib.mCalib_gyro_bias.getCalibrated(packet_imu.mData[0].mvAcc_gyr.tail<3>());
		}

		std::cout<<"Correlator::TakeMeasurements(): if (mpPrev_frame): BEFORE 2nd while loop:  data->t_ns = "<<(packet_imu.mData[0]).mvTimestamp
																					<<", curr_frame->t_ns = "<<curr_frame->mvTimestamp[CAM0]<<std::endl;
		while (packet_imu.mData[0].mvTimestamp <= curr_frame->mvTimestamp[CAM0])
		{
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): BEFORE meas->integrate(): 1"<<std::endl;
			meas->integrate(packet_imu.mData[0],
							accel_cov,
							gyro_cov);// TODO: should we update accel_cov & gyro_cov, which are members of mCalib?

			pFIFO_IMU->Dequeue(&packet_imu);
			
			packet_imu.mData[0].mvAcc_gyr.head<3>() = mCalib.mCalib_accel_bias.getCalibrated(packet_imu.mData[0].mvAcc_gyr.head<3>());
			packet_imu.mData[0].mvAcc_gyr.tail<3>() = mCalib.mCalib_gyro_bias.getCalibrated(packet_imu.mData[0].mvAcc_gyr.tail<3>());
		}

		if (meas->get_start_t_ns() + meas->get_dt_ns() < curr_frame->mvTimestamp[CAM0])
		{
			int64_t tmp = packet_imu.mData[0].mvTimestamp;
			packet_imu.mData[0].mvTimestamp = curr_frame->mvTimestamp[CAM0];
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): BEFORE meas->integrate(): 2"<<std::endl;
			meas->integrate(packet_imu.mData[0],
							accel_cov,
							gyro_cov);
			packet_imu.mData[0].mvTimestamp = tmp;
		}
	}//if (mpPrev_frame)
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator:: before measure() HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl;
	measure(curr_frame,
			meas);
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator:: after measure() HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl;
	prev_frame_t_ns = curr_frame->mvTimestamp[CAM0];





	/* Write the preamble, then the measurements */
	if((measurement_tic % MEASUREMENT_MOD) == 0)
	{
		preamble.tic_measurement = measurement_tic;
		bwrite = write(ISRP_2_PVT_P[WRITE], &preamble, sizeof(Preamble_2_PVT_S)); if (bwrite != sizeof(Preamble_2_PVT_S)) std::cout<<"Correlator::TakeMeasurements():write(ISRP_2_PVT_P[WRITE], &preamble, sizeof(Preamble_2_PVT_S)) failed: "<<strerror(errno)<<std::endl;
	}

}

/*----------------------------------------------------------------------------------------------*/
bool Correlator::measure(const FeaMeasurement::Ptr&           opt_flow_meas,
                         const IntegratedImuMeasurement::Ptr& meas)
{
    if (meas.get())
	{
		FAIM_ASSERT(mFrame_states[miLast_state_t_ns].getState().miT_ns ==  meas->get_start_t_ns());
		FAIM_ASSERT(opt_flow_meas->mvTimestamp[CAM0] == meas->get_dt_ns() + meas->get_start_t_ns());

		PoseVelBiasState next_state = mFrame_states.at(miLast_state_t_ns).getState();

		meas->predictState(mFrame_states.at(miLast_state_t_ns).getState(),	//in
				           g,												//in	
						   next_state);										//in & out
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): AFTER predictState()"<<std::endl;

		miLast_state_t_ns = opt_flow_meas->mvTimestamp[CAM0];
		next_state.miT_ns = opt_flow_meas->mvTimestamp[CAM0];

		mFrame_states[miLast_state_t_ns] = next_state;

		mImu_meas[meas->get_start_t_ns()] = *meas;
    }
	OpticalFlowResultReduced::Ptr prev_observations;
	prev_observations.reset(new OpticalFlowResultReduced);
	prev_observations->observations.resize(2);
	for (size_t camid = 0; camid < 2; camid++)
	{
		for (size_t i = 0; i < opt_flow_meas->muCnt_un_pts[camid]; i++)
		{
			cv::Point2f pt(opt_flow_meas->mfCur_un_pts_x[camid][i],
						   opt_flow_meas->mfCur_un_pts_y[camid][i]);
			prev_observations->observations.at(camid).insert(std::make_pair(opt_flow_meas->mvIds[camid][i],
															  pt));
		}
	}
    // save results
	mPrev_opt_flow_res_test[opt_flow_meas->mvTimestamp[CAM0]] = prev_observations;
/*
	std::cout<<"CCCCCCCCCCCCCCCCCCCCCCCCCCC "<<"mPrev_opt_flow_res_test"<<" START with map size = "<<mPrev_opt_flow_res_test.size()<<"  CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"<<std::endl;
	for (auto oo : mPrev_opt_flow_res_test)
	{
		std::cout<<"CCCCCCCCCCCCCCCCCCCCCCCCCCC "<<oo.first<<"  CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"<<std::endl;
		for (size_t camid = 0; camid < 2; camid++)//xin 20230816
		{
			std::cout<<"CCCCCCC "<< "CAM"<<camid<<":"<<std::endl
			<<"CCCCCCC "<<"muCnt_un_pts = "<<oo.second->observations.at(camid).size()<<std::endl;
			size_t kv_i = 0;
			for (auto pt : oo.second->observations.at(camid))
			{
				std::cout<<"CCCCCCC "<<kv_i<<" : (x, y) = ("<<pt.second.x<<", "<<pt.second.y<<")    kpt_id = "<<pt.first<<std::endl;
				kv_i++;
			}
			std::cout<<std::endl;
		}
	}
	std::cout<<"CCCCCCCCCCCCCCCCCCCCCCCCCCC "<<"mPrev_opt_flow_res_test"<<" END"<<"  CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"<<std::endl;
*/
    // Make new residual for existing keypoints
    int connected0 = 0;
    std::map<int64_t, int> num_points_connected;
    std::unordered_set<int> unconnected_obs0;
    for (size_t camid = 0; camid < 2; camid++)//two cameras
	{
		TimeCamId tcid_target(opt_flow_meas->mvTimestamp[CAM0], camid);
//xin 20230803		std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::measure(): ----------camid = "<<camid<<std::endl;
		for (size_t kv_obs_i = 0; kv_obs_i < opt_flow_meas->muCnt_un_pts[camid]; kv_obs_i++)
		{
		    size_t kpt_id = opt_flow_meas->mvIds[camid][kv_obs_i];

		    if (mLmdb.landmarkExists(kpt_id))
			{
				KeypointObservation kobs;
				kobs.miKpt_id = kpt_id;
				Eigen::Vector2d pp;
				pp << opt_flow_meas->mfCur_un_pts_x[camid][kv_obs_i],
				      opt_flow_meas->mfCur_un_pts_y[camid][kv_obs_i];
				kobs.mPos = pp;
				mLmdb.addObservation(tcid_target, kobs);

				const TimeCamId& tcid_host = mLmdb.getLandmark(kpt_id).mKf_id;
				if (num_points_connected.count(tcid_host.miFrame_id) == 0)
				    num_points_connected[tcid_host.miFrame_id] = 0;
				num_points_connected[tcid_host.miFrame_id]++;

				if (camid == CAM0) connected0++;
		    }
			else
			{
				if (camid == CAM0)
				  unconnected_obs0.emplace(kpt_id);
		    }
		}
    }

	// 2 cases to assign a frame keyframe
    if (double(connected0) / (connected0 + unconnected_obs0.size()) < mConfig.mfVio_new_kf_keypoints_thresh && //(1) only less than 70% of the observations are recognized as 'seen landmarks': new info should be embedded in this new frame, so make it keyframe!! 
        miFrames_after_kf > mConfig.miVio_min_frames_after_kf) //(2) after the latest keyframe, we have at least 5 new frames
        mbTake_kf = true;
/* //xin 20230818
//xin 20230816
	std::cout<<"unconnected_obs0.size() : "<<unconnected_obs0.size()<<std::endl;
	for (auto idx : unconnected_obs0)
		std::cout<<idx<<", "<<std::endl;
	std::cout<<"connected0: "<<connected0<<std::endl;
*/
//	if (mConfig.vio_debug)
 //xin 20230803       std::cout << "connected0 " << connected0 
	//xin 20230803  	          << " unconnected_obs0.size() " << unconnected_obs0.size() 
	//xin 20230803			  << " mConfig.mfVio_new_kf_keypoints_thresh "<< mConfig.mfVio_new_kf_keypoints_thresh
	//xin 20230803			  << " miFrames_after_kf = " << miFrames_after_kf 
	//xin 20230803			  << " mConfig.miVio_min_frames_after_kf " << mConfig.miVio_min_frames_after_kf << std::endl;

	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::measure() : mbTake_kf = "<<mbTake_kf<<std::endl;
    if (mbTake_kf)
    {
        // Triangulate new points from stereo and make keyframe for camera 0
        mbTake_kf = false;
        miFrames_after_kf = 0;
        mKf_ids.emplace(miLast_state_t_ns);

        TimeCamId tcidl(opt_flow_meas->mvTimestamp[CAM0], CAM0); // TODO: deduced from this sentence, FeaMeasurement.mvTimestamp should be only for CAM0, not both CAM0 and CAM1

        int num_points_added = 0;
//xin 20230818		size_t lm_id_ii = 0;
        for (int lm_id : unconnected_obs0)
	    {
            // Find all observations
            std::map<TimeCamId, KeypointObservation> kp_obs; //xin: <PREVIOUS_tcid, CURRENT_lmid_&_cvPoint2f>

            for (const auto& kv : mPrev_opt_flow_res_test)
		    {
//xin 20230818				std::cout<<"kv.first = "<<kv.first<<std::endl;
			    for (size_t camid = 0; camid < kv.second->observations.size(); camid++)
			    {
					auto it = kv.second->observations[camid].find(lm_id);
					if (it != kv.second->observations[camid].end())
				    {
						KeypointObservation kobs;
					    kobs.miKpt_id = lm_id;
						Eigen::Vector2d qq;
						qq << it->second.x,
							  it->second.y;
					    kobs.mPos = qq;

				  	    TimeCamId tcido(kv.first, camid);
					    kp_obs[tcido] = kobs;//xin: kobs:CURRENT_lmid_&_cvPoint2f; tcido:PREVIOUS_tcido
				    }
			    }//for (size_t camid = 0; camid < 2; camid++)
            }//for (const auto& kv : mPrev_opt_flow_res_test)

            // triangulate
            bool valid_kp = false;
            const double min_triang_distance2 = mConfig.mdVio_min_triangulation_dist * mConfig.mdVio_min_triangulation_dist;
//xin 20230818			size_t kv_obs_ii = 0;
		    for (const auto& kv_obs : kp_obs)
		    {
			    if (valid_kp) break;
			    TimeCamId tcido = kv_obs.first;
//				const Eigen::Vector2d p0 = opt_flow_meas->observations.at(0)
//												.at(lm_id)
//												.translation()
//												.cast<double>();
//				const Eigen::Vector2d p1 = prev_opt_flow_res[tcido.frame_id]->observations[tcido.cam_id]
//												.at(lm_id)
//												.translation()
//												.cast<double>();
				cv::Point2f p0_cv = prev_observations->observations.at(CAM0)
																.at(lm_id);
				cv::Point2f p1_cv = mPrev_opt_flow_res_test[tcido.miFrame_id]->observations[tcido.miCam_id]
																.at(lm_id);
				Eigen::Vector2d p0, p1;
				p0 << (double) p0_cv.x, (double) p0_cv.y;
				p1 << (double) p1_cv.x, (double) p1_cv.y;

/* //xin 20230818
				std::cout<<"p0: ("<< p0[0] <<", "<<p0[1]<<")"<<std::endl;
				std::cout<<"p1: ("<< p1[0] <<", "<<p1[1]<<")"<<std::endl;
*/
				Eigen::Vector4d p0_3d, p1_3d;
			    bool valid1 = mCalib.mIntrinsics[0].unproject(p0,//DoubleSphereCamera
				    	                                  	  p0_3d);
			    bool valid2 = mCalib.mIntrinsics[tcido.miCam_id].unproject(p1,
					                                                       p1_3d);
			    if (!valid1 || !valid2) continue;

			    Sophus::SE3d T_i0_i1 = getPoseStateWithLin(tcidl.miFrame_id).getPose().inverse() *
				                       getPoseStateWithLin(tcido.miFrame_id).getPose();
			    Sophus::SE3d T_0_1 = mCalib.mT_i_c[0].inverse() * T_i0_i1 * mCalib.mT_i_c[tcido.miCam_id];
  
  			    if (T_0_1.translation().squaredNorm() < min_triang_distance2) continue;

			    Eigen::Vector4d p0_triangulated = triangulate(p0_3d.head<3>(),
						                                      p1_3d.head<3>(),
															  T_0_1);

			    if (p0_triangulated.array().isFinite().all() &&
                    p0_triangulated[3] > 0 && p0_triangulated[3] < 3.0)
			    {
				    KeypointPosition kpt_pos;
				    kpt_pos.mKf_id = tcidl;
				    kpt_pos.mDir = StereographicParam<double>::project(p0_triangulated);
				    kpt_pos.mdId = p0_triangulated[3];
					kpt_pos.mdBackup_id = -1;// xin: 2023-6-16. this is added only for suppressing 'possible uninitialized'
				    mLmdb.addLandmark(lm_id, kpt_pos);

				    num_points_added++;
				    valid_kp = true;
			    }
				
/* //xin 20230818
				std::cout<<"kv_obs_ii = "<<kv_obs_ii<<std::endl;
				kv_obs_ii++;
*/
		    }//for (const auto& kv_obs : kp_obs)
/* //xin 20230818
			std::cout<<"for (const auto& kv_obs : kp_obs) ENDs!!!!!!: lm_id_ii = "<<lm_id_ii<<", lm_id = "<<lm_id<<std::endl<<std::endl;//xin 20230812: remove all instances of 'lm_id_ii' and 'kv_obs_ii' once debugging is complete
			lm_id_ii++;
*/
		    if (valid_kp)
		    {
			    for (const auto& kv_obs : kp_obs)
			        mLmdb.addObservation(kv_obs.first, kv_obs.second); // <target_tcid, host_tcid>: target=PREVIOUS / host=CURRENT
		    }
        }//for (int lm_id : unconnected_obs0)

        mNum_points_kf[opt_flow_meas->mvTimestamp[CAM0]] = num_points_added;
    }
    else//if (mbTake_kf)
        miFrames_after_kf++;

	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator:: measure() 3: optimize() HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl;
    optimize();
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator:: measure() 3: marginalize() HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"<<std::endl;
    marginalize(num_points_connected);
/*
    if (mOut_state_queue)
	{
		PoseVelBiasStateWithLin p = mFrame_states.at(miLast_state_t_ns);

		PoseVelBiasState::Ptr data(new PoseVelBiasState(p.getState()));

		mOut_state_queue->push(data);
    }

    if (mOut_vis_queue)
	{
		VioVisualizationData::Ptr data(new VioVisualizationData);

		data->t_ns = miLast_state_t_ns;

		for (const auto& kv : mFrame_states)
		    data->states.emplace_back(kv.second.getState().mT_w_i);

		for (const auto& kv : mFrame_poses)
		    data->frames.emplace_back(kv.second.getPose());

		get_current_points(data->points,
				           data->point_ids);

		data->projections.resize(opt_flow_meas->observations.size());
		computeProjections(data->projections);

		data->opt_flow_res = mPrev_opt_flow_res_test[miLast_state_t_ns];

		mOut_vis_queue->push(data);
    }
*/
    mLast_processed_t_ns = miLast_state_t_ns;

	if (!mbIs_header_written)
	{
		mosTraj_vio_csv << "timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz"<<std::endl;
		mbIs_header_written = true;
	}
	PoseVelBiasStateWithLin p = mFrame_states.at(miLast_state_t_ns);
	PoseVelBiasState::Ptr data(new PoseVelBiasState(p.getState()));
	mosTraj_vio_csv	<< data->miT_ns << ","
					<< data->mT_w_i.translation()[0] << ","
					<< data->mT_w_i.translation()[1] << ","
					<< data->mT_w_i.translation()[2] << ","
					<< data->mT_w_i.so3().data()[3] << ","			//qw
					<< data->mT_w_i.so3().data()[0] << ","			//qx
					<< data->mT_w_i.so3().data()[1] << ","			//qy
					<< data->mT_w_i.so3().data()[2] << ","			//qz
					<< data->mVel_w_i(0) << ","
					<< data->mVel_w_i(1) << ","
					<< data->mVel_w_i(2) << ","
					<< data->mBias_gyro(0) << ","
					<< data->mBias_gyro(1) << ","
					<< data->mBias_gyro(2) << ","
					<< data->mBias_accel(0) << ","
					<< data->mBias_accel(1) << ","
					<< data->mBias_accel(2)	<< std::endl;

    return true;
}

/****************** ba_base.cpp ***********************/
Sophus::SE3d Correlator::computeRelPose(const Sophus::SE3d&     T_w_i_h,
                                        const Sophus::SE3d&     T_i_c_h,
                                        const Sophus::SE3d&     T_w_i_t,
                                        const Sophus::SE3d&     T_i_c_t,
                                              Sophus::Matrix6d* d_rel_d_h,
                                              Sophus::Matrix6d* d_rel_d_t)
{
    Sophus::SE3d tmp2 = (T_i_c_t).inverse();

    Sophus::SE3d T_t_i_h_i;
    T_t_i_h_i.so3() = T_w_i_t.so3().inverse() * T_w_i_h.so3();
    T_t_i_h_i.translation() = T_w_i_t.so3().inverse() * (T_w_i_h.translation() - T_w_i_t.translation());

    Sophus::SE3d tmp = tmp2 * T_t_i_h_i;
    Sophus::SE3d res = tmp * T_i_c_h;

    if (d_rel_d_h)
	{
		Eigen::Matrix3d R = T_w_i_h.so3().inverse().matrix();

		Sophus::Matrix6d RR;
		RR.setZero();
		RR.topLeftCorner<3, 3>() = R;
		RR.bottomRightCorner<3, 3>() = R;

		*d_rel_d_h = tmp.Adj() * RR;
    }

    if (d_rel_d_t)
	{
		Eigen::Matrix3d R = T_w_i_t.so3().inverse().matrix();

		Sophus::Matrix6d RR;
		RR.setZero();
		RR.topLeftCorner<3, 3>() = R;
		RR.bottomRightCorner<3, 3>() = R;

		*d_rel_d_t = -tmp2.Adj() * RR;
    }

    return res;
}

/****************** ba_base.cpp ***********************/
void Correlator::linearizeHelper(      Eigen::aligned_vector<RelLinData>&                                                  rld_vec,     //out
                                 const Eigen::aligned_map<TimeCamId,
										                  Eigen::aligned_map<TimeCamId,
																	         Eigen::aligned_vector<KeypointObservation>>>& obs_to_lin,  //in
                                       double&                                                                             error) const //out
{
    error = 0;

    rld_vec.clear();

    std::vector<TimeCamId> obs_tcid_vec;
    for (const auto& kv : obs_to_lin)
	{
		obs_tcid_vec.emplace_back(kv.first);
		rld_vec.emplace_back(mLmdb.numLandmarks(),
				             kv.second.size());//size of Eigen::aligned_vector<KeypointObservation>
    }
    
	tbb::parallel_for
    (
        tbb::blocked_range<size_t>(0, obs_tcid_vec.size()),
        [&](const tbb::blocked_range<size_t>& range)
		{
            for (size_t r = range.begin(); r != range.end(); ++r)
			{
			    auto kv = obs_to_lin.find(obs_tcid_vec[r]);

			    RelLinData& rld = rld_vec[r];

			    rld.error = 0;

			    const TimeCamId& tcid_h = kv->first;

			    for (const auto& obs_kv : kv->second)
				{
					const TimeCamId& tcid_t = obs_kv.first;
					if (tcid_h != tcid_t)
                    {
					    // target and host are not the same
					    rld.order.emplace_back(std::make_pair(tcid_h, tcid_t));

					    PoseStateWithLin state_h = getPoseStateWithLin(tcid_h.miFrame_id);
					    PoseStateWithLin state_t = getPoseStateWithLin(tcid_t.miFrame_id);

					    Sophus::Matrix6d d_rel_d_h, d_rel_d_t;

					    Sophus::SE3d T_t_h_sophus = computeRelPose(state_h.getPoseLin(),			//IN
								                                   mCalib.mT_i_c[tcid_h.miCam_id],	//IN
																   state_t.getPoseLin(),			//IN
																   mCalib.mT_i_c[tcid_t.miCam_id],	//IN
																   &d_rel_d_h,						//OUT
						                                           &d_rel_d_t);						//OUT

//xin 20230803					std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN linearizeHelper(): AFTER computeRelPose()"<<std::endl;
					    rld.d_rel_d_h.emplace_back(d_rel_d_h);
					    rld.d_rel_d_t.emplace_back(d_rel_d_t);

					    if (state_h.isLinearized() || state_t.isLinearized())
						    T_t_h_sophus = computeRelPose(state_h.getPose(),				//IN
									                      mCalib.mT_i_c[tcid_h.miCam_id],	//IN
														  state_t.getPose(),				//IN
														  mCalib.mT_i_c[tcid_t.miCam_id]);	//IN

					    Eigen::Matrix4d T_t_h = T_t_h_sophus.matrix();

					    FrameRelLinData frld;

//					    std::visit([&](const auto& cam)
//									{
										for (size_t i = 0; i < obs_kv.second.size(); i++)
										{
											const KeypointObservation& kpt_obs = obs_kv.second[i];
											const KeypointPosition& kpt_pos = mLmdb.getLandmark(kpt_obs.miKpt_id);

											Eigen::Vector2d res;
											Eigen::Matrix<double, 2, POSE_SIZE> d_res_d_xi;
											Eigen::Matrix<double, 2, 3> d_res_d_p;

											bool valid = linearizePoint(kpt_obs,								//IN
																		kpt_pos,								//IN
																		T_t_h,									//IN
																		mCalib.mIntrinsics[tcid_t.miCam_id],//cam,IN
																		res,									//OUT
																		&d_res_d_xi,							//OUT
																		&d_res_d_p);							//OUT

											if (valid)
											{
												double e = res.norm();
												double huber_weight = e < mdHuber_thresh ? 1.0 : mdHuber_thresh / e;
												double obs_weight = huber_weight / (mdObs_std_dev * mdObs_std_dev);

												rld.error               += (2 - huber_weight) * obs_weight * res.transpose() * res;
//xin 20230803											std::cout<< "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH							= delta(rld.error) @[if tcid_h != tcid_t]:        " << (2 - huber_weight) * obs_weight * res.transpose() * res << std::endl<<std::endl;

												if (rld.Hll.count(kpt_obs.miKpt_id) == 0)
												{
													rld.Hll[kpt_obs.miKpt_id].setZero();
													rld.bl[kpt_obs.miKpt_id].setZero();
												}

												rld.Hll[kpt_obs.miKpt_id] += obs_weight * d_res_d_p.transpose() * d_res_d_p;
												rld.bl[kpt_obs.miKpt_id]  += obs_weight * d_res_d_p.transpose() * res;

												frld.Hpp                += obs_weight * d_res_d_xi.transpose() * d_res_d_xi;
												frld.bp                 += obs_weight * d_res_d_xi.transpose() * res;
												frld.Hpl.emplace_back(obs_weight * d_res_d_xi.transpose() * d_res_d_p);
												frld.lm_id.emplace_back(kpt_obs.miKpt_id);

												rld.lm_to_obs[kpt_obs.miKpt_id].emplace_back(rld.Hpppl.size(), frld.lm_id.size() - 1);
											}
										}
//									}, mCalib.mIntrinsics[tcid_t.miCam_id].variant);

					    rld.Hpppl.emplace_back(frld);

                    }//if (tcid_h != tcid_t)
				    else
					{
					    // target and host are the same
					    // residual does not depend on the pose
					    // it just depends on the point

//					    std::visit([&](const auto& cam)
//									{
										for (size_t i = 0; i < obs_kv.second.size(); i++)
										{
											const KeypointObservation& kpt_obs = obs_kv.second[i];
											const KeypointPosition& kpt_pos = mLmdb.getLandmark(kpt_obs.miKpt_id);

											Eigen::Vector2d res;
											Eigen::Matrix<double, 2, 3> d_res_d_p;

											bool valid = linearizePoint(kpt_obs,
																		kpt_pos,
																		mCalib.mIntrinsics[tcid_t.miCam_id],//cam,
																		res,
																		&d_res_d_p);

											if (valid)
											{
												double e = res.norm();
												double huber_weight = e < mdHuber_thresh ? 1.0 : mdHuber_thresh / e;
												double obs_weight = huber_weight / (mdObs_std_dev * mdObs_std_dev);

												rld.error += (2 - huber_weight) * obs_weight * res.transpose() * res;
//xin 20230803											std::cout<< "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH							= delta(rld.error) @[if tcid_h == tcid_t]:        " << (2 - huber_weight) * obs_weight * res.transpose() * res << std::endl<<std::endl;

												if (rld.Hll.count(kpt_obs.miKpt_id) == 0)
												{
													rld.Hll[kpt_obs.miKpt_id].setZero();
													rld.bl[kpt_obs.miKpt_id].setZero();
												}

												rld.Hll[kpt_obs.miKpt_id] += obs_weight * d_res_d_p.transpose() * d_res_d_p;
												rld.bl[kpt_obs.miKpt_id] += obs_weight * d_res_d_p.transpose() * res;
											}
										}
//									}, mCalib.mIntrinsics[tcid_t.miCam_id].variant);
                    }//if (tcid_h != tcid_t) else
			    }//for (const auto& obs_kv : kv->second)
            }//for (size_t r = range.begin(); r != range.end(); ++r)
        }//[&](const tbb::blocked_range<size_t>& range)
    ); //tbb::parallel_for

    for (const auto& rld : rld_vec)
		error += rld.error;
}//void linearizeHelper()

/******************** keypoint_vio_linearize.cpp ********************/
void Correlator::linearizeAbsIMU(const AbsOrderMap&                                           aom,               //IN
		                               Eigen::MatrixXd&                                       abs_H,             //OUT
									   Eigen::VectorXd&                                       abs_b,             //OUT
                                       double&                                                imu_error,         //OUT
									   double&                                                bg_error,          //OUT
									   double&                                                ba_error,          //OUT
                                 const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>&  states,            //IN
                                 const Eigen::aligned_map<int64_t, IntegratedImuMeasurement>& imu_meas,          //IN
                                 const Eigen::Vector3d&                                       gyro_bias_weight,  //IN
                                 const Eigen::Vector3d&                                       accel_bias_weight, //IN
								 const Eigen::Vector3d&                                       g)                 //IN
{
    imu_error = 0;
    bg_error = 0;
    ba_error = 0;
    for (const auto& kv : imu_meas)
	{
		if (kv.second.get_dt_ns() != 0)
		{
		    int64_t start_t = kv.second.get_start_t_ns();
		    int64_t end_t = kv.second.get_start_t_ns() + kv.second.get_dt_ns();

		    if (aom.mAbs_order_map.count(start_t) == 0 ||
			    aom.mAbs_order_map.count(end_t) == 0)
			    continue;

		    const size_t start_idx = aom.mAbs_order_map.at(start_t).first;
		    const size_t end_idx = aom.mAbs_order_map.at(end_t).first;

		    PoseVelBiasStateWithLin start_state = states.at(start_t);
		    PoseVelBiasStateWithLin end_state = states.at(end_t);

		    IntegratedImuMeasurement::MatNN d_res_d_start, d_res_d_end;
		    IntegratedImuMeasurement::MatN3 d_res_d_bg, d_res_d_ba;

		    PoseVelState::VecN res = kv.second.residual(start_state.getStateLin(),
					                                    g,
														end_state.getStateLin(),
			                                            start_state.getStateLin().mBias_gyro,
			                                            start_state.getStateLin().mBias_accel,
														&d_res_d_start,
														&d_res_d_end,
			                                            &d_res_d_bg,
														&d_res_d_ba);

		    if (start_state.isLinearized() || end_state.isLinearized())
				res = kv.second.residual(start_state.getState(),
						                 g,
										 end_state.getState(),
									     start_state.getState().mBias_gyro,
									     start_state.getState().mBias_accel);

		    // error
		    imu_error += 0.5 * res.transpose() * kv.second.get_cov_inv() * res;

		    // states
		    abs_H.block<9, 9>(start_idx, start_idx) += d_res_d_start.transpose() * kv.second.get_cov_inv() * d_res_d_start;
		    abs_H.block<9, 9>(start_idx, end_idx)   += d_res_d_start.transpose() * kv.second.get_cov_inv() * d_res_d_end;
		    abs_H.block<9, 9>(end_idx, start_idx)   += d_res_d_end.transpose()   * kv.second.get_cov_inv() * d_res_d_start;
		    abs_H.block<9, 9>(end_idx, end_idx)     += d_res_d_end.transpose()   * kv.second.get_cov_inv() * d_res_d_end;

		    abs_b.segment<9>(start_idx) += d_res_d_start.transpose() * kv.second.get_cov_inv() * res;
		    abs_b.segment<9>(end_idx)   += d_res_d_end.transpose()   * kv.second.get_cov_inv() * res;

		    // bias
		    IntegratedImuMeasurement::MatN6 d_res_d_bga;
		    d_res_d_bga.topLeftCorner<9, 3>() = d_res_d_bg;
		    d_res_d_bga.topRightCorner<9, 3>() = d_res_d_ba;

		    abs_H.block<6, 6>(start_idx + 9, start_idx + 9) += d_res_d_bga.transpose()   * kv.second.get_cov_inv() * d_res_d_bga;
		    abs_H.block<9, 6>(start_idx,     start_idx + 9) += d_res_d_start.transpose() * kv.second.get_cov_inv() * d_res_d_bga;
		    abs_H.block<9, 6>(end_idx,       start_idx + 9) += d_res_d_end.transpose()   * kv.second.get_cov_inv() * d_res_d_bga;
		    abs_H.block<6, 9>(start_idx + 9, start_idx)     += d_res_d_bga.transpose()   * kv.second.get_cov_inv() * d_res_d_start;
		    abs_H.block<6, 9>(start_idx + 9, end_idx)       += d_res_d_bga.transpose()   * kv.second.get_cov_inv() * d_res_d_end;
		    abs_b.segment<6>(start_idx + 9)                 += d_res_d_bga.transpose()   * kv.second.get_cov_inv() * res;

		    // difference between biases
		    double dt = kv.second.get_dt_ns() * 1e-9;
		    {
				Eigen::Vector3d gyro_bias_weight_dt = gyro_bias_weight / dt;

				//        std::cerr << "gyro_bias_weight_dt " <<
				//        gyro_bias_weight_dt.transpose()
				//                  << std::endl;

				Eigen::Vector3d res_bg = start_state.getState().mBias_gyro - end_state.getState().mBias_gyro;

				abs_H.block<3, 3>(start_idx + 9, start_idx + 9) += gyro_bias_weight_dt.asDiagonal();
				abs_H.block<3, 3>(end_idx   + 9, end_idx   + 9) += gyro_bias_weight_dt.asDiagonal();

				abs_H.block<3, 3>(end_idx   + 9, start_idx + 9) -= gyro_bias_weight_dt.asDiagonal();
				abs_H.block<3, 3>(start_idx + 9, end_idx   + 9) -= gyro_bias_weight_dt.asDiagonal();

				abs_b.segment<3>(start_idx + 9) += gyro_bias_weight_dt.asDiagonal() * res_bg;
				abs_b.segment<3>(end_idx   + 9) -= gyro_bias_weight_dt.asDiagonal() * res_bg;

				bg_error += 0.5 * res_bg.transpose() * gyro_bias_weight_dt.asDiagonal() * res_bg;
		    }

		    {
				Eigen::Vector3d accel_bias_weight_dt = accel_bias_weight / dt;
				Eigen::Vector3d res_ba = start_state.getState().mBias_accel - end_state.getState().mBias_accel;

				abs_H.block<3, 3>(start_idx + 12, start_idx + 12) += accel_bias_weight_dt.asDiagonal();
				abs_H.block<3, 3>(end_idx   + 12, end_idx   + 12) += accel_bias_weight_dt.asDiagonal();
				abs_H.block<3, 3>(end_idx   + 12, start_idx + 12) -= accel_bias_weight_dt.asDiagonal();
				abs_H.block<3, 3>(start_idx + 12, end_idx   + 12) -= accel_bias_weight_dt.asDiagonal();

				abs_b.segment<3>(start_idx + 12) += accel_bias_weight_dt.asDiagonal() * res_ba;
				abs_b.segment<3>(end_idx   + 12) -= accel_bias_weight_dt.asDiagonal() * res_ba;

				ba_error += 0.5 * res_ba.transpose() * accel_bias_weight_dt.asDiagonal() * res_ba;
		    }
		}//if (kv.second.get_dt_ns() != 0)
    }//for (const auto& kv : imu_meas)
//xin 20230803std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN linearizeAbsIMU()"<<std::endl;
}//void linearizeAbsIMU()


/*********************** ba_base.cpp *************************/
void Correlator::computeDelta(const AbsOrderMap&     marg_order,
                                    Eigen::VectorXd& delta) const
{
    size_t marg_size = marg_order.miTotal_size;
    delta.setZero(marg_size);
    for (const auto& kv : marg_order.mAbs_order_map)
	{
		if (kv.second.second == POSE_SIZE)
		{
//			FAIM_ASSERT(mFrame_poses.at(kv.first).isLinearized());
			delta.segment<POSE_SIZE>(kv.second.first) = mFrame_poses.at(kv.first).getDelta();
		}
		else if (kv.second.second == POSE_VEL_BIAS_SIZE)
		{
//			FAIM_ASSERT(mFrame_states.at(kv.first).isLinearized());
			delta.segment<POSE_VEL_BIAS_SIZE>(kv.second.first) = mFrame_states.at(kv.first).getDelta();
		}
		else
		{
//		    FAIM_ASSERT(false);//TODO TODO TODO TODO
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::computeDelta() fails"<<std::endl;
			exit(-1);
		}
    }
}//void computeDelta()



/*********************** ba_base.cpp *************************/
void Correlator::linearizeMargPrior(const AbsOrderMap&     marg_order,              //IN
                                    const Eigen::MatrixXd& marg_H,                  //IN
                                    const Eigen::VectorXd& marg_b,                  //IN
                                    const AbsOrderMap&     aom,                     //IN
                                          Eigen::MatrixXd& abs_H,                   //OUT
                                          Eigen::VectorXd& abs_b,                   //OUT
                                          double&          marg_prior_error) const  //OUT
{
    // Assumed to be in the top left corner
//	FAIM_ASSERT(size_t(marg_H.cols()) == marg_order.miTotal_size);

    // Check if the order of variables is the same.
//	for (const auto& kv : marg_order.mAbs_order_map)
//		FAIM_ASSERT(aom.mAbs_order_map.at(kv.first) == kv.second);

    size_t marg_size = marg_order.miTotal_size;
    abs_H.topLeftCorner(marg_size, marg_size) += marg_H;

    Eigen::VectorXd delta;
    computeDelta(marg_order,	//IN
			     delta);		//OUT

    abs_b.head(marg_size)                     += marg_b;
    abs_b.head(marg_size)                     += marg_H * delta;

    marg_prior_error =  0.5 * delta.transpose() * marg_H * delta;
    marg_prior_error += delta.transpose() * marg_b;
//xin 20230803std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN linearizeMargPrior()"<<std::endl;
}//void linearizeMargPrior()


/*********************** ba_base.cpp *************************/
void Correlator::updatePoints(const AbsOrderMap&     aom, //in
                              const RelLinData&      rld, //in
                              const Eigen::VectorXd& inc) //in
{
    Eigen::VectorXd rel_inc;
    rel_inc.setZero(rld.order.size() * POSE_SIZE);
    for (size_t i = 0; i < rld.order.size(); i++)
	{
        const TimeCamId& tcid_h = rld.order[i].first;
        const TimeCamId& tcid_t = rld.order[i].second;

		if (tcid_h.miFrame_id != tcid_t.miFrame_id)
		{
		    int abs_h_idx = aom.mAbs_order_map.at(tcid_h.miFrame_id).first;
		    int abs_t_idx = aom.mAbs_order_map.at(tcid_t.miFrame_id).first;

		    rel_inc.segment<POSE_SIZE>(i * POSE_SIZE) = rld.d_rel_d_h[i] * inc.segment<POSE_SIZE>(abs_h_idx) +
				                                        rld.d_rel_d_t[i] * inc.segment<POSE_SIZE>(abs_t_idx);
		}
    }

    for (const auto& kv : rld.lm_to_obs)
	{
		int lm_idx = kv.first;
		const auto& other_obs = kv.second;

		Eigen::Vector3d H_l_p_x;
		H_l_p_x.setZero();

		for (size_t k = 0; k < other_obs.size(); k++)
		{
		    int rel_idx = other_obs[k].first;
		    const FrameRelLinData& frld_other = rld.Hpppl.at(rel_idx);

		    Eigen::Matrix<double, 3, POSE_SIZE> H_l_p_other = frld_other.Hpl[other_obs[k].second].transpose();

		    H_l_p_x += H_l_p_other * rel_inc.segment<POSE_SIZE>(rel_idx * POSE_SIZE);

		    // std::cerr << "inc_p " << inc_p.transpose() << std::endl;
		}

		Eigen::Vector3d inc_p = rld.Hll.at(lm_idx) * (rld.bl.at(lm_idx) - H_l_p_x);

		KeypointPosition& kpt = mLmdb.getLandmark(lm_idx);
		kpt.mDir -= inc_p.head<2>();
		kpt.mdId -= inc_p[2];

		kpt.mdId = std::max(0., kpt.mdId);
    }
//xin 20230803std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN updatePoints()"<<std::endl;
}//void updatePoints()


/*********************** ba_base.cpp *************************/
void Correlator::computeError(double&                                                   error,
                              std::map<int, std::vector<std::pair<TimeCamId, double>>>* outliers,
                              double outlier_threshold) const
{
    error = 0;

    for (const auto& kv : mLmdb.getObservations())
	{
		const TimeCamId& tcid_h = kv.first;

		for (const auto& obs_kv : kv.second)
		{
		    const TimeCamId& tcid_t = obs_kv.first;

		    if (tcid_h != tcid_t)
			{
				PoseStateWithLin state_h = getPoseStateWithLin(tcid_h.miFrame_id);
				PoseStateWithLin state_t = getPoseStateWithLin(tcid_t.miFrame_id);

				Sophus::SE3d T_t_h_sophus = computeRelPose(state_h.getPose(),				//IN
						                                   mCalib.mT_i_c[tcid_h.miCam_id],	//IN
								                           state_t.getPose(),				//IN
														   mCalib.mT_i_c[tcid_t.miCam_id]);	//IN
//xin 20230803			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN computeError(): AFTER computeRelPose()"<<std::endl;

				Eigen::Matrix4d T_t_h = T_t_h_sophus.matrix();

//				std::visit([&](const auto& cam)
//						   {
						       for (size_t i = 0; i < obs_kv.second.size(); i++)
						       {
								   const KeypointObservation& kpt_obs = obs_kv.second[i];
								   const KeypointPosition& kpt_pos = mLmdb.getLandmark(kpt_obs.miKpt_id);

								   Eigen::Vector2d res;

								   bool valid = linearizePoint(kpt_obs,
										                       kpt_pos,
															   T_t_h,
															   mCalib.mIntrinsics[tcid_t.miCam_id],//cam,
															   res);

								  if (valid)
								  {
									  double e = res.norm();

									  if (outliers && e > outlier_threshold)
										  (*outliers)[kpt_obs.miKpt_id].emplace_back(tcid_t,
												                                   	 e);

									  double huber_weight = e < mdHuber_thresh ? 1.0 : mdHuber_thresh / e;
									  double obs_weight = huber_weight / (mdObs_std_dev * mdObs_std_dev);

									  error += (2 - huber_weight) * obs_weight * res.transpose() * res;
								  }
								  else
								  {
								      if (outliers)
									      (*outliers)[kpt_obs.miKpt_id].emplace_back(tcid_t, -1);
								  }
						       }
//						   }, mCalib.mIntrinsics[tcid_t.miCam_id].variant);

		    }
			else
			{
				// target and host are the same
				// residual does not depend on the pose
				// it just depends on the point

//				std::visit([&](const auto& cam)
//						   {
							  for (size_t i = 0; i < obs_kv.second.size(); i++)
							  {
							      const KeypointObservation& kpt_obs = obs_kv.second[i];
								  const KeypointPosition& kpt_pos = mLmdb.getLandmark(kpt_obs.miKpt_id);

								  Eigen::Vector2d res;

								  bool valid = linearizePoint(kpt_obs,
										                      kpt_pos,
															  mCalib.mIntrinsics[tcid_t.miCam_id],//cam,
															  res);
								  if (valid)
								  {
								      double e = res.norm();

									  if (outliers && e > outlier_threshold)
										(*outliers)[kpt_obs.miKpt_id].emplace_back(tcid_t, -2);//TODO: configurate this value in config.h!!!

									  double huber_weight = e < mdHuber_thresh ? 1.0 : mdHuber_thresh / e;
									  double obs_weight = huber_weight / (mdObs_std_dev * mdObs_std_dev);

									  error += (2 - huber_weight) * obs_weight * res.transpose() * res;
								  }
								  else
								  {
								      if (outliers)
									      (*outliers)[kpt_obs.miKpt_id].emplace_back(tcid_t,
												                                     -2);//TODO: configurate this value in config.h!!!
								  }
							  }
//					       }, mCalib.mIntrinsics[tcid_t.miCam_id].variant);
		    }
		}//for (const auto& obs_kv : kv.second)
    }//for (const auto& kv : lmdb.getObservations())
//xin 20230803std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN computeError()"<<std::endl;
}//void computeError()

/****************** keypoint_vio_linearize.cpp  ********************/
void Correlator::computeImuError(const AbsOrderMap&												aom,
									   double&													imu_error,
									   double&													bg_error,
    								   double&													ba_error,
   								 const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>&	states,
    							 const Eigen::aligned_map<int64_t, IntegratedImuMeasurement>&	imu_meas,
								 const Eigen::Vector3d&											gyro_bias_weight,
								 const Eigen::Vector3d&											accel_bias_weight,
								 const Eigen::Vector3d&											g)
{
	imu_error = 0;
	bg_error = 0;
	ba_error = 0;
	for (const auto& kv : imu_meas)
	{
		if (kv.second.get_dt_ns() != 0)
		{
			int64_t start_t = kv.second.get_start_t_ns();
			int64_t end_t = kv.second.get_start_t_ns() + kv.second.get_dt_ns();

			if (aom.mAbs_order_map.count(start_t) == 0 || aom.mAbs_order_map.count(end_t) == 0)
				continue;

			PoseVelBiasStateWithLin start_state = states.at(start_t);
			PoseVelBiasStateWithLin end_state = states.at(end_t);

			const PoseVelState::VecN res = kv.second.residual(start_state.getState(),
															  g,
															  end_state.getState(),
															  start_state.getState().mBias_gyro,
															  start_state.getState().mBias_accel);

			//      std::cout << "res: (" << start_t << "," << end_t << ") "
			//                << res.transpose() << std::endl;

			//      std::cerr << "cov_inv:\n" << kv.second.get_cov_inv() <<
			//      std::endl;

			imu_error += 0.5 * res.transpose() * kv.second.get_cov_inv() * res;

			double dt = kv.second.get_dt_ns() * 1e-9;
			Eigen::Vector3d gyro_bias_weight_dt = gyro_bias_weight / dt;
			Eigen::Vector3d res_bg = start_state.getState().mBias_gyro - end_state.getState().mBias_gyro;
			bg_error += 0.5 * res_bg.transpose() * gyro_bias_weight_dt.asDiagonal() * res_bg;

			Eigen::Vector3d accel_bias_weight_dt = accel_bias_weight / dt;
			Eigen::Vector3d res_ba = start_state.getState().mBias_accel - end_state.getState().mBias_accel;
			ba_error += 0.5 * res_ba.transpose() * accel_bias_weight_dt.asDiagonal() * res_ba;
		}
	}
//xin 20230803std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN computeImuError()"<<std::endl;
}//void computeImuError()

/************ ba_base.h moved to .cpp by xin************/
inline void Correlator::backup()
{
	for (auto& kv : mFrame_states)
		kv.second.backup();

	for (auto& kv : mFrame_poses)
		kv.second.backup();

	mLmdb.backup();
}

inline void Correlator::restore()
{
	for (auto& kv : mFrame_states)
		kv.second.restore();

	for (auto& kv : mFrame_poses)
		kv.second.restore();
	
	mLmdb.restore();
}


/*********************** ba_base.cpp *************************/
void Correlator::filterOutliers(double outlier_threshold,
                                int	   min_num_obs)
{
	double error;
	std::map<int, std::vector<std::pair<TimeCamId, double>>> outliers;
	computeError(error,
			     &outliers,
				 outlier_threshold);

	//  std::cout << "============================================" <<
	//  std::endl; std::cout << "Num landmarks: " << lmdb.numLandmarks() << "
	//  with outliners
	//  "
	//            << outliers.size() << std::endl;

	for (const auto& kv : outliers)
	{
		int num_obs = mLmdb.numObservations(kv.first);
		int num_outliers = kv.second.size();

		bool remove = false;

		if (num_obs - num_outliers < min_num_obs) remove = true;

		//    std::cout << "\tlm_id: " << kv.first << " num_obs: " << num_obs
		//              << " outliers: " << num_outliers << " [";

		for (const auto& kv2 : kv.second)
		{
			if (kv2.second == -2) remove = true;
		  //      std::cout << kv2.second << ", ";
		}
		//    std::cout << "] " << std::endl;

		if (remove)
			mLmdb.removeLandmark(kv.first);
		else
		{
			std::set<TimeCamId> outliers;
			for (const auto& kv2 : kv.second)
				outliers.emplace(kv2.first);
			mLmdb.removeObservations(kv.first, outliers);
		}
	}

	// std::cout << "============================================" <<
	// std::endl;
}

/*********************** ba_base.cpp *************************/
void Correlator::computeMargPriorError(const AbsOrderMap&     marg_order,
		                               const Eigen::MatrixXd& marg_H,
                                       const Eigen::VectorXd& marg_b,
											 double&          marg_prior_error) const
{
//	FAIM_ASSERT(size_t(marg_H.cols()) == marg_order.miTotal_size);

    Eigen::VectorXd delta;
    computeDelta(marg_order, delta);

    marg_prior_error = 0.5 * delta.transpose() * marg_H * delta;
    marg_prior_error += delta.transpose() * marg_b;
//xin 20230803std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN computeMargPriorError()"<<std::endl;
}

void Correlator::optimize()
{
	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::optimize(): mFrame_states.size() = "<<mFrame_states.size()<<std::endl;
    if (mbOpt_started || mFrame_states.size() > 4)
	{
        // Optimize
        mbOpt_started = true;

        AbsOrderMap aom;

        for (const auto& kv : mFrame_poses)
		{
            aom.mAbs_order_map[kv.first] = std::make_pair(aom.miTotal_size,
					                                      POSE_SIZE);

            // Check that we have the same order as marginalization
//			FAIM_ASSERT(mMarg_order.mAbs_order_map.at(kv.first) == aom.mAbs_order_map.at(kv.first));

            aom.miTotal_size += POSE_SIZE;
            aom.miItems++;
        }

        for (const auto& kv : mFrame_states)
		{
            aom.mAbs_order_map[kv.first] = std::make_pair(aom.miTotal_size,
					                                      POSE_VEL_BIAS_SIZE);

            // Check that we have the same order as marginalization
//			if (aom.miItems < mMarg_order.mAbs_order_map.size())
//				FAIM_ASSERT(mMarg_order.mAbs_order_map.at(kv.first) == aom.mAbs_order_map.at(kv.first));

            aom.miTotal_size += POSE_VEL_BIAS_SIZE;
            aom.miItems++;
        }

//xin 20230803	std::cout << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::optimize(): opt order" << std::endl;
//xin 20230803	aom.print_order();

//xin 20230803	std::cout << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::optimize():  marg prior order" << std::endl;
//xin 20230803	mMarg_order.print_order();

        for (int iter = 0; iter < mConfig.miVio_max_iterations; iter++)
		{
			auto t1 = std::chrono::high_resolution_clock::now();

			double rld_error;
			Eigen::aligned_vector<RelLinData> rld_vec;
			linearizeHelper(rld_vec,                 //out
					        mLmdb.getObservations(), //in
							rld_error);              //out

			LinearizeAbsReduce<DenseAccumulator<double>> lopt(aom);

			tbb::blocked_range<Eigen::aligned_vector<RelLinData>::iterator> range(rld_vec.begin(), rld_vec.end());

			tbb::parallel_reduce(range, lopt);

			double marg_prior_error = 0;
			double imu_error = 0, bg_error = 0, ba_error = 0;
			linearizeAbsIMU(aom,
							lopt.mAccum.getH(), //out
							lopt.mAccum.getB(), //out
							imu_error,         //out
							bg_error,          //out
							ba_error,          //out
							mFrame_states,
							mImu_meas,
							mGyro_bias_weight, 
							mAccel_bias_weight,
							g);
			linearizeMargPrior(mMarg_order,
							   mMarg_H,
							   mMarg_b,
							   aom,
							   lopt.mAccum.getH(), //out
							   lopt.mAccum.getB(), //out
							   marg_prior_error); //out

			double error_total = rld_error + imu_error + marg_prior_error + ba_error + bg_error;

//			if (mConfig.vio_debug)
//xin 20230803				std::cout << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::optimize():  [LINEARIZE] Error:        " << error_total << " num points " << std::endl
	//xin 20230803					  << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH								= rld_error :        " << rld_error << std::endl
	//xin 20230803					  << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH								+ imu_error :        " << imu_error << std::endl
	//xin 20230803					  << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH								+ marg_prior_error : " << marg_prior_error << std::endl
	//xin 20230803					  << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH								+ ba_error 	:        " << ba_error << std::endl
	//xin 20230803					  << "HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH								+ bg_error  :        " << bg_error << std::endl;

			lopt.mAccum.setup_solver();
			Eigen::VectorXd Hdiag = lopt.mAccum.Hdiagonal();

			bool converged = false;

			if (mConfig.mbVio_use_lm)
			{   // Use Levenberg–Marquardt
				bool step = false;//TODO:configurate this boolean in config.h
				int max_iter = 10;//TODO:configurate this number in config.h

				while (!step && max_iter > 0 && !converged)
				{
					Eigen::VectorXd Hdiag_lambda = Hdiag * mdLambda;
					for (int i = 0; i < Hdiag_lambda.size(); i++)
						Hdiag_lambda[i] = std::max(Hdiag_lambda[i], mdMin_lambda);

					const Eigen::VectorXd inc = lopt.mAccum.solve(&Hdiag_lambda);
					double max_inc = inc.array().abs().maxCoeff();
					if (max_inc < 1e-4)//TODO:configurate this number in config.h
						converged = true;

					backup();

					// apply increment to poses
					for (auto& kv : mFrame_poses)
					{
						int idx = aom.mAbs_order_map.at(kv.first).first;
						kv.second.applyInc(-inc.segment<POSE_SIZE>(idx));
					}

					// apply increment to states
					for (auto& kv : mFrame_states)
					{
						int idx = aom.mAbs_order_map.at(kv.first).first;
						kv.second.applyInc(-inc.segment<POSE_VEL_BIAS_SIZE>(idx));
					}

					// Update points
					tbb::blocked_range<size_t> keys_range(0, rld_vec.size());
					auto update_points_func = [&](const tbb::blocked_range<size_t>& r)
					{
						for (size_t i = r.begin(); i != r.end(); ++i)
						{
							const auto& rld = rld_vec[i];
							updatePoints(aom,
										 rld,
										 inc);
						}
					};
					tbb::parallel_for(keys_range, update_points_func);

					double after_update_marg_prior_error = 0;
					double after_update_vision_error = 0, after_update_imu_error = 0, after_bg_error = 0, after_ba_error = 0;

					computeError(after_update_vision_error);//OUT
					computeImuError(aom,					//IN
									after_update_imu_error,	//OUT
									after_bg_error,			//OUT
									after_ba_error,			//OUT
									mFrame_states,			//IN
									mImu_meas,				//IN
									mGyro_bias_weight,		//IN
									mAccel_bias_weight,		//IN
									g);						//IN
					computeMargPriorError(mMarg_order,
										  mMarg_H,
										  mMarg_b,
										  after_update_marg_prior_error);

					double after_error_total = after_update_vision_error + 
											   after_update_imu_error +
											   after_update_marg_prior_error + 
											   after_bg_error + 
											   after_ba_error;

					double f_diff = (error_total - after_error_total);

					if (f_diff < 0)
					{
//						if (mConfig.vio_debug)
							std::cout << "\t[REJECTED] mdLambda:" << mdLambda
									  << " f_diff: " << f_diff
									  << " max_inc: " << max_inc
									  << " Error: " << after_error_total << std::endl;
						mdLambda = std::min(mdMax_lambda, mdLambda_vee * mdLambda);
						mdLambda_vee *= 2;

						restore();
					}
					else
					{
//						if (mConfig.vio_debug)
							std::cout << "\t[ACCEPTED] mdLambda:" << mdLambda
									  << " f_diff: " << f_diff
									  << " max_inc: " << max_inc
									  << " Error: " << after_error_total << std::endl;

						mdLambda = std::max(mdMin_lambda, mdLambda / 3);
						mdLambda_vee = 2;

						step = true;
					}
					max_iter--;
				}//while (!step && max_iter > 0 && !converged)

//				if (mConfig.vio_debug && converged)
				if (converged)
					std::cout << "[CONVERGED]" << std::endl;
			}//if (mConfig.mbVio_use_lm)
			else
			{  // Use Gauss-Newton
				Eigen::VectorXd Hdiag_lambda = Hdiag * mdMin_lambda;
				for (int i = 0; i < Hdiag_lambda.size(); i++)
					Hdiag_lambda[i] = std::max(Hdiag_lambda[i], mdMin_lambda);

				const Eigen::VectorXd inc = lopt.mAccum.solve(&Hdiag_lambda);
				double max_inc = inc.array().abs().maxCoeff();
				if (max_inc < 1e-4)
					converged = true;

				// apply increment to poses
				for (auto& kv : mFrame_poses)
				{
					int idx = aom.mAbs_order_map.at(kv.first).first;
					kv.second.applyInc(-inc.segment<POSE_SIZE>(idx));
				}

				// apply increment to states
				for (auto& kv : mFrame_states)
				{
					int idx = aom.mAbs_order_map.at(kv.first).first;
					kv.second.applyInc(-inc.segment<POSE_VEL_BIAS_SIZE>(idx));
				}

				// Update points
				tbb::blocked_range<size_t> keys_range(0, rld_vec.size());
				auto update_points_func = [&](const tbb::blocked_range<size_t>& r)
				{
					for (size_t i = r.begin(); i != r.end(); ++i)
					{
						const auto& rld = rld_vec[i];
						updatePoints(aom, rld, inc);
					}
				};
				tbb::parallel_for(keys_range, update_points_func);
			}//if (mConfig.mbVio_use_lm) else

//			if (mConfig.vio_debug)
			{
				double after_update_marg_prior_error = 0;
				double after_update_vision_error = 0, after_update_imu_error = 0, after_bg_error = 0, after_ba_error = 0;

				computeError(after_update_vision_error);
				computeImuError(aom,
								after_update_imu_error,
								after_bg_error,
								after_ba_error,
								mFrame_states,
								mImu_meas,
								mGyro_bias_weight,
								mAccel_bias_weight,
								g);
				computeMargPriorError(mMarg_order,
									  mMarg_H,
									  mMarg_b,
									  after_update_marg_prior_error);

				double after_error_total = after_update_vision_error + 
										   after_update_imu_error +
										   after_update_marg_prior_error + 
										   after_bg_error + 
										   after_ba_error;

				double error_diff = error_total - after_error_total;

				auto t2 = std::chrono::high_resolution_clock::now();

				auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

				std::cout<<std::endl;
				std::cout << "iter " << iter
						  << " before_update_error: vision: " << rld_error
						  << " imu: " << imu_error << " bg_error: " << bg_error
						  << " ba_error: " << ba_error
						  << " marg_prior: " << marg_prior_error
						  << " total: " << error_total << std::endl;

				std::cout << "iter " << iter << "  after_update_error: vision: "
						  << after_update_vision_error
						  << " imu: " << after_update_imu_error
						  << " bg_error: " << after_bg_error
						  << " ba_error: " << after_ba_error
						  << " marg prior: " << after_update_marg_prior_error
						  << " total: " << after_error_total << " error_diff "
						  << error_diff << " time : " << elapsed.count()
						  << "(us),  num_states " << mFrame_states.size()
						  << " num_poses " << mFrame_poses.size() << std::endl;

				if (after_error_total > error_total)
					std::cout << "increased error after update!!!" << std::endl;
			}//if (mConfig.vio_debug)

			if (iter == mConfig.miVio_filter_iteration)
				filterOutliers(mConfig.mdVio_outlier_threshold, 4);//TODO:configurate '4' in config.h!!!

			if (converged) break;

			// std::cerr << "LT\n" << LT << std::endl;
			// std::cerr << "z_p\n" << z_p.transpose() << std::endl;
			// std::cerr << "inc\n" << inc.transpose() << std::endl;
        }//for (int iter = 0; iter < mConfig.vio_max_iterations; iter++)
    }//if (mbOpt_started || mFrame_states.size() > 4)

//	if (mConfig.vio_debug)
//		std::cout << "=================================" << std::endl;
}  // namespace basalt


/*********************** ba_base.cpp *************************/
Eigen::VectorXd Correlator::checkNullspace(const Eigen::MatrixXd&                                      H,
		                                   const Eigen::VectorXd&                                      b,
                                           const AbsOrderMap&                                          order,
                                           const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>& frame_states,
                                           const Eigen::aligned_map<int64_t, PoseStateWithLin>&        frame_poses)
{
//	FAIM_ASSERT(size_t(H.cols()) == order.miTotal_size);
    size_t marg_size = order.miTotal_size;

    Eigen::VectorXd inc_x, inc_y, inc_z, inc_roll, inc_pitch, inc_yaw;
    inc_x.setZero(marg_size);
    inc_y.setZero(marg_size);
    inc_z.setZero(marg_size);
    inc_roll.setZero(marg_size);
    inc_pitch.setZero(marg_size);
    inc_yaw.setZero(marg_size);

    int num_trans = 0;
    Eigen::Vector3d mean_trans;
    mean_trans.setZero();

    // Compute mean translation
    for (const auto& kv : order.mAbs_order_map)
	{
		Eigen::Vector3d trans;
		if (kv.second.second == POSE_SIZE)
		{
		    mean_trans += frame_poses.at(kv.first).getPoseLin().translation();
		    num_trans++;
		}
		else if (kv.second.second == POSE_VEL_BIAS_SIZE)
		{
		    mean_trans += frame_states.at(kv.first).getStateLin().mT_w_i.translation();
		    num_trans++;
		}
		else
		{
		    std::cerr << "Unknown size of the state: " << kv.second.second << std::endl;
		    std::abort();
		}
    }
    mean_trans /= num_trans;

    double eps = 0.01;//TODO: configurate this value in config.h!!!

    // Compute nullspace increments
    for (const auto& kv : order.mAbs_order_map)
	{
		inc_x(kv.second.first + 0) = eps;
		inc_y(kv.second.first + 1) = eps;
		inc_z(kv.second.first + 2) = eps;
		inc_roll(kv.second.first + 3) = eps;
		inc_pitch(kv.second.first + 4) = eps;
		inc_yaw(kv.second.first + 5) = eps;

		Eigen::Vector3d trans;
		if (kv.second.second == POSE_SIZE)
		    trans = frame_poses.at(kv.first).getPoseLin().translation();
		else if (kv.second.second == POSE_VEL_BIAS_SIZE)
		    trans = frame_states.at(kv.first).getStateLin().mT_w_i.translation();
		else exit(-1);//TODO TODO TODO TODO
//			FAIM_ASSERT(false);

		trans -= mean_trans;

		Eigen::Matrix3d J = -Sophus::SO3d::hat(trans);
		J *= eps;

		inc_roll.segment<3>(kv.second.first) = J.col(0);
		inc_pitch.segment<3>(kv.second.first) = J.col(1);
		inc_yaw.segment<3>(kv.second.first) = J.col(2);

		if (kv.second.second == POSE_VEL_BIAS_SIZE)
		{
		    Eigen::Vector3d vel = frame_states.at(kv.first).getStateLin().mVel_w_i;
		    Eigen::Matrix3d J_vel = -Sophus::SO3d::hat(vel);
		    J_vel *= eps;

		    inc_roll.segment<3>(kv.second.first + POSE_SIZE) = J_vel.col(0);
		    inc_pitch.segment<3>(kv.second.first + POSE_SIZE) = J_vel.col(1);
		    inc_yaw.segment<3>(kv.second.first + POSE_SIZE) = J_vel.col(2);
		}
    }

    inc_x.normalize();
    inc_y.normalize();
    inc_z.normalize();
    inc_roll.normalize();
    inc_pitch.normalize();
    inc_yaw.normalize();

    //  std::cout << "inc_x   " << inc_x.transpose() << std::endl;
    //  std::cout << "inc_y   " << inc_y.transpose() << std::endl;
    //  std::cout << "inc_z   " << inc_z.transpose() << std::endl;
    //  std::cout << "inc_yaw " << inc_yaw.transpose() << std::endl;

    Eigen::VectorXd inc_random;
    inc_random.setRandom(marg_size);
    inc_random.normalize();

    Eigen::VectorXd xHx(7), xb(7);
    xHx[0] = 0.5 * inc_x.transpose()      * H * inc_x;
    xHx[1] = 0.5 * inc_y.transpose()      * H * inc_y;
    xHx[2] = 0.5 * inc_z.transpose()      * H * inc_z;
    xHx[3] = 0.5 * inc_roll.transpose()   * H * inc_roll;
    xHx[4] = 0.5 * inc_pitch.transpose()  * H * inc_pitch;
    xHx[5] = 0.5 * inc_yaw.transpose()    * H * inc_yaw;
    xHx[6] = 0.5 * inc_random.transpose() * H * inc_random;

    xb[0] = inc_x.transpose()      * b;
    xb[1] = inc_y.transpose()      * b;
    xb[2] = inc_z.transpose()      * b;
    xb[3] = inc_roll.transpose()   * b;
    xb[4] = inc_pitch.transpose()  * b;
    xb[5] = inc_yaw.transpose()    * b;
    xb[6] = inc_random.transpose() * b;

    std::cout << "nullspace x_trans: " << xHx[0] << " + " << xb[0] << std::endl;
    std::cout << "nullspace y_trans: " << xHx[1] << " + " << xb[1] << std::endl;
    std::cout << "nullspace z_trans: " << xHx[2] << " + " << xb[2] << std::endl;
    std::cout << "nullspace roll   : " << xHx[3] << " + " << xb[3] << std::endl;
    std::cout << "nullspace pitch  : " << xHx[4] << " + " << xb[4] << std::endl;
    std::cout << "nullspace yaw    : " << xHx[5] << " + " << xb[5] << std::endl;
    std::cout << "nullspace random : " << xHx[6] << " + " << xb[6] << std::endl;

    return xHx + xb;
}


/*********************** ba_base.cpp *************************/
/*//moved to correlator.h as static function
void Correlator::linearizeRel(const RelLinData&       rld,  //IN
                                    Eigen::MatrixXd&  H,    //OUT
                                    Eigen::VectorXd&  b)    //OUT
{
    //  std::cout << "linearizeRel: KF " << frame_states.size() << " obs "
    //            << obs.size() << std::endl;

    // Do schur complement
    size_t msize = rld.order.size();
    H.setZero(POSE_SIZE * msize, POSE_SIZE * msize);
    b.setZero(POSE_SIZE * msize);

    for (size_t i = 0; i < rld.order.size(); i++)
	{
		const FrameRelLinData& frld = rld.Hpppl.at(i);

		H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * i)               += frld.Hpp;
		b.segment<POSE_SIZE>(POSE_SIZE * i)                                       += frld.bp;

		for (size_t j = 0; j < frld.lm_id.size(); j++)
		{
		    Eigen::Matrix<double, POSE_SIZE, 3> H_pl_H_ll_inv;
		    int lm_id = frld.lm_id[j];

		    H_pl_H_ll_inv = frld.Hpl[j] * rld.Hll.at(lm_id);
		    b.segment<POSE_SIZE>(POSE_SIZE * i)                                   -= H_pl_H_ll_inv * rld.bl.at(lm_id);

		    const auto& other_obs = rld.lm_to_obs.at(lm_id);
		    for (size_t k = 0; k < other_obs.size(); k++)
			{
				const FrameRelLinData& frld_other = rld.Hpppl.at(other_obs[k].first);
				int other_i = other_obs[k].first;

				Eigen::Matrix<double, 3, POSE_SIZE> H_l_p_other = frld_other.Hpl[other_obs[k].second].transpose();

				H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * other_i) -= H_pl_H_ll_inv * H_l_p_other;
		    }
		}//for (size_t j = 0; j < frld.lm_id.size(); j++)
    }//for (size_t i = 0; i < rld.order.size(); i++)
}
*/
/*********************** ba_base.cpp *************************/
void Correlator::marginalizeHelper(      Eigen::MatrixXd&     abs_H,        //in & out
                                         Eigen::VectorXd&     abs_b,        //in & out
                                   const std::set<int>&       idx_to_keep,  //in
                                   const std::set<int>&       idx_to_marg,  //in
                                         Eigen::MatrixXd&     marg_H,       //out
                                         Eigen::VectorXd&     marg_b)       //out
{
    int keep_size = idx_to_keep.size();
    int marg_size = idx_to_marg.size();

//	FAIM_ASSERT(keep_size + marg_size == abs_H.cols());

    // Fill permutation matrix
    Eigen::Matrix<int, Eigen::Dynamic, 1> indices(idx_to_keep.size() +
                                                  idx_to_marg.size());

    auto it = idx_to_keep.begin();
    for (size_t i = 0; i < idx_to_keep.size(); i++)
	{
        indices[i] = *it;
        it++;
    }

/*	auto it = idx_to_marg.begin();
    for (size_t i = 0; i < idx_to_marg.size(); i++)
	{
        indices[idx_to_keep.size() + i] = *it;
        it++;
    }
*/
	auto it2 = idx_to_marg.begin();
    for (size_t i = 0; i < idx_to_marg.size(); i++)
	{
        indices[idx_to_keep.size() + i] = *it2;
        it2++;
    }


    const Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>> p(indices);
    const Eigen::PermutationMatrix<Eigen::Dynamic> pt = p.transpose();

    abs_b.applyOnTheLeft(pt);
    abs_H.applyOnTheLeft(pt);
    abs_H.applyOnTheRight(p);

    Eigen::MatrixXd H_mm_inv;
    //  H_mm_inv.setIdentity(marg_size, marg_size);
    //  abs_H.bottomRightCorner(marg_size,
    //  marg_size).ldlt().solveInPlace(H_mm_inv);

    H_mm_inv = abs_H.bottomRightCorner(marg_size, marg_size).fullPivLu()
                 .solve(Eigen::MatrixXd::Identity(marg_size, marg_size));

    //  H_mm_inv = abs_H.bottomRightCorner(marg_size, marg_size)
    //                 .fullPivHouseholderQr()
    //                 .solve(Eigen::MatrixXd::Identity(marg_size,
    //                 marg_size));

    abs_H.topRightCorner(keep_size, marg_size) *= H_mm_inv;

    marg_H = abs_H.topLeftCorner(keep_size, keep_size);
    marg_b = abs_b.head(keep_size);

    marg_H -= abs_H.topRightCorner(keep_size, marg_size) *
              abs_H.bottomLeftCorner(marg_size, keep_size);
    marg_b -= abs_H.topRightCorner(keep_size, marg_size) * abs_b.tail(marg_size);

    abs_H.resize(0, 0);
    abs_b.resize(0);
}

void Correlator::checkMargNullspace() const
{
    checkNullspace(mMarg_H,
			       mMarg_b,
				   mMarg_order,
				   mFrame_states,
				   mFrame_poses);
}

void Correlator::marginalize(const std::map<int64_t, int>& num_points_connected)
{
    if (!mbOpt_started) return;
	
	std::cout<<std::endl<<std::endl;
	std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<< ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> "<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<< "Correlator::marginalize(): mFrame_poses.size() = "<<mFrame_poses.size() <<", mFrame_states.size() = "<<mFrame_states.size()<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	std::cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<< ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> "<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl<<std::endl;

    if (mFrame_poses.size() > mConfig.miVio_max_kfs || mFrame_states.size() >= mConfig.miVio_max_states)
	{
        // Marginalize

        const int states_to_remove = mFrame_states.size() - mConfig.miVio_max_states + 1;

        auto it = mFrame_states.cbegin();
        for (int i = 0; i < states_to_remove; i++) it++;//xin: this means always marginalize the first state (note: differentiate 'pose' from 'state')
        int64_t last_state_to_marg = it->first;

        AbsOrderMap aom;

        // remove all mFrame_poses that are not kfs
        std::set<int64_t> poses_to_marg;
        for (const auto& kv : mFrame_poses)
		{
            aom.mAbs_order_map[kv.first] = std::make_pair(aom.miTotal_size,
					                                      POSE_SIZE);

            if (mKf_ids.count(kv.first) == 0) poses_to_marg.emplace(kv.first);

            // Check that we have the same order as marginalization
//			FAIM_ASSERT(mMarg_order.mAbs_order_map.at(kv.first) == aom.mAbs_order_map.at(kv.first));

            aom.miTotal_size += POSE_SIZE;
            aom.miItems++;
        }

        std::set<int64_t> states_to_marg_vel_bias;
        std::set<int64_t> states_to_marg_all;
        for (const auto& kv : mFrame_states)
		{
            if (kv.first > last_state_to_marg) break;

            if (kv.first != last_state_to_marg)
			{
                if (mKf_ids.count(kv.first) > 0)
                    states_to_marg_vel_bias.emplace(kv.first);
                else
                    states_to_marg_all.emplace(kv.first);
            }

            aom.mAbs_order_map[kv.first] = std::make_pair(aom.miTotal_size,
					                                      POSE_VEL_BIAS_SIZE);//implicitly includes 2 cases: Fig.5(b) and (c). Very intricate. Mind yourself.

//			// Check that we have the same order as marginalization
//			if (aom.miItems < mMarg_order.mAbs_order_map.size())
//				FAIM_ASSERT(mMarg_order.mAbs_order_map.at(kv.first) == aom.mAbs_order_map.at(kv.first));

            aom.miTotal_size += POSE_VEL_BIAS_SIZE;
            aom.miItems++;
        }

        auto kf_ids_all = mKf_ids;
        std::set<int64_t> kfs_to_marg;
        while (mKf_ids.size() > mConfig.miVio_max_kfs && !states_to_marg_vel_bias.empty())//Fig.5(a) and (c).
		{
            int64_t id_to_marg = -1;

			//xin: 2 special cases:(1) no landmarks connected to this keyframe; (2) suddenly a large bunch of landmarks (compared to the number of landmarks already connected to this keyframe) added to this keyframe.
            std::vector<int64_t> ids;
            for (int64_t id : mKf_ids)
                ids.push_back(id);
            for (size_t i = 0; i < ids.size() - 2; i++)
                if (num_points_connected.count(ids[i]) == 0 || (num_points_connected.at(ids[i]) / mNum_points_kf.at(ids[i]) < 0.05))//TODO: configurate this value in config.h!!!
				{
                    id_to_marg = ids[i];
                    break;
                }

			//xin: the nominal case: current keyframe is to be marginalized if (1) translation btw keyframe i(current) and j are minimal (low disparity), plus(2) translation btw keyframe i(current) and the last keyframe is the largest(information decaying)
            if (id_to_marg < 0)
			{
                std::vector<int64_t> ids;
                for (int64_t id : mKf_ids)
                    ids.push_back(id);

                int64_t last_kf = *mKf_ids.crbegin();
                double min_score = std::numeric_limits<double>::max();
                int64_t min_score_id = -1;

                for (size_t i = 0; i < ids.size() - 2; i++)
				{
                    double denom = 0;
                    for (size_t j = 0; j < ids.size() - 2; j++)
                        denom += 1 / ((mFrame_poses.at(ids[i]).getPose().translation() - mFrame_poses.at(ids[j]).getPose().translation()).norm() +
                                      1e-5);

                    double score = std::sqrt((mFrame_poses.at(ids[i]).getPose().translation() - mFrame_states.at(last_kf).getState().mT_w_i.translation()).norm()) *
                                   denom;

                    if (score < min_score)
		            {
                        min_score_id = ids[i];
                        min_score = score;
                    }
                }

                id_to_marg = min_score_id;
            }

            kfs_to_marg.emplace(id_to_marg);
            poses_to_marg.emplace(id_to_marg);

            mKf_ids.erase(id_to_marg);
        }//while (mKf_ids.size() > mConfig.miVio_max_kfs && !states_to_marg_vel_bias.empty())

		//    std::cout << "marg order" << std::endl;
		//    aom.print_order();

		//    std::cout << "marg prior order" << std::endl;
		//    mMarg_order.print_order();

//		if (mConfig.vio_debug)
		{
	//xin 20230803	    std::cout << "states_to_remove " << states_to_remove << std::endl;
	//xin 20230803	    std::cout << "poses_to_marg.size() " << poses_to_marg.size() << std::endl;
	//xin 20230803	    std::cout << "states_to_marg.size() " << states_to_marg_all.size() << std::endl;
	//xin 20230803	    std::cout << "states_to_marg_vel_bias.size() " << states_to_marg_vel_bias.size() << std::endl;
	//xin 20230803	    std::cout << "kfs_to_marg.size() " << kfs_to_marg.size() << std::endl;
		}

		size_t asize = aom.miTotal_size;

		double marg_prior_error;
		double imu_error, bg_error, ba_error;

		DenseAccumulator<double> accum;//DenseAccumulator accum;
		accum.reset(asize);

		// Linearize points
		Eigen::aligned_map<TimeCamId,
			               Eigen::aligned_map<TimeCamId,
						                      Eigen::aligned_vector<KeypointObservation>>> obs_to_lin;

		//xin: whether to marginalize out a landmark: condition 1&2
		for (auto it = mLmdb.getObservations().cbegin(); it != mLmdb.getObservations().cend();)
		{
			if (kfs_to_marg.count(it->first.miFrame_id) > 0)//xin: (condition 1) current keyframe-to-marginalize DOES have observations
			{
				for (auto it2 = it->second.cbegin(); it2 != it->second.cend(); ++it2)//xin: for each other keyframe that 'sees' common landmarks(thus becoming observations taged with miFrame_id). 
					if (it2->first.miFrame_id <= last_state_to_marg)//xin: (condition 2) if these observations were inserted before the leftmost frame timestamp (not keyframe-to_marginalize!) 
						obs_to_lin[it->first].emplace(*it2);
			}
			++it;
		}

		double rld_error;
		Eigen::aligned_vector<RelLinData> rld_vec;

		linearizeHelper(rld_vec,   //out
				        obs_to_lin,//in
						rld_error);//out

		for (auto& rld : rld_vec)
		{
			rld.invert_keypoint_hessians();

			Eigen::MatrixXd rel_H;
			Eigen::VectorXd rel_b;
			linearizeRel(rld,    //in
					     rel_H,  //out
						 rel_b); //out

			//update accumulator.H and accumulator.b
			linearizeAbs(rel_H, //in
					     rel_b, //in
						 rld,   //in
						 aom,   //in
						 accum);//in & out
		}

		linearizeAbsIMU(aom,
				        accum.getH(), //out
				        accum.getB(), //out
						imu_error,    //out
						bg_error,     //out
						ba_error,     //out
						mFrame_states, 
						mImu_meas,
						mGyro_bias_weight,
						mAccel_bias_weight,
						g);
		linearizeMargPrior(mMarg_order,
				           mMarg_H,
						   mMarg_b,
						   aom,
						   accum.getH(),      //out
						   accum.getB(),      //out
						   marg_prior_error); //out

        // Save marginalization prior
        if (mOut_marg_queue && !kfs_to_marg.empty())
		{
			MargData::Ptr m(new MargData);
			m->mAom = aom;
			m->mAbs_H = accum.getH();
			m->mAbs_b = accum.getB();
			m->mFrame_poses = mFrame_poses;
			m->mFrame_states = mFrame_states;
			m->mKfs_all = kf_ids_all;
			m->mKfs_to_marg = kfs_to_marg;
			m->mbUse_imu = true;

			for (int64_t t : m->mKfs_all)
   			    m->mOpt_flow_res.emplace_back(mPrev_opt_flow_res_test.at(t));

			mOut_marg_queue->push(m);
        }

		std::set<int> idx_to_keep, idx_to_marg;
		for (const auto& kv : aom.mAbs_order_map)
		{
		    if (kv.second.second == POSE_SIZE)
			{
				int start_idx = kv.second.first;
				if (poses_to_marg.count(kv.first) == 0)
				{
				    for (size_t i = 0; i < POSE_SIZE; i++)
				    	idx_to_keep.emplace(start_idx + i);
				}
				else
				{
				    for (size_t i = 0; i < POSE_SIZE; i++)
					    idx_to_marg.emplace(start_idx + i);
				}
		    }
			else
			{
//				FAIM_ASSERT(kv.second.second == POSE_VEL_BIAS_SIZE);
				// state
				int start_idx = kv.second.first;
				if (states_to_marg_all.count(kv.first) > 0)
				{
				    for (size_t i = 0; i < POSE_VEL_BIAS_SIZE; i++)
				    	idx_to_marg.emplace(start_idx + i);
				}
				else if (states_to_marg_vel_bias.count(kv.first) > 0)
				{
				    for (size_t i = 0; i < POSE_SIZE; i++)
				    	idx_to_keep.emplace(start_idx + i);
				    for (size_t i = POSE_SIZE; i < POSE_VEL_BIAS_SIZE; i++)
					    idx_to_marg.emplace(start_idx + i);
				}
				else
				{
//				    FAIM_ASSERT(kv.first == last_state_to_marg);
				    for (size_t i = 0; i < POSE_VEL_BIAS_SIZE; i++)
					    idx_to_keep.emplace(start_idx + i);
				}
		    }
		}

//		if (mConfig.vio_debug)
		{
		    std::cout << "keeping " << idx_to_keep.size()
					  << " marg " << idx_to_marg.size()
					  << " total " << asize << std::endl;
		    std::cout << "last_state_to_marg " << last_state_to_marg
			 	      << " mFrame_poses " << mFrame_poses.size()
					  << " mFrame_states " << mFrame_states.size() << std::endl;
		}

		Eigen::MatrixXd marg_H_new;
		Eigen::VectorXd marg_b_new;
		marginalizeHelper(accum.getH(), //in & out0
				          accum.getB(), //in & out0
						  idx_to_keep,  //in
						  idx_to_marg,  //in
						  marg_H_new,   //out
						  marg_b_new);  //out

		{
//			FAIM_ASSERT(mFrame_states.at(last_state_to_marg).isLinearized() == false);
		    mFrame_states.at(last_state_to_marg).setLinTrue();
		}

        for (const int64_t id : states_to_marg_all)
		{
            mFrame_states.erase(id);
            mImu_meas.erase(id);
            mPrev_opt_flow_res_test.erase(id);
        }

        for (const int64_t id : states_to_marg_vel_bias)
		{
            const PoseVelBiasStateWithLin& state = mFrame_states.at(id);
            PoseStateWithLin pose(state);

            mFrame_poses[id] = pose;
            mFrame_states.erase(id);
            mImu_meas.erase(id);
        }

        for (const int64_t id : poses_to_marg)
		{
            mFrame_poses.erase(id);
            mPrev_opt_flow_res_test.erase(id);
        }

        mLmdb.removeKeyframes(kfs_to_marg,
				              poses_to_marg,
							  states_to_marg_all);

        AbsOrderMap marg_order_new;

        for (const auto& kv : mFrame_poses)
		{
            marg_order_new.mAbs_order_map[kv.first] = std::make_pair(marg_order_new.miTotal_size, POSE_SIZE);

            marg_order_new.miTotal_size += POSE_SIZE;
            marg_order_new.miItems++;
        }

        marg_order_new.mAbs_order_map[last_state_to_marg] = std::make_pair(marg_order_new.miTotal_size, POSE_VEL_BIAS_SIZE);
        marg_order_new.miTotal_size += POSE_VEL_BIAS_SIZE;
        marg_order_new.miItems++;

		mMarg_H = marg_H_new;
		mMarg_b = marg_b_new;
		mMarg_order = marg_order_new;

//		FAIM_ASSERT(size_t(mMarg_H.cols()) == mMarg_order.miTotal_size);

		Eigen::VectorXd delta;
		computeDelta(mMarg_order, delta);//TODO don't care currently
		mMarg_b -= mMarg_H * delta;

//		if (mConfig.vio_debug)
	    {
            std::cout << "marginalizaon done!!" << std::endl;

            std::cout << "======== Marg nullspace ==========" << std::endl;
            checkMargNullspace();//TODO don't care currently
            std::cout << "=================================" << std::endl;
        }

        //    std::cout << "new marg prior order" << std::endl;
        //    mMarg_order.print_order();
    }//if (mFrame_poses.size() > mConfig.miVio_max_kfs || mFrame_states.size() >= mConfig.miVio_max_states)
}
/*
void Correlator::computeProjections(std::vector<Eigen::aligned_vector<Eigen::Vector4d>>& data) const
{
	for (const auto& kv : mLmdb.getObservations())
	{
		const TimeCamId& tcid_h = kv.first;

		for (const auto& obs_kv : kv.second)
		{
			const TimeCamId& tcid_t = obs_kv.first;

			if (tcid_t.miFrame_id != miLast_state_t_ns)
				continue;

			if (tcid_h != tcid_t)
			{
				PoseStateWithLin state_h = getPoseStateWithLin(tcid_h.miFrame_id);
				PoseStateWithLin state_t = getPoseStateWithLin(tcid_t.miFrame_id);

				Sophus::SE3d T_t_h_sophus =
					computeRelPose(state_h.getPose(), mCalib.mT_i_c[tcid_h.miCam_id],
								   state_t.getPose(), mCalib.mT_i_c[tcid_t.miCam_id]);

				Eigen::Matrix4d T_t_h = T_t_h_sophus.matrix();

				FrameRelLinData rld;

				std::visit(
					[&](const auto& cam)
					{
						for (size_t i = 0; i < obs_kv.second.size(); i++)
						{
							const KeypointObservation& kpt_obs = obs_kv.second[i];
							const KeypointPosition& kpt_pos = mLmdb.getLandmark(kpt_obs.kpt_id);

							Eigen::Vector2d res;
							Eigen::Vector4d proj;

							linearizePoint(kpt_obs,
									       kpt_pos,
										   T_t_h,
										   cam,
										   res,
										   nullptr,
										   nullptr,
										   &proj);

							proj[3] = kpt_obs.kpt_id;
							data[tcid_t.miCam_id].emplace_back(proj);
						}
					},
					mCalib.mIntrinsics[tcid_t.miCam_id].variant);

			} else {
				// target and host are the same
				// residual does not depend on the pose
				// it just depends on the point

				std::visit(
					[&](const auto& cam)
					{
						for (size_t i = 0; i < obs_kv.second.size(); i++)
						{
							const KeypointObservation& kpt_obs = obs_kv.second[i];
							const KeypointPosition& kpt_pos = mLmdb.getLandmark(kpt_obs.kpt_id);

							Eigen::Vector2d res;
							Eigen::Vector4d proj;

							linearizePoint(kpt_obs,
									       kpt_pos,
										   Eigen::Matrix4d::Identity(),
										   cam,
										   res,
										   nullptr,
										   nullptr,
										   &proj);

							proj[3] = kpt_obs.kpt_id;
							data[tcid_t.miCam_id].emplace_back(proj);
						}
					},
					mCalib.mIntrinsics[tcid_t.miCam_id].variant);
			}//for (const auto& obs_kv : kv.second) else
		}//for (const auto& obs_kv : kv.second)
	}//for (const auto& kv : mLmdb.getObservations())
}
*/
/////////////////// 604 end, started at line 491////////////////////////////
