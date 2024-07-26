
























/*----------------------------------------------------------------------------------------------*/

#ifndef CORRELATOR_H_
#define CORRELATOR_H_

#include "includes.h"
//#include "sv_select.h"
//#include "channel.h"
#include "fifo_cam.h"
#include "accessories/PinholeCamera.h"
#include "fifo_imu.h"
#include "fifo_fea.h"
#include "signaldef.h"//g//604
#include "accessories/preintegration.h" //IntegratedImuMeasurement
#include "accessories/calibration.h" //Calibration
#include "accessories/cameras.h" //StereographicParam
#include <tbb/blocked_range.h> //tbb::blocked_range, tbb::split in .h
#include <tbb/concurrent_queue.h> //tbb::concurrent_bounded_queue in .h
#include <tbb/parallel_for.h> //tbb::parallel_for in .cpp
#include <tbb/parallel_reduce.h> //tbb::parallel_reduce in .cpp
#include <atomic>//std::atomic
#include <unordered_set>//std::unordered_set in .cpp
#include <chrono>//std::chrono::high_resolution_clock::now()
#include "accessories/accumulator.h"//DenseAccumulator

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <type_traits>
#include <cereal/types/vector.hpp>// for nested cereal aligned_vector<DoubleSphereCamera>, aligned_vector<SE3d>@mIntrinsics, aligned_vector<Vector2i>@mResolution
/*! \ingroup CLASSES
 *
 */
class Correlator : public Threaded_Object
{

	private:

		/* Default object variables */





		Preamble_2_PVT_S	preamble;

		/* These variables are shared among all the channels */
		Acq_Command_S 		result; 							//!< An acquisition result has been returned!
		ms_packet_cam		packet_cam;							//!< Count 1ms packets
		ms_packet_imu		packet_imu;							//!< Count 1ms packets
		ms_packet_fea		packet_fea;							//!< Count 1ms packets
		int32				packet_count_fea;					//!< Count 1ms packets
		int32				measurement_tic;					//!< Measurement tic

		std::ofstream		mosTraj_vio_csv;
		bool				mbIs_header_written;

	public:

		Correlator();
		~Correlator();
		void Import();											//!< Get IF data, NCO commands, and acq results
		void Export();											//!< Dump results to channels and Navigation
		void Start();											//!< Start the thread
		void Correlate();										//!< Run the actual correlation






		void TakeMeasurements();																//!< Take some measurements
		//VIO
		Calibration<double>										mCalib;
		bool													mbInitialized;
		Sophus::SE3d											mT_w_i_init;
		int64_t													miLast_state_t_ns;
		Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>	mFrame_states;
		Eigen::aligned_map<int64_t, PoseStateWithLin>			mFrame_poses;
		Eigen::aligned_map<int64_t, IntegratedImuMeasurement>	mImu_meas;
		//Input
		Eigen::aligned_map<int64_t, OpticalFlowResultReduced::Ptr>		mPrev_opt_flow_res_test;
		//Point management//from ba_base.h
		LandmarkDatabase										mLmdb;
		
		bool													mbTake_kf;
		int														miFrames_after_kf;	
		Vioconfig												mConfig;
		std::set<int64_t>										mKf_ids;
		std::map<int64_t, int>									mNum_points_kf;
		std::atomic<int64_t>									mLast_processed_t_ns;

		bool													mbOpt_started;
		double													mdHuber_thresh;// Point management
		Eigen::Vector3d											mGyro_bias_weight, mAccel_bias_weight;
		double													mdLambda, mdMin_lambda, mdMax_lambda, mdLambda_vee;
		// Marginalization
		AbsOrderMap												mMarg_order;
		Eigen::MatrixXd											mMarg_H;
		Eigen::VectorXd											mMarg_b;

		// Point management
		double													mdObs_std_dev;
//		tbb::concurrent_bounded_queue<FeaMeasurement::Ptr>		mVision_data_queue;
		tbb::concurrent_bounded_queue<ImuMeasurement::Ptr>		mImu_data_queue;
//		tbb::concurrent_bounded_queue<PoseVelBiasState::Ptr>*	mOut_state_queue = nullptr;
		tbb::concurrent_bounded_queue<MargData::Ptr>*			mOut_marg_queue = nullptr;

		Eigen::Vector3d											mvBg;
		Eigen::Vector3d											mvBa;
//		FeaMeasurement::Ptr										mpPrev_frame, curr_frame;
		void getMeasurements();
		bool measure(const FeaMeasurement::Ptr&        	  opt_flow_meas,
                     const IntegratedImuMeasurement::Ptr& meas);

		void checkMargNullspace() const;
		void marginalize(const std::map<int64_t, int>& num_points_connected);
		void optimize();
		void computeProjections(std::vector<Eigen::aligned_vector<Eigen::Vector4d>>& data) const;

		// from ba_base.h
		/// Triangulates the point and returns homogeneous representation.
		/// First 3 components - unit-length direction vector.
		/// Last componenet inverse distance.
		template <class Derived>
		static Eigen::Matrix<typename Derived::Scalar, 4, 1> triangulate(const Eigen::MatrixBase<Derived>&            f0,
																		 const Eigen::MatrixBase<Derived>&            f1,
																		 const Sophus::SE3<typename Derived::Scalar>& T_0_1)
		{
//			EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);//TODO: EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE

			using Scalar = typename Derived::Scalar;
			using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

			Eigen::Matrix<Scalar, 3, 4> P1, P2;
			P1.setIdentity();
			P2 = T_0_1.inverse().matrix3x4();

			Eigen::Matrix<Scalar, 4, 4> A(4, 4);
			A.row(0) = f0[0] * P1.row(2) - f0[2] * P1.row(0);
			A.row(1) = f0[1] * P1.row(2) - f0[2] * P1.row(1);
			A.row(2) = f1[0] * P2.row(2) - f1[2] * P2.row(0);
			A.row(3) = f1[1] * P2.row(2) - f1[2] * P2.row(1);

			Eigen::JacobiSVD<Eigen::Matrix<Scalar, 4, 4>> mySVD(A, Eigen::ComputeFullV);
			Vec4 worldPoint = mySVD.matrixV().col(3);
			worldPoint /= worldPoint.template head<3>().norm();

			// Enforce same direction of bearing vector and initial point
			if (f0.dot(worldPoint.template head<3>()) < 0)
				worldPoint *= -1;

//			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN Correlator::triangulate()"<<std::endl;
			return worldPoint;
		}

		/************** ba_base.h ************/
		/* ver. '7 params' */
		template <class CamT>
		static bool linearizePoint(const KeypointObservation&                 kpt_obs,
								   const KeypointPosition&                    kpt_pos,
								   const Eigen::Matrix4d&                     T_t_h,
								   const CamT&                                cam,
										 Eigen::Vector2d&                     res,
										 Eigen::Matrix<double, 2, POSE_SIZE>* d_res_d_xi = nullptr,
										 Eigen::Matrix<double, 2, 3>*         d_res_d_p  = nullptr,
										 Eigen::Vector4d*                     proj       = nullptr)
		{
			Eigen::Matrix<double, 4, 2> Jup;
			Eigen::Vector4d p_h_3d;
			p_h_3d = StereographicParam<double>::unproject(kpt_pos.mDir,
														   &Jup);
			p_h_3d[3] = kpt_pos.mdId;
//xin 20230803		std::cout<<"linearizePoint[7 params]():kpt_pos.mDir  = ["<<kpt_pos.mDir[0]<<", "<<kpt_pos.mDir[1]<<"], kpt_pos.mdId  = "<<kpt_pos.mdId<<std::endl;
//xin 20230803		std::cout<<"linearizePoint[7 params]():p_h_3d        = ["<<p_h_3d[0]<<", "<<p_h_3d[1]<<", "<<p_h_3d[2]<<", "<<p_h_3d[3]<<"]"<<std::endl;

			Eigen::Vector4d p_t_3d = T_t_h * p_h_3d;
//			std::cout<<"linearizePoint[7 params](): T_t_h: "<<std::endl<<T_t_h<<std::endl;


			Eigen::Matrix<double, 4, POSE_SIZE> d_point_d_xi;
			d_point_d_xi.topLeftCorner<3, 3>()  = Eigen::Matrix3d::Identity() * kpt_pos.mdId;
			d_point_d_xi.topRightCorner<3, 3>() = -Sophus::SO3d::hat(p_t_3d.head<3>());
			d_point_d_xi.row(3).setZero();

			Eigen::Matrix<double, 2, 4> Jp;
			bool valid = cam.project(p_t_3d,
									 res,
									 &Jp);
//xin 20230803		std::cout<<"linearizePoint[7 params](): cam.project  = ("<<res[0]<<", "<<res[1]<<")"<<std::endl;
			valid &= res.array().isFinite().all();

			if (!valid)
			{
				std::cerr	<< " Invalid projection! kpt_pos.dir " << kpt_pos.mDir.transpose()
							<< " kpt_pos.id " << kpt_pos.mdId
							<< " idx " << kpt_obs.miKpt_id << std::endl;

				std::cerr << "T_t_h\n" << T_t_h << std::endl;
				std::cerr << "p_h_3d\n" << p_h_3d.transpose() << std::endl;
				std::cerr << "p_t_3d\n" << p_t_3d.transpose() << std::endl;

				return false;
			}

			if (proj)
			{
				proj->head<2>() = res;
				(*proj)[2] = p_t_3d[3] / p_t_3d.head<3>().norm();
			}
//xin 20230803		std::cout<<"linearizePoint[7 params](): kpt_obs.mPos = ("<<kpt_obs.mPos[0]<<", "<<kpt_obs.mPos[1]<<")"<<std::endl;
			res -= kpt_obs.mPos;

			if (d_res_d_xi)
			  *d_res_d_xi = Jp * d_point_d_xi;

			if (d_res_d_p)
			{
				Eigen::Matrix<double, 4, 3> Jpp;
				Jpp.setZero();
				Jpp.block<3, 2>(0, 0) = T_t_h.topLeftCorner<3, 4>() * Jup;
				Jpp.col(2) = T_t_h.col(3);

				*d_res_d_p = Jp * Jpp;
			}

			return true;
		}

		/* ver. '5 params' */
		template <class CamT>
		inline static bool linearizePoint(const KeypointObservation&         kpt_obs,
										  const KeypointPosition&            kpt_pos,
										  const CamT&                        cam,
												Eigen::Vector2d&             res,
												Eigen::Matrix<double, 2, 3>* d_res_d_p = nullptr,
												Eigen::Vector4d*             proj      = nullptr)
		{
			// Todo implement without jacobians
			Eigen::Matrix<double, 4, 2> Jup;
			Eigen::Vector4d p_h_3d;
			p_h_3d = StereographicParam<double>::unproject(kpt_pos.mDir,
														   &Jup);
//xin 20230803		std::cout<<"linearizePoint[5 params]():kpt_pos.mDir  = ["<<kpt_pos.mDir[0]<<", "<<kpt_pos.mDir[1]<<"]"<<std::endl;
//xin 20230803		std::cout<<"linearizePoint[5 params]():p_h_3d        = ["<<p_h_3d[0]<<", "<<p_h_3d[1]<<", "<<p_h_3d[2]<<", "<<p_h_3d[3]<<"]"<<std::endl;

			Eigen::Matrix<double, 2, 4> Jp;
			bool valid = cam.project(p_h_3d,
									 res,
									 &Jp);
//xin 20230803		std::cout<<"linearizePoint[5 params](): cam.project  = ("<<res[0]<<", "<<res[1]<<")"<<std::endl;
			valid &= res.array().isFinite().all();
			valid &= res.array().isFinite().all();

			if (!valid)
			{
				std::cerr	<< " Invalid projection! kpt_pos.dir " << kpt_pos.mDir.transpose()
							<< " kpt_pos.id " << kpt_pos.mdId
							<< " idx " << kpt_obs.miKpt_id << std::endl;
				
				std::cerr << "p_h_3d\n" << p_h_3d.transpose() << std::endl;

				return false;
			}

			if (proj)
			{
				proj->head<2>() = res;
				(*proj)[2] = kpt_pos.mdId;
			}
//xin 20230803		std::cout<<"linearizePoint[5 params](): kpt_obs.mPos = ("<<kpt_obs.mPos[0]<<", "<<kpt_obs.mPos[1]<<")"<<std::endl;
			res -= kpt_obs.mPos;

			if (d_res_d_p)
			{
				Eigen::Matrix<double, 4, 3> Jpp;
				Jpp.setZero();
				Jpp.block<4, 2>(0, 0) = Jup;
				Jpp.col(2).setZero();

				*d_res_d_p = Jp * Jpp;
			}

			return true;
		}


		void linearizeHelper(      Eigen::aligned_vector<RelLinData>&                                                  rld_vec,
                         	 const Eigen::aligned_map<TimeCamId,
                                                  	  Eigen::aligned_map<TimeCamId,
                                                                     	 Eigen::aligned_vector<KeypointObservation>>>& obs_to_lin,
                               	   double&                                                                             error) const;

		/******************** keypoint_vio_linearize.cpp ********************/
		void linearizeAbsIMU(const AbsOrderMap&                                           aom,               //IN
								   Eigen::MatrixXd&                                       abs_H,             //OUT
								   Eigen::VectorXd&                                       abs_b,             //OUT
								   double&                                                imu_error,         //OUT
								   double&                                                bg_error,          //OUT
								   double&                                                ba_error,          //OUT
							 const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>&  states,            //IN
							 const Eigen::aligned_map<int64_t, IntegratedImuMeasurement>& imu_meas,          //IN
							 const Eigen::Vector3d&                                       gyro_bias_weight,  //IN
							 const Eigen::Vector3d&                                       accel_bias_weight, //IN
							 const Eigen::Vector3d&                                       g);                //IN

		/*********************** ba_base.h *************************/
		void computeDelta(const AbsOrderMap&     marg_order,
                                Eigen::VectorXd& delta) const;
		void linearizeMargPrior(const AbsOrderMap&     marg_order,              //IN
                                const Eigen::MatrixXd& marg_H,                  //IN
                                const Eigen::VectorXd& marg_b,                  //IN
                                const AbsOrderMap&     aom,                     //IN
                                      Eigen::MatrixXd& abs_H,                   //OUT
                                      Eigen::VectorXd& abs_b,                   //OUT
                                      double&          marg_prior_error) const; //OUT
		void updatePoints(const AbsOrderMap&     aom,
						  const RelLinData&      rld,
						  const Eigen::VectorXd& inc);
		void computeError(double&                                                   error,
						  std::map<int, std::vector<std::pair<TimeCamId, double>>>* outliers          = nullptr,
						  double                                                    outlier_threshold = 0) const;
		
		PoseStateWithLin getPoseStateWithLin(int64_t t_ns) const
		{
			auto it = mFrame_poses.find(t_ns);
			if (it != mFrame_poses.end())
				return it->second;

			auto it2 = mFrame_states.find(t_ns);
			if (it2 == mFrame_states.end())
			{
				std::cerr << "Could not find pose " << t_ns << std::endl;
				std::abort();
			}

			return PoseStateWithLin(it2->second);
		}
		
		/****************** keypoint_vio_linearize.cpp  ********************/
		void computeImuError(const AbsOrderMap&												aom,
								   double&													imu_error,
								   double&													bg_error,
    							   double&													ba_error,
   							 const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>&	states,
    						 const Eigen::aligned_map<int64_t, IntegratedImuMeasurement>&	imu_meas,
							 const Eigen::Vector3d&											gyro_bias_weight,
							 const Eigen::Vector3d&											accel_bias_weight,
							 const Eigen::Vector3d&											g);

		/*********************** ba_base.cpp *************************/
		void computeMargPriorError(const AbsOrderMap&     marg_order,
		                           const Eigen::MatrixXd& marg_H,
                                   const Eigen::VectorXd& marg_b,
										 double&          marg_prior_error) const;

		static Eigen::VectorXd checkNullspace(const Eigen::MatrixXd&                                      H,
		                               const Eigen::VectorXd&                                      b,
                                       const AbsOrderMap&                                          order,
                                       const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>& frame_states,
                                       const Eigen::aligned_map<int64_t, PoseStateWithLin>&        frame_poses);

		static void linearizeRel(const RelLinData&       rld,  //IN
                                	   Eigen::MatrixXd&  H,    //OUT
                                	   Eigen::VectorXd&  b)	//OUT
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
//xin 20230803		std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN LinearizeAbsReduce: IN linearizeRel()"<<std::endl;
		}

		/*********************** ba_base.h *************************/
		template <class AccumT>
		static void linearizeAbs(const Eigen::MatrixXd& rel_H,
								 const Eigen::VectorXd& rel_b,
								 const RelLinDataBase&  rld,
								 const AbsOrderMap&     aom,
									   AccumT&          accum)
		{
			for (size_t i = 0; i < rld.order.size(); i++)
			{
				const TimeCamId& tcid_h  = rld.order[i].first;
				const TimeCamId& tcid_ti = rld.order[i].second;

				int abs_h_idx  = aom.mAbs_order_map.at(tcid_h.miFrame_id).first;
				int abs_ti_idx = aom.mAbs_order_map.at(tcid_ti.miFrame_id).first;

				accum.template addB<POSE_SIZE>(abs_h_idx, 
											   rld.d_rel_d_h[i].transpose() * rel_b.segment<POSE_SIZE>(i * POSE_SIZE));
				accum.template addB<POSE_SIZE>(abs_ti_idx,
											   rld.d_rel_d_t[i].transpose() * rel_b.segment<POSE_SIZE>(i * POSE_SIZE));

				for (size_t j = 0; j < rld.order.size(); j++)
				{
//					FAIM_ASSERT(rld.order[i].first == rld.order[j].first);

					const TimeCamId& tcid_tj = rld.order[j].second;

					int abs_tj_idx = aom.mAbs_order_map.at(tcid_tj.miFrame_id).first;

					if (tcid_h.miFrame_id == tcid_ti.miFrame_id ||
						tcid_h.miFrame_id == tcid_tj.miFrame_id)
						continue;

					accum.template addH<POSE_SIZE, POSE_SIZE>(abs_h_idx,
															  abs_h_idx,
															  rld.d_rel_d_h[i].transpose() *
																							 rel_H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * j) * 
																																							   rld.d_rel_d_h[j]);

					accum.template addH<POSE_SIZE, POSE_SIZE>(abs_ti_idx,
															  abs_h_idx,
															  rld.d_rel_d_t[i].transpose() *
																							 rel_H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * j) *
																																							   rld.d_rel_d_h[j]);

					accum.template addH<POSE_SIZE, POSE_SIZE>(abs_h_idx,
															  abs_tj_idx,
															  rld.d_rel_d_h[i].transpose() *
																							 rel_H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * j) *
																																							   rld.d_rel_d_t[j]);

					accum.template addH<POSE_SIZE, POSE_SIZE>(abs_ti_idx,
															  abs_tj_idx,
															  rld.d_rel_d_t[i].transpose() *
																							 rel_H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * j) *
																																							   rld.d_rel_d_t[j]);
				}//for (size_t j = 0; j < rld.order.size(); j++)
			}//for (size_t i = 0; i < rld.order.size(); i++)
//xin 20230803		std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN LinearizeAbsReduce: IN linearizeAbs()"<<std::endl;
		}

		/*********************** ba_base.cpp *************************/
		void marginalizeHelper(      Eigen::MatrixXd&     abs_H,        //in & out
                                     Eigen::VectorXd&     abs_b,        //in & out
                               const std::set<int>&       idx_to_keep,  //in
                               const std::set<int>&       idx_to_marg,  //in
                                     Eigen::MatrixXd&     marg_H,       //out
                                     Eigen::VectorXd&     marg_b);      //out


		template <class AccumT>
		struct LinearizeAbsReduce
		{
			using RelLinDataIter = Eigen::aligned_vector<RelLinData>::iterator;

			LinearizeAbsReduce(AbsOrderMap& aom) : aom(aom)
			{
				mAccum.reset(aom.miTotal_size);
			}

			LinearizeAbsReduce(const LinearizeAbsReduce& other, tbb::split) : aom(other.aom)
			{
				mAccum.reset(aom.miTotal_size);
			}

			void operator()(const tbb::blocked_range<RelLinDataIter>& range)
			{
//xin 20230803			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN LinearizeAbsReduce : IN operator()"<<std::endl;
				for (RelLinData& rld : range)
				{
					rld.invert_keypoint_hessians();

					Eigen::MatrixXd rel_H;
					Eigen::VectorXd rel_b;
					linearizeRel(rld,   //IN
								 rel_H, //OUT
								 rel_b);//OUT
					linearizeAbs(rel_H,  //IN
								 rel_b,  //IN
								 rld,    //IN
								 aom,    //IN
								 mAccum);//OUT
				}
			}

			void join(LinearizeAbsReduce& rhs) { mAccum.join(rhs.mAccum); }

			AbsOrderMap& aom;
			AccumT mAccum;
		};//struct LinearizeAbsReduce


		static Sophus::SE3d computeRelPose(const Sophus::SE3d&     T_w_i_h,
										   const Sophus::SE3d&     T_i_c_h,
										   const Sophus::SE3d&     T_w_i_t,
										   const Sophus::SE3d&     T_i_c_t,
										  		 Sophus::Matrix6d* d_rel_d_h = nullptr,
									      		 Sophus::Matrix6d* d_rel_d_t = nullptr);


		/************ ba_base.h ************/
		inline void backup();
		inline void restore();
		void filterOutliers(double outlier_threshold,
			            	int    min_num_obs);



		/*************** vio.cpp ****************/
		void load_data(const std::string& calib_path);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* CORRELATOR_H_ */
