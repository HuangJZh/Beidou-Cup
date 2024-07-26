





































#ifndef PREINTIGRATION_H_
#define PREINTIGRATION_H_

#include "config.h"//POSE_VEL_SIZE
#include "sdr_structs.h"//PoseVelState






class IntegratedImuMeasurement
{
public:
	using Ptr = std::shared_ptr<IntegratedImuMeasurement>;
	using MatN3 = Eigen::Matrix<double, POSE_VEL_SIZE, 3>;
	using MatN6 = Eigen::Matrix<double, POSE_VEL_SIZE, 6>;
  	using MatNN = Eigen::Matrix<double, POSE_VEL_SIZE, POSE_VEL_SIZE>;
	using VecN = Eigen::Matrix<double, POSE_VEL_SIZE, 1>;
	using Vec3 = Eigen::Matrix<double, 3, 1>;

	/// @brief Default constructor.
	IntegratedImuMeasurement() : miStart_t_ns(0), mbSqrt_cov_inv_computed(false)
	{
		mCov.setZero();
		mD_state_d_ba.setZero();
		mD_state_d_bg.setZero();//TODO: no derivative about scale factors?
		mBias_gyro_lin.setZero();
		mBias_accel_lin.setZero();//TODO: no scale factors?
	}
	
	/// @brief Constructor with start time and bias estimates.
	IntegratedImuMeasurement(      int64_t          start_t_ns,
			                 const Eigen::Vector3d& bias_gyro_lin,
							 const Eigen::Vector3d& bias_accel_lin)
		: miStart_t_ns(start_t_ns),
		  mbSqrt_cov_inv_computed(false),
		  mBias_gyro_lin(bias_gyro_lin),
		  mBias_accel_lin(bias_accel_lin)
	{
		mCov.setZero();
		mD_state_d_ba.setZero();
		mD_state_d_bg.setZero();
	}
	
	/// @brief Propagate current state given ImuMeasurement and optionally compute
	/// Jacobians.
	///
	/// @param[in] curr_state current state
	/// @param[in] data IMU data
	/// @param[out] next_state predicted state
	/// @param[out] d_next_d_curr Jacobian of the predicted state with respect
	/// to current state
	/// @param[out] d_next_d_accel Jacobian of the predicted state with respect
	/// accelerometer measurement
	/// @param[out] d_next_d_gyro Jacobian of the predicted state with respect
	/// gyroscope measurement
	inline static void propagateState(const PoseVelState& curr_state,
									  const ImuMeasurement& data,
									        PoseVelState& next_state,
									        MatNN*        d_next_d_curr = nullptr,
									        MatN3*        d_next_d_accel = nullptr,
									        MatN3*        d_next_d_gyro = nullptr)
	{
//		FAIM_ASSERT_STREAM(data.mvTimestamp > curr_state.miT_ns,
//			"data.mvTimestamp " << data.mvTimestamp << " curr_state.miT_ns " << curr_state.miT_ns);

		int64_t dt_ns = data.mvTimestamp - curr_state.miT_ns;
		double dt = dt_ns * 1e-9;

		if (data.mvAcc_gyr.tail<3>().array().isNaN()[0] || data.mvAcc_gyr.tail<3>().array().isNaN()[1] || data.mvAcc_gyr.tail<3>().array().isNaN()[2])
		{
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN meas->integrate(): IN propagateState(): HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH: SO3d::exp() #1"<<std::endl;//xin 20230930
		}
		Sophus::SO3d R_w_i_new_2 = curr_state.mT_w_i.so3() * Sophus::SO3d::exp(0.5 * dt * data.mvAcc_gyr.tail<3>());
		Eigen::Matrix3d RR_w_i_new_2 = R_w_i_new_2.matrix();

		Eigen::Vector3d accel_world = RR_w_i_new_2 * data.mvAcc_gyr.head<3>();

		next_state.miT_ns = data.mvTimestamp;
		if (data.mvAcc_gyr.tail<3>().array().isNaN()[0] || data.mvAcc_gyr.tail<3>().array().isNaN()[1] || data.mvAcc_gyr.tail<3>().array().isNaN()[2])
		{
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN meas->integrate(): IN propagateState(): HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH: SO3d::exp() #2"<<std::endl;//xin 20230930
		}
		next_state.mT_w_i.so3() = curr_state.mT_w_i.so3() * Sophus::SO3d::exp(dt * data.mvAcc_gyr.tail<3>());
		next_state.mVel_w_i = curr_state.mVel_w_i + accel_world * dt;
		next_state.mT_w_i.translation() = curr_state.mT_w_i.translation() +
										 curr_state.mVel_w_i * dt +
										 0.5 * accel_world * dt * dt;

//xin 20230803		std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN meas->integrate(): IN propagateState()"<<std::endl;
		if (d_next_d_curr)
		{
			d_next_d_curr->setIdentity();
			d_next_d_curr->block<3, 3>(0, 6).diagonal().setConstant(dt);
			d_next_d_curr->block<3, 3>(6, 3) = Sophus::SO3d::hat(-accel_world * dt);
			d_next_d_curr->block<3, 3>(0, 3) = d_next_d_curr->block<3, 3>(6, 3) * dt * 0.5;
		}

		if (d_next_d_accel)
		{
			d_next_d_accel->setZero();
			d_next_d_accel->block<3, 3>(0, 0) = 0.5 * RR_w_i_new_2 * dt * dt;
			d_next_d_accel->block<3, 3>(6, 0) = RR_w_i_new_2 * dt;
		}

		if (d_next_d_gyro)
		{
			d_next_d_gyro->setZero();

			Eigen::Matrix3d Jr;
			Sophus::rightJacobianSO3(dt * data.mvAcc_gyr.tail<3>(), Jr);

			Eigen::Matrix3d Jr2;
			Sophus::rightJacobianSO3(0.5 * dt * data.mvAcc_gyr.tail<3>(), Jr2);

			d_next_d_gyro->block<3, 3>(3, 0) = next_state.mT_w_i.so3().matrix() * Jr * dt;
			d_next_d_gyro->block<3, 3>(6, 0) = Sophus::SO3d::hat(-accel_world * dt) * RR_w_i_new_2 * Jr2 * 0.5 * dt;

			d_next_d_gyro->block<3, 3>(0, 0) = 0.5 * dt * d_next_d_gyro->block<3, 3>(6, 0);
		}
	}


	/// @brief Integrate IMU data
	///
	/// @param[in] data IMU data
	/// @param[in] accel_cov diagonal of accelerometer noise covariance matrix
	/// @param[in] gyro_cov diagonal of gyroscope noise covariance matrix
    void integrate(const ImuMeasurement&	data, 
			       const Vec3&				accel_cov,
                   const Vec3&				gyro_cov)
	{
		ImuMeasurement data_corrected = data;
		data_corrected.mvTimestamp -= miStart_t_ns;
		data_corrected.mvAcc_gyr.head<3>() -= mBias_accel_lin;
		data_corrected.mvAcc_gyr.tail<3>() -= mBias_gyro_lin;

		PoseVelState new_state;

		MatNN F;
		MatN3 A, G;

		propagateState(mDelta_state,
				       data_corrected,
					   new_state,
					   &F,
					   &A,
					   &G);

//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN meas->integrate(): AFTER propagateState()"<<std::endl;
		mDelta_state = new_state;
		mCov = F * mCov * F.transpose() + A * accel_cov.asDiagonal() * A.transpose() +
			   G * gyro_cov.asDiagonal() * G.transpose();
		mbSqrt_cov_inv_computed = false;

		mD_state_d_ba = -A + F * mD_state_d_ba;
		mD_state_d_bg = -G + F * mD_state_d_bg;
    }

	/// @brief Predict state given this pseudo-measurement
	///
	/// @param[in] state0 current state
	/// @param[in] g gravity vector
	/// @param[out] state1 predicted state
	void predictState(const PoseVelState&    state0,
			          const Eigen::Vector3d& g,
					        PoseVelState&    state1) const
	{
		double dt = mDelta_state.miT_ns * 1e-9;

		state1.mT_w_i.so3()         = state0.mT_w_i.so3() * mDelta_state.mT_w_i.so3();
		state1.mVel_w_i             = state0.mVel_w_i + g * dt + state0.mT_w_i.so3() * mDelta_state.mVel_w_i;
		state1.mT_w_i.translation() = state0.mT_w_i.translation() + state0.mVel_w_i * dt + 0.5 * g * dt * dt +
			                          state0.mT_w_i.so3() * mDelta_state.mT_w_i.translation();
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN IntegratedImuMeasurement::predictState()"<<std::endl;
	}

	/// @brief Time duration of preintegrated measurement in nanoseconds
	int64_t get_start_t_ns() const { return miStart_t_ns; }
	
	/// @brief Time duration of preintegrated measurement in nanoseconds
	int64_t get_dt_ns() const { return mDelta_state.miT_ns; }

	/// @brief Helper function to compute square root of the inverse covariance
	void compute_sqrt_cov_inv() const
	{
		mSqrt_cov_inv.setIdentity();
		auto ldlt = mCov.ldlt();

		mSqrt_cov_inv = ldlt.transpositionsP() * mSqrt_cov_inv;
		ldlt.matrixL().solveInPlace(mSqrt_cov_inv);

		VecN D_inv_sqrt;
		for (size_t i = 0; i < POSE_VEL_SIZE; i++)
		{
			if (ldlt.vectorD()[i] < std::numeric_limits<double>::min())
				D_inv_sqrt[i] = 0;
			else
				D_inv_sqrt[i] = 1.0 / sqrt(ldlt.vectorD()[i]);
		}
		mSqrt_cov_inv = D_inv_sqrt.asDiagonal() * mSqrt_cov_inv;
	}

	/// @brief Inverse of the measurement covariance matrix
	inline const MatNN get_cov_inv() const
	{
		if (!mbSqrt_cov_inv_computed)
		{
			compute_sqrt_cov_inv();
			mbSqrt_cov_inv_computed = true;
		}

		return mSqrt_cov_inv.transpose() * mSqrt_cov_inv;
	}
	
	/// @brief Compute residual between two states given this pseudo-measurement
	/// and optionally compute Jacobians.
	///
	/// @param[in] state0 initial state
	/// @param[in] g gravity vector
	/// @param[in] state1 next state
	/// @param[in] curr_bg current estimate of gyroscope bias
	/// @param[in] curr_ba current estimate of accelerometer bias
	/// @param[out] d_res_d_state0 if not nullptr, Jacobian of the residual with respect to state0
	/// @param[out] d_res_d_state1 if not nullptr, Jacobian of the residual with respect to state1
	/// @param[out] d_res_d_bg if not nullptr, Jacobian of the residual with respect to gyroscope bias
	/// @param[out] d_res_d_ba if not nullptr, Jacobian of the residual with respect to accelerometer bias
	/// @return residual
    VecN residual(const PoseVelState&    state0,
	  	          const Eigen::Vector3d& g,
                  const PoseVelState&    state1,
				  const Eigen::Vector3d& curr_bg,
                  const Eigen::Vector3d& curr_ba,
				        MatNN*           d_res_d_state0 = nullptr,
                        MatNN*           d_res_d_state1 = nullptr,
					    MatN3*           d_res_d_bg     = nullptr,
                        MatN3*           d_res_d_ba     = nullptr) const
	{
		double dt = mDelta_state.miT_ns * 1e-9;
		VecN res;

		VecN bg_diff, ba_diff;
		bg_diff = mD_state_d_bg * (curr_bg - mBias_gyro_lin);
		ba_diff = mD_state_d_ba * (curr_ba - mBias_accel_lin);

//		FAIM_ASSERT(ba_diff.segment<3>(3).isApproxToConstant(0));

		Eigen::Matrix3d R0_inv = state0.mT_w_i.so3().inverse().matrix();
		Eigen::Vector3d tmp = R0_inv * (state1.mT_w_i.translation() 
				                        - state0.mT_w_i.translation()
										- state0.mVel_w_i * dt 
										- 0.5 * g * dt * dt);

		res.segment<3>(0) = tmp - (mDelta_state.mT_w_i.translation()
				                   + bg_diff.segment<3>(0) 
								   + ba_diff.segment<3>(0));
		if (bg_diff.segment<3>(3).array().isNaN()[0] || bg_diff.segment<3>(3).array().isNaN()[1] || bg_diff.segment<3>(3).array().isNaN()[2])
		{
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN linearizeAbsIMU() : IN residual() HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH: SO3d::exp() #3"<<std::endl; //xin 20230930
		}
		res.segment<3>(3) = (Sophus::SO3d::exp(bg_diff.segment<3>(3))
				             * mDelta_state.mT_w_i.so3()
							 * state1.mT_w_i.so3().inverse()
							 * state0.mT_w_i.so3())
				            .log();

		Eigen::Vector3d tmp2 = R0_inv * (state1.mVel_w_i 
				                         - state0.mVel_w_i 
										 - g * dt);
		res.segment<3>(6) = tmp2 - (mDelta_state.mVel_w_i 
				                    + bg_diff.segment<3>(6)
									+ ba_diff.segment<3>(6));

		if (d_res_d_state0 || d_res_d_state1)
		{
		    Eigen::Matrix3d J;
		    Sophus::rightJacobianInvSO3(res.segment<3>(3), J);

		    if (d_res_d_state0)
			{
				d_res_d_state0->setZero();
				d_res_d_state0->block<3, 3>(0, 0) = -R0_inv;
				d_res_d_state0->block<3, 3>(0, 3) = Sophus::SO3d::hat(tmp) * R0_inv;
				d_res_d_state0->block<3, 3>(3, 3) = J * R0_inv;
				d_res_d_state0->block<3, 3>(6, 3) = Sophus::SO3d::hat(tmp2) * R0_inv;

				d_res_d_state0->block<3, 3>(0, 6) = -R0_inv * dt;
				d_res_d_state0->block<3, 3>(6, 6) = -R0_inv;
		    }

		    if (d_res_d_state1)
			{
				d_res_d_state1->setZero();
				d_res_d_state1->block<3, 3>(0, 0) = R0_inv;
				d_res_d_state1->block<3, 3>(3, 3) = -J * R0_inv;

				d_res_d_state1->block<3, 3>(6, 6) = R0_inv;
		    }
		}

		if (d_res_d_ba)
		    *d_res_d_ba = -mD_state_d_ba;

		if (d_res_d_bg)
		{
		    d_res_d_bg->setZero();
		    *d_res_d_bg = -mD_state_d_bg;

		    Eigen::Matrix3d J;
		    Sophus::leftJacobianInvSO3(res.segment<3>(3),
										J);
		    d_res_d_bg->block<3, 3>(3, 0) = J * mD_state_d_bg.block<3, 3>(3, 0);
		}

//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN linearizeAbsIMU() : IN residual()"<<std::endl;
		return res;
    }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
			int64_t			miStart_t_ns; ///< Integration start time in nanoseconds
	mutable bool		 	mbSqrt_cov_inv_computed; ///< If the cached square root inverse convariance is computed
			MatN3			mD_state_d_ba, mD_state_d_bg;
			Eigen::Vector3d mBias_gyro_lin, mBias_accel_lin;
			PoseVelState	mDelta_state; ///< Delta state 
			MatNN			mCov;
	mutable MatNN			mSqrt_cov_inv;///< Cached square root inverse of measurement covariance
};

#endif /*# PREINTIGRATION_H_*/
