






































#ifndef CALIBRATION_H_
#define CALIBRATION_H_

//#include "cameras.h" //DoubleSphereCamera


/// @brief Static calibration for accelerometer.
///
/// Calibrates axis scaling and misalignment and has 9 parameters \f$ [b_x,
/// b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6]^T \f$.
/// \f[
/// a_c = \begin{bmatrix} s_1 + 1 & 0 & 0 \\ s_2 & s_4 + 1 & 0 \\ s_3 & s_5 &
/// s_6 + 1 \\  \end{bmatrix} a_r -  \begin{bmatrix} b_x \\ b_y \\ b_z
/// \end{bmatrix}
/// \f] where  \f$ a_c \f$ is a calibrated measurement and \f$ a_r \f$ is a
/// raw measurement. When all elements are zero applying calibration results in
/// Identity mapping.
template <typename Scalar>
class CalibAccelBias
{
public:
	using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
	using Mat33 = Eigen::Matrix<Scalar, 3, 3>;

	/// @brief Default constructor with zero initialization.
	inline CalibAccelBias()
	{
		accel_bias_full.setZero();
	}

/*	/// @brief  Set calibration to random values (used in unit-tests).
	inline void setRandom()
	{
		accel_bias_full.setRandom();
		accel_bias_full.template head<3>() /= 10;
		accel_bias_full.template tail<6>() /= 100;
	}
*/
	/// @brief Return const vector of parameters.
	/// See detailed description in \ref CalibAccelBias.
	inline const Eigen::Matrix<Scalar, 9, 1>& getParam() const
	{
		return accel_bias_full;
	}

	/// @brief Return vector of parameters. See detailed description in \ref
	/// CalibAccelBias.
	inline Eigen::Matrix<Scalar, 9, 1>& getParam()
	{
		return accel_bias_full;
	}

	/// @brief Increment the calibration vector
	///
	/// @param inc increment vector
	inline void operator+=(const Eigen::Matrix<Scalar, 9, 1>& inc)
	{
		accel_bias_full += inc;
	}

	/// @brief Return bias vector and scale matrix. See detailed description in
	/// \ref CalibAccelBias.
	inline void getBiasAndScale(Vec3& accel_bias, Mat33& accel_scale) const
	{
		accel_bias = accel_bias_full.template head<3>();

		accel_scale.setZero();
		accel_scale.col(0) = accel_bias_full.template segment<3>(3);
		accel_scale(1, 1) = accel_bias_full(6);
		accel_scale(2, 1) = accel_bias_full(7);
		accel_scale(2, 2) = accel_bias_full(8);
	}

	/// @brief Calibrate the measurement. See detailed description in
	/// \ref CalibAccelBias.
	///
	/// @param raw_measurement
	/// @return calibrated measurement
	inline Vec3 getCalibrated(const Vec3& raw_measurement) const
	{
		Vec3 accel_bias;
		Mat33 accel_scale;

		getBiasAndScale(accel_bias, accel_scale);

		return (raw_measurement + accel_scale * raw_measurement - accel_bias);
	}
/*
  /// @brief Invert calibration (used in unit-tests).
  ///
  /// @param calibrated_measurement
  /// @return raw measurement
  inline Vec3 invertCalibration(const Vec3& calibrated_measurement) const 
  {
    Vec3 accel_bias;
    Mat33 accel_scale;

    getBiasAndScale(accel_bias, accel_scale);

    Mat33 accel_scale_inv =
        (Eigen::Matrix3d::Identity() + accel_scale).inverse();

    return accel_scale_inv * (calibrated_measurement + accel_bias);
  }
*/
private:
	Eigen::Matrix<Scalar, 9, 1> accel_bias_full;
};

/// @brief Static calibration for gyroscope.
///
/// Calibrates rotation, axis scaling and misalignment and has 12 parameters \f$
/// [b_x, b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6, s_7, s_8, s_9]^T \f$. \f[
/// \omega_c = \begin{bmatrix} s_1 + 1 & s_4 & s_7 \\ s_2 & s_5 + 1 & s_8 \\ s_3
/// & s_6 & s_9 +1 \\  \end{bmatrix} \omega_r -  \begin{bmatrix} b_x \\ b_y
/// \\ b_z \end{bmatrix} \f] where  \f$ \omega_c \f$ is a calibrated measurement
/// and \f$ \omega_r \f$ is a raw measurement. When all elements are zero
/// applying calibration results in Identity mapping.
template <typename Scalar>
class CalibGyroBias
{
public:
	using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
	using Mat33 = Eigen::Matrix<Scalar, 3, 3>;

	/// @brief Default constructor with zero initialization.
	inline CalibGyroBias()
	{
		gyro_bias_full.setZero();
	}
/*
	/// @brief  Set calibration to random values (used in unit-tests).
	inline void setRandom()
	{
		gyro_bias_full.setRandom();
		gyro_bias_full.template head<3>() /= 10;
		gyro_bias_full.template tail<9>() /= 100;
	}
*/
	/// @brief Return const vector of parameters.
	/// See detailed description in \ref CalibGyroBias.
	inline const Eigen::Matrix<Scalar, 12, 1>& getParam() const
	{
		return gyro_bias_full;
	}

	/// @brief Return vector of parameters.
	/// See detailed description in \ref CalibGyroBias.
	inline Eigen::Matrix<Scalar, 12, 1>& getParam()
	{
		return gyro_bias_full;
	}

	/// @brief Increment the calibration vector
	///
	/// @param inc increment vector
	inline void operator+=(const Eigen::Matrix<Scalar, 12, 1>& inc)
	{
		gyro_bias_full += inc;
	}

	/// @brief Return bias vector and scale matrix. See detailed description in
	/// \ref CalibGyroBias.
	inline void getBiasAndScale(Vec3& gyro_bias, Mat33& gyro_scale) const
	{
		gyro_bias = gyro_bias_full.template head<3>();
		gyro_scale.col(0) = gyro_bias_full.template segment<3>(3);
		gyro_scale.col(1) = gyro_bias_full.template segment<3>(6);
		gyro_scale.col(2) = gyro_bias_full.template segment<3>(9);
	}

	/// @brief Calibrate the measurement. See detailed description in
	/// \ref CalibGyroBias.
	///
	/// @param raw_measurement
	/// @return calibrated measurement
	inline Vec3 getCalibrated(const Vec3& raw_measurement) const
	{
		Vec3 gyro_bias;
		Mat33 gyro_scale;

		getBiasAndScale(gyro_bias, gyro_scale);

		return (raw_measurement + gyro_scale * raw_measurement - gyro_bias);
	}
/*
	/// @brief Invert calibration (used in unit-tests).
	///
	/// @param calibrated_measurement
	/// @return raw measurement
	inline Vec3 invertCalibration(const Vec3& calibrated_measurement) const
	{
		Vec3 gyro_bias;
		Mat33 gyro_scale;

		getBiasAndScale(gyro_bias, gyro_scale);

		Mat33 gyro_scale_inv = (Eigen::Matrix3d::Identity() + gyro_scale).inverse();

		return gyro_scale_inv * (calibrated_measurement + gyro_bias);
	}
*/
private:
	Eigen::Matrix<Scalar, 12, 1> gyro_bias_full;
};


/// @brief Struct to store camera-IMU calibration
template <class Scalar>
struct Calibration
{
	using Ptr = std::shared_ptr<Calibration>;
	using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
	using SE3 = Sophus::SE3<Scalar>;


	/// @brief Default constructor.
	Calibration()
	{
		miCam_time_offset_ns = 0;

		mImu_update_rate = 200;//xin: from Kimera-VIO

		// reasonable defaults//xin: from VINS-Mono
		mAccel_noise_std.setConstant(0.08);
		mGyro_noise_std.setConstant(0.004);
		mAccel_bias_std.setConstant(0.00004);
		mGyro_bias_std.setConstant(2.0e-6);
	}

	/// @brief Continuous time accelerometer noise standard deviation.
	Vec3 mAccel_noise_std;
	
	/// @brief Continuous time gyroscope noise standard deviation.
	Vec3 mGyro_noise_std;

	/// @brief Continuous time accelerometer bias random walk standard deviation.
	Vec3 mAccel_bias_std;

	/// @brief Continuous time gyroscope bias random walk standard deviation.
	Vec3 mGyro_bias_std;
	
	/// @brief Dicrete time gyroscope noise standard deviation.
	///
	/// \f$ \sigma_d = \sigma_c \sqrt{r} \f$, where \f$ r \f$ is IMU update
	/// rate.
	inline Vec3 dicrete_time_gyro_noise_std() const
	{
		return mGyro_noise_std * std::sqrt(mImu_update_rate);
	}

	/// @brief Dicrete time accelerometer noise standard deviation.
	///
	/// \f$ \sigma_d = \sigma_c \sqrt{r} \f$, where \f$ r \f$ is IMU update
	/// rate.
	inline Vec3 dicrete_time_accel_noise_std() const
	{
		return mAccel_noise_std * std::sqrt(mImu_update_rate);
	}

	/// @brief Time offset between cameras and IMU in nanoseconds.
	///
	/// With raw image timestamp \f$ t_r \f$ and this offset \f$ o \f$ we cam get
	/// a timestamp aligned with IMU clock as \f$ t_c = t_r + o \f$.
	int64_t miCam_time_offset_ns;
	
	/// @brief Static accelerometer bias from calibration.
	CalibAccelBias<Scalar> mCalib_accel_bias;

	/// @brief Static gyroscope bias from calibration.
	CalibGyroBias<Scalar> mCalib_gyro_bias;
	
	/// @brief IMU update rate.
	Scalar mImu_update_rate;
	
	/// @brief Vector of camera intrinsics. Can store different camera models. See
	/// \ref GenericCamera
//	Eigen::aligned_vector<GenericCamera<Scalar>> mIntrinsics;//xin 2023-6-11
	Eigen::aligned_vector<DoubleSphereCamera<Scalar>> mIntrinsics;

	/// @brief Vector of transformations from camera to IMU
	///
	/// Point in camera coordinate frame \f$ p_c \f$ can be tranformed to the
	/// point in IMU coordinate frame as \f$ p_i = T_{ic} p_c, T_{ic} \in
	/// SE(3)\f$
	Eigen::aligned_vector<SE3> mT_i_c;

	/// @brief Camera resolutions.
	Eigen::aligned_vector<Eigen::Vector2i> mResolution;
	
	/// @brief Vector of splines representing radially symmetric vignetting for
	/// each of the camera.
	///
	/// Splines use time in nanoseconds for evaluation, but in this case we use
	/// distance from the optical center in pixels multiplied by 1e9 as a "time"
	/// parameter.
//	std::vector<basalt::RdSpline<1, 4, Scalar>> vignette;

	/// @brief Cast to other scalar type
	template <class Scalar2>
	Calibration<Scalar2> cast() const
	{
		Calibration<Scalar2> new_cam;

		for (const auto& v : mT_i_c)
		  new_cam.mT_i_c.emplace_back(v.template cast<Scalar2>());
		for (const auto& v : mIntrinsics)
		  new_cam.mIntrinsics.emplace_back(v.template cast<Scalar2>());
//		for (const auto& v : vignette)
//		  new_cam.vignette.emplace_back(v.template cast<Scalar2>());

		new_cam.mResolution = mResolution;
		new_cam.miCam_time_offset_ns = miCam_time_offset_ns;

		new_cam.mCalib_accel_bias.getParam() = mCalib_accel_bias.getParam().template cast<Scalar2>();
		new_cam.mCalib_gyro_bias.getParam() = mCalib_gyro_bias.getParam().template cast<Scalar2>();

		new_cam.mGyro_noise_std = mGyro_noise_std.template cast<Scalar2>();
		new_cam.mAccel_noise_std = mAccel_noise_std.template cast<Scalar2>();
		new_cam.mGyro_bias_std = mGyro_bias_std.template cast<Scalar2>();
		new_cam.mAccel_bias_std = mAccel_bias_std.template cast<Scalar2>();

		return new_cam;
	}


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //CALIBRATION_H_
