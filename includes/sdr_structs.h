
























/*----------------------------------------------------------------------------------------------*/

#ifndef SDR_STRUCTS_H_
#define SDR_STRUCTS_H_
//#include "source_cam.h"
//#include "source_imu.h"
#include "se3.hpp"
#include "eigen_utils.h"
#include <cstddef>//hash_combine
#include <set>//std::set
/*----------------------------------------------------------------------------------------------*/
/*! @ingroup STRUCTS
 *  @brief Define the CPX structure, used in the Fine_Acquisition FFT */
/*
typedef struct CPX
{
	cv::Mat										mvCam_meas;		//!< 
	std::vector<std::pair<ImuStamp, ImuAccGyr>>	mvImu_meas;		//!<
} CPX;
*/
// Inertial containers.
using Timestamp = std::int64_t; //common/vio_types.h
using ImuStamp = Timestamp;
//using ImuStampS = Eigen::Matrix<std::int64_t, 1, Eigen::Dynamic>;
// order: acceleration data [m/s^2], angular velocities [rad/s].
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;
//using ImuAccGyrS = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using ImuBias = gtsam::imuBias::ConstantBias;

// common_types.h from BASALT
using FrameId = std::int64_t;//TODO see if 'Timestamp' and 'FrameId' are the same//604
using CamId = std::size_t;//604

static const Eigen::Vector3d g(0, 0, -9.81);
// thirdparty->utils->hash.h
// to work around static_assert(false, ...)
/*template <class T> struct dependent_false : std::false_type {};

template <class T> inline void hash_combine(	  size_t& seed,
											const T&	  value)
{
	// Simple hash_combine, see e.g. here:
	// https://github.com/HowardHinnant/hash_append/issues/7
	// Not sure we ever need 32bit, but here it is ...
	if constexpr (sizeof(size_t) ==4)
		seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
	else if constexpr(sizeof(std::size_t) == 8)
		seed ^= std::hash<T>{}(value) + 0x9e3779b97f4a7c15LLU + (seed << 12) + (seed >> 4);
	else
    	static_assert(dependent_false<T>::value, "hash_combine not implemented");
}
*/

// to work around static_assert(false, ...)
/*
template <class T>
struct dependent_false : std::false_type {};

template <class T>
inline void hash_combine(std::size_t& seed, const T& value) {
  // Simple hash_combine, see e.g. here:
  // https://github.com/HowardHinnant/hash_append/issues/7
  // Not sure we ever need 32bit, but here it is...
  if constexpr (sizeof(std::size_t) == 4) {
    seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
  } else if constexpr (sizeof(std::size_t) == 8) {
    seed ^= std::hash<T>{}(value) + 0x9e3779b97f4a7c15LLU + (seed << 12) +
            (seed >> 4);
  } else {
    static_assert(dependent_false<T>::value, "hash_combine not implemented");
  }
}
*/
template <class T>
inline void hash_combine(std::size_t& seed, const T& value) {
  // Simple hash_combine, see e.g. here:
  // https://github.com/HowardHinnant/hash_append/issues/7
  // Not sure we ever need 32bit, but here it is...
    seed ^= std::hash<T>{}(value) + 0x9e3779b97f4a7c15LLU + (seed << 12) +
            (seed >> 4);
}



typedef struct ImuMeasurement
{
	using Ptr = std::shared_ptr<ImuMeasurement>;//604
    ImuStamp 	mvTimestamp;
    ImuAccGyr	mvAcc_gyr;
	
	ImuMeasurement()//604
	{
		mvAcc_gyr.setZero();
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW//604
} ImuMeasurement;

using CamStamp = Timestamp;

typedef struct CamMeasurement
{
	CamStamp	mvTimestamp[2];
	char		msImage[2][128];
} CamMeasurement;

typedef struct FeaMeasurement
{
	using Ptr = std::shared_ptr<FeaMeasurement>;//604
	
	CamStamp	mvTimestamp[2];
	int		mvIds[2][150];// xin: 2023-5-30
	float   mfCur_un_pts_x[2][150];//float   mfCur_un_pts_x[2048];
	float   mfCur_un_pts_y[2][150];//float   mfCur_un_pts_y[2048];
	uint32	muCnt_un_pts[2]; // the maximum number of feature points in a single frame is 150(MAX_CNT), so uint32 should be enough, even for future use
} FeaMeasurement;


















/*----------------------------------------------------------------------------------------------*/
/*! \ingroup STRUCTS
 * @brief Options parsed from command line to start the receiver */
typedef struct _Options_S
{

  int64_t verbose;    //!< Do a lot of extra printing
  int64_t source;     //!< GPS data source type
  std::string msInput_dataset_path; //imu, cam
//  std::string msInput_dataset_gse_path; //xin: only for visualization in gps-gse, so that file open oper in gps-gse will not contend with file open oper in gps-sdr
  std::string msImuName; //IMU
  std::string msCamName; //cam

} Options_S;
/*----------------------------------------------------------------------------------------------*/














//!< FIFO structure for linked list?
/*----------------------------------------------------------------------------------------------*/
/*! \ingroup STRUCTS
 *  @brief linked list structure for circular FIFO buffer */
/*
typedef struct ms_packet {

	ms_packet *next;
    int64_t count;          //!< Number of packets
	CPX data[SAMPS_MS];		//!< Payload size

} ms_packet;
*/
typedef struct ms_packet_imu {
	ms_packet_imu*	mpNext;
	int64_t			miCount;
	ImuMeasurement	mData[SAMPS_MS];
} ms_packet_imu;

typedef struct ms_packet_cam {
	ms_packet_cam*	mpNext;
	int64_t			miCount;
	CamMeasurement	mData[SAMPS_MS];
} ms_packet_cam;

typedef struct ms_packet_fea {
	ms_packet_fea*	mpNext;
	int64_t			miCount;
	FeaMeasurement	mData[SAMPS_MS];
} ms_packet_fea;
/*----------------------------------------------------------------------------------------------*/


///////////////////// 604 start, end at line 831//////////////////////
//------------------------------- non-thirdparty::imu_types.h----------------------
typedef struct AbsOrderMap
{
	std::map<int64_t, std::pair<int, int>> mAbs_order_map;
	size_t miItems = 0;
	size_t miTotal_size = 0;
	
	void print_order()
	{
		for (const auto& kv : mAbs_order_map)
			std::cout<<kv.first<<" ("<<kv.second.first<<","<<kv.second.second<<")"<<std::endl;
		std::cout<<std::endl;
	}
} AbsOrderMap;

//-------------------------------thirdparty imu_types.h----------------------
/// @brief State that consists of SE(3) pose at a certain time.
struct PoseState
{
	using VecN = Eigen::Matrix<double, POSE_SIZE, 1>;

	/// @brief Default constructor with Identity pose and zero timestamp.
	PoseState() { miT_ns = 0; }

	/// @brief Constructor with timestamp and pose.
	///
	/// @param t_ns timestamp of the state in nanoseconds
	/// @param T_w_i transformation from the body frame to the world frame
	PoseState(int64_t t_ns, const Sophus::SE3d& T_w_i)
	  : miT_ns(t_ns), mT_w_i(T_w_i) {}

	/// @brief Apply increment to the pose
	///
	/// For details see \ref incPose
	/// @param[in] inc 6x1 increment vector
	void applyInc(const VecN& inc)
	{
		incPose(inc, mT_w_i);
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN optimize() : IN PoseState::applyInc()"<<std::endl;
	}

	/// @brief Apply increment to the pose
	///
	/// The incremernt vector consists of translational and rotational parts \f$
	/// [\upsilon, \omega]^T \f$. Given the current pose \f$ R \in
	/// SO(3), p \in \mathbb{R}^3\f$ the updated pose is: \f{align}{ R' &=
	/// \exp(\omega) R
	/// \\ p' &= p + \upsilon
	/// \f}
	///  The increment is consistent with \ref
	/// Se3Spline::applyInc.
	///
	/// @param[in] inc 6x1 increment vector
	/// @param[in,out] T the pose to update
	inline static void incPose(const Sophus::Vector6d& inc,
									 Sophus::SE3d&     T)
	{
		T.translation() += inc.head<3>();
		if (inc.tail<3>().array().isNaN()[0] || inc.tail<3>().array().isNaN()[1] || inc.tail<3>().array().isNaN()[3])
		{
			std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN optimize() : IN PoseState::applyInc() HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH: SO3d::exp() #4"<<std::endl;//xin 20230930
		}
		T.so3() = Sophus::SO3d::exp(inc.tail<3>()) * T.so3();
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int64_t			miT_ns;        ///< timestamp of the state in nanoseconds
	Sophus::SE3d	mT_w_i;  ///< pose of the state
};

//-------------------------------thirdparty imu_types.h----------------------
/// @brief State that consists of SE(3) pose and linear velocity at a certain
/// time.
struct PoseVelState : public PoseState
{
	using VecN = Eigen::Matrix<double, POSE_VEL_SIZE, 1>;

	/// @brief Default constructor with Identity pose and zero other values.
	PoseVelState() { mVel_w_i.setZero(); };

	/// @brief Constructor with timestamp, pose and linear velocity.
	///
	/// @param t_ns timestamp of the state in nanoseconds
	/// @param T_w_i transformation from the body frame to the world frame
	/// @param vel_w_i linear velocity in world coordinate frame
	PoseVelState(      int64_t          t_ns,
			     const Sophus::SE3d&    T_w_i,
			     const Eigen::Vector3d& vel_w_i)
	  : PoseState(t_ns, T_w_i), mVel_w_i(vel_w_i) {}

	/// @brief Apply increment to the state.
	///
	/// For pose see \ref incPose. For velocity simple addition.
	/// @param[in] inc 9x1 increment vector [trans, rot, vel]
	void applyInc(const VecN& inc)
	{
		PoseState::applyInc(inc.head<6>());
		mVel_w_i += inc.tail<3>();
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN optimize() : IN PoseVelState::applyInc()"<<std::endl;
	}

	/// @brief Compute difference to other state.
	///
	/// ```
	///      PoseVelState::VecN inc;
	///      PoseVelState p0, p1;
	///      // Initialize p0 and inc
	///      p1 = p0;
	///      p1.applyInc(inc);
	///      p0.diff(p1) == inc; // Should be true.
	/// ```
	/// @param other state to compute difference.
	VecN diff(const PoseVelState& other) const 
	{
		VecN res;
		res.segment<3>(0) = other.mT_w_i.translation() - mT_w_i.translation();
		res.segment<3>(3) = (other.mT_w_i.so3() * mT_w_i.so3().inverse()).log();
		res.tail<3>() = other.mVel_w_i - mVel_w_i;
		return res;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Vector3d mVel_w_i;  ///< Linear velocity of the state
};


//-------------------------------thirdparty imu_types.h----------------------
/// @brief State that consists of SE(3) pose, linear velocity, gyroscope and
/// accelerometer biases at a certain time.
struct PoseVelBiasState : public PoseVelState
{
	using Ptr = std::shared_ptr<PoseVelBiasState>;
	using VecN = Eigen::Matrix<double, POSE_VEL_BIAS_SIZE, 1>;

	/// @brief Default constructor with Identity pose and zero other values.
	PoseVelBiasState()
	{
		mBias_gyro.setZero();
		mBias_accel.setZero();
	};

	/// @brief Constructor with timestamp, pose, linear velocity, gyroscope and
	/// accelerometer biases.
	///
	/// @param t_ns timestamp of the state in nanoseconds
	/// @param T_w_i transformation from the body frame to the world frame
	/// @param vel_w_i linear velocity in world coordinate frame
	/// @param bias_gyro gyroscope bias
	/// @param bias_accel accelerometer bias
	PoseVelBiasState(      int64_t          t_ns,
			         const Sophus::SE3d&    T_w_i,
				     const Eigen::Vector3d& vel_w_i,
				     const Eigen::Vector3d& bias_gyro,
				     const Eigen::Vector3d& bias_accel)
	  : PoseVelState(t_ns, T_w_i, vel_w_i),
		mBias_gyro(bias_gyro),
		mBias_accel(bias_accel) {}

	/// @brief Apply increment to the state.
	///
	/// For pose see \ref incPose. For velocity and biases simple addition.
	/// @param[in] inc 15x1 increment vector [trans, rot, vel, mBias_gyro,
	/// mBias_accel]
	void applyInc(const VecN& inc)
	{
		PoseVelState::applyInc(inc.head<9>());
		mBias_gyro += inc.segment<3>(9);
		mBias_accel += inc.segment<3>(12);
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN optimize() : IN PoseVelBiasState::applyInc()"<<std::endl;
	}

	/// @brief Compute difference to other state.
	///
	/// ```
	///      PoseVelBiasState::VecN inc;
	///      PoseVelBiasState p0, p1;
	///      // Initialize p0 and inc
	///      p1 = p0;
	///      p1.applyInc(inc);
	///      p0.diff(p1) == inc; // Should be true.
	/// ```
	/// @param other state to compute difference.
	VecN diff(const PoseVelBiasState& other) const 
	{
		VecN res;
		res.segment<9>(0) = PoseVelState::diff(other);
		res.segment<3>(9) = other.mBias_gyro - mBias_gyro;
		res.segment<3>(12) = other.mBias_accel - mBias_accel;
		return res;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Vector3d mBias_gyro;   ///< Gyroscope bias
	Eigen::Vector3d mBias_accel;  ///< Accelerometer bias
};

//------------------------------- imu_types.h----------------------
struct PoseVelBiasStateWithLin
{
	using VecN = PoseVelBiasState::VecN;

	PoseVelBiasStateWithLin()
	{
		mbLinearized = false;
		mDelta.setZero();
	};

	PoseVelBiasStateWithLin(      int64_t          t_ns,
			                const Sophus::SE3d&    T_w_i,
						    const Eigen::Vector3d& vel_w_i,
						    const Eigen::Vector3d& bias_gyro,
						    const Eigen::Vector3d& bias_accel,
							      bool             linearized)
	  : mbLinearized(linearized),
		mState_linearized(t_ns, T_w_i, vel_w_i, bias_gyro, bias_accel)
	{
		mDelta.setZero();
		mState_current = mState_linearized;
	}

	PoseVelBiasStateWithLin(const PoseVelBiasState& other)
	  : mbLinearized(false), mState_linearized(other)
	{
		mDelta.setZero();
		mState_current = other;
	}

	void setLinFalse()
	{
		mbLinearized = false;
		mDelta.setZero();
	}

	void setLinTrue()
	{
		mbLinearized = true;
//		FAIM_ASSERT(mDelta.isApproxToConstant(0));
		mState_current = mState_linearized;
	}

	void applyInc(const VecN& inc)
	{
		if (!mbLinearized)
			mState_linearized.applyInc(inc);
		else {
			mDelta += inc;
			mState_current = mState_linearized;
			mState_current.applyInc(mDelta);
		}
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN optimize() : IN PoseVelBiasStateWithLin::applyInc()"<<std::endl;
	}

	inline const PoseVelBiasState& getState() const
	{
		if (!mbLinearized)
			return mState_linearized;
		else
			return mState_current;
	}

	inline const PoseVelBiasState& getStateLin() const
	{
		return mState_linearized;
	}

	inline bool isLinearized() const { return mbLinearized; }
	inline const VecN& getDelta() const { return mDelta; }
	inline int64_t getT_ns() const { return mState_linearized.miT_ns; }

	friend struct PoseStateWithLin;

	inline void backup()
	{
		mBackup_delta = mDelta;
		mBackup_state_linearized = mState_linearized;
		mBackup_state_current = mState_current;
	}

	inline void restore()
	{
		mDelta = mBackup_delta;
		mState_linearized = mBackup_state_linearized;
		mState_current = mBackup_state_current;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	bool				mbLinearized;
	VecN				mDelta;
	PoseVelBiasState	mState_linearized, mState_current;

	VecN				mBackup_delta;
	PoseVelBiasState	mBackup_state_linearized, mBackup_state_current;
/*
	friend class cereal::access;

	template <class Archive>
	void serialize(Archive& ar)
	{
		ar(mState_linearized.mT_w_i);
		ar(mState_linearized.mVel_w_i);
		ar(mState_linearized.mBias_gyro);
		ar(mState_linearized.mBias_accel);
		ar(mState_current.mT_w_i);
		ar(mState_current.mVel_w_i);
		ar(mState_current.mBias_gyro);
		ar(mState_current.mBias_accel);
		ar(mDelta);
		ar(mbLinearized);
		ar(mState_linearized.miT_ns);
	}*/
};


//------------------------------- common_types.h----------------------
/// pair of image timestamp and camera id identifies an image (imageId)
struct TimeCamId
{
	TimeCamId() : miFrame_id(0), miCam_id(0) {}

	TimeCamId(const FrameId&	frame_id,
			  const CamId&		cam_id)
		: miFrame_id(frame_id), miCam_id(cam_id) {}

	FrameId miFrame_id;
	CamId   miCam_id;
};

inline bool operator<(const TimeCamId& o1, const TimeCamId& o2) {
  if (o1.miFrame_id == o2.miFrame_id) return o1.miCam_id < o2.miCam_id;
  return o1.miFrame_id < o2.miFrame_id;
}

// required at correlator.cpp about line 851
inline bool operator==(const TimeCamId& o1, const TimeCamId& o2) {
  return o1.miFrame_id == o2.miFrame_id && o1.miCam_id == o2.miCam_id;
}

inline bool operator!=(const TimeCamId& o1, const TimeCamId& o2) {
  return o1.miFrame_id != o2.miFrame_id || o1.miCam_id != o2.miCam_id;
}

// common_types.h
namespace std
{
	template <> struct hash<TimeCamId>
	{
		size_t operator()(const TimeCamId& x) const
		{
			size_t seed = 0;
			hash_combine(seed, x.miFrame_id);
			hash_combine(seed, x.miCam_id);
			return seed;
		}
	};
}


//----------------------- landmark_database.h ----------------------
// keypoint position defined relative to some frame
struct KeypointPosition
{
	TimeCamId		mKf_id;
	Eigen::Vector2d mDir;
	double			mdId;
	double			mdBackup_id;

	inline void backup()
	{
		mBackup_dir = mDir;
		mdBackup_id = mdId;
	}

	inline void restore()
	{
		mDir = mBackup_dir;
		mdId = mdBackup_id;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Eigen::Vector2d mBackup_dir;
//	double			mdBackup_id;
};//struct KeypointPosition

struct KeypointObservation
{
	int				miKpt_id;
	Eigen::Vector2d mPos;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef struct LandmarkDatabase
{
public:
	// Non-const
	void addObservation(const TimeCamId				&tcid_target,
						const KeypointObservation	&o)
	{
		auto it = mKpts.find(o.miKpt_id);
//		FAIM_ASSERT(it != mKpts.end());
		
		auto &obs_vec = mObs[it->second.mKf_id][tcid_target];

/*		// Check that the point observation is inserted only once
		for (const auto &oo : obs_vec)
		{
			FAIM_ASSERT(oo.miKpt_id ! o.miKpt_id); 
		}
*/
		obs_vec.emplace_back(o);

		miNum_observations++;
		mKpts_num_obs[o.miKpt_id]++;
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::measure(): ---------- IN LandmarkDatabase::addObservation() "<<std::endl;
	}
	
	void addLandmark(	   int				lm_id,
					 const KeypointPosition &pos)
	{
		mKpts[lm_id] = pos;
		mHost_to_kpts[pos.mKf_id].emplace(lm_id);
	}

	inline void backup()
	{
		for (auto& kv : mKpts) kv.second.backup();
	}

	inline void restore()
	{
		for (auto& kv : mKpts) kv.second.restore();
	}

	void removeKeyframes(const std::set<FrameId> &kfs_to_marg,
    					 const std::set<FrameId> &poses_to_marg,
    					 const std::set<FrameId> &states_to_marg_all)
	{
		// remove points
		for (auto it = mKpts.cbegin(); it != mKpts.cend();)
			if (kfs_to_marg.count(it->second.mKf_id.miFrame_id) > 0)
			{
				auto num_obs_it = mKpts_num_obs.find(it->first);
				miNum_observations -= num_obs_it->second;
				mKpts_num_obs.erase(num_obs_it);

				it = mKpts.erase(it);
			} else
			  ++it;

		for (auto it = mObs.cbegin(); it != mObs.cend();)
			if (kfs_to_marg.count(it->first.miFrame_id) > 0)
				it = mObs.erase(it);
			else
				++it;

		for (auto it = mObs.begin(); it != mObs.end(); ++it)
			for (auto it2 = it->second.cbegin(); it2 != it->second.cend();)
				if (poses_to_marg.count(it2->first.miFrame_id) > 0 ||
				  	states_to_marg_all.count(it2->first.miFrame_id) > 0 ||
				  	kfs_to_marg.count(it->first.miFrame_id) > 0)
				{
					for (const auto &v : it2->second)
						mKpts_num_obs.at(v.miKpt_id)--;

					it2 = it->second.erase(it2);
				} else
					++it2;

		for (auto it = mHost_to_kpts.cbegin(); it != mHost_to_kpts.cend();)
			if (kfs_to_marg.count(it->first.miFrame_id) > 0 ||
				states_to_marg_all.count(it->first.miFrame_id) > 0 ||
				poses_to_marg.count(it->first.miFrame_id) > 0)
				it = mHost_to_kpts.erase(it);
			else
				++it;
	}

	// Const
	bool landmarkExists(int lm_id) const
	{
		return mKpts.count(lm_id) > 0;
	}

	KeypointPosition& getLandmark(int lm_id)
	{
		return mKpts.at(lm_id);
	}
	
	const KeypointPosition& getLandmark(int lm_id) const
	{
		return mKpts.at(lm_id);
	}

	size_t numLandmarks() const { return mKpts.size(); }

	const Eigen::aligned_map<TimeCamId,
                             Eigen::aligned_map<TimeCamId, Eigen::aligned_vector<KeypointObservation> > >& 
			getObservations() const {return mObs;}
    
	int numObservations(int lm_id) const
	{
  		return mKpts_num_obs.at(lm_id);
	}

	/************* landmark_database.cpp ***************/
	void removeLandmark(int lm_id)
	{
		auto it = mKpts.find(lm_id);
//		FAIM_ASSERT(it != mKpts.end());

		mHost_to_kpts.at(it->second.mKf_id).erase(lm_id);

		std::set<TimeCamId> to_remove;

		for (auto &kv : mObs.at(it->second.mKf_id))
		{
			int idx = -1;
			for (size_t i = 0; i < kv.second.size(); ++i)
			{
				if (kv.second[i].miKpt_id == lm_id)
				{
					idx = i;
					break;
				}
			}

			if (idx >= 0)
			{
//				FAIM_ASSERT(kv.second.size() > 0);

				std::swap(kv.second[idx], kv.second[kv.second.size() - 1]);
				kv.second.resize(kv.second.size() - 1);

				miNum_observations--;
				mKpts_num_obs.at(lm_id)--;

				if (kv.second.size() == 0)
					to_remove.insert(kv.first);
			}
		}

		for (const auto &v : to_remove)
		{
			mObs.at(it->second.mKf_id).erase(v);
		}

//		FAIM_ASSERT_STREAM(mKpts_num_obs.at(lm_id) == 0, mKpts_num_obs.at(lm_id));
		mKpts_num_obs.erase(lm_id);
		mKpts.erase(lm_id);
	}

	void removeObservations(	  int				  lm_id,
							const std::set<TimeCamId> &outliers)
	{
		auto it = mKpts.find(lm_id);
//		FAIM_ASSERT(it != mKpts.end());

		std::set<TimeCamId> to_remove;

		for (auto &kv : mObs.at(it->second.mKf_id))
		{
			if (outliers.count(kv.first) > 0)
			{
				int idx = -1;
				for (size_t i = 0; i < kv.second.size(); i++)
				{
					if (kv.second[i].miKpt_id == lm_id) {
						idx = i;
						break;
					}
				}
//				FAIM_ASSERT(idx >= 0);
//				FAIM_ASSERT(kv.second.size() > 0);

				std::swap(kv.second[idx], kv.second[kv.second.size() - 1]);
				kv.second.resize(kv.second.size() - 1);

				miNum_observations--;
				mKpts_num_obs.at(lm_id)--;

				if (kv.second.size() == 0)
					to_remove.insert(kv.first);
			}
		}

		for (const auto &v : to_remove)
		{
			mObs.at(it->second.mKf_id).erase(v);
		}
	}

private:
	Eigen::aligned_unordered_map<int, KeypointPosition> mKpts;
	Eigen::aligned_map<TimeCamId, Eigen::aligned_map<TimeCamId,
													 Eigen::aligned_vector<KeypointObservation>>> //xin: just take this as a triplet: <tcid_host, tcid_target, bunch of KeypointPosition>
														mObs;
	
	std::unordered_map<TimeCamId, std::set<int>> 		mHost_to_kpts;
	int 												miNum_observations = 0;
	Eigen::aligned_unordered_map<int, int> 		 		mKpts_num_obs;
} LandmarkDatabase;

//------------------------------- imu_types.h----------------------
struct PoseStateWithLin
{
    using VecN = PoseState::VecN;

    PoseStateWithLin()
	{
		linearized = false;
		delta.setZero();
    };

    PoseStateWithLin(      int64_t       t_ns,
			         const Sophus::SE3d& T_w_i,
                           bool          linearized = false)
      : linearized(linearized),
	    pose_linearized(t_ns, T_w_i)
	{
        delta.setZero();
        T_w_i_current = T_w_i;
    }

    PoseStateWithLin(const PoseVelBiasStateWithLin& other)
      : linearized(other.mbLinearized),
        delta(other.mDelta.head<6>()),
        pose_linearized(other.mState_linearized.miT_ns,
                        other.mState_linearized.mT_w_i)
	{
        T_w_i_current = pose_linearized.mT_w_i;
        PoseState::incPose(delta, T_w_i_current);
    }

    void setLinTrue()
	{
        linearized = true;
//		FAIM_ASSERT(delta.isApproxToConstant(0));
        T_w_i_current = pose_linearized.mT_w_i;
    }

    inline const Sophus::SE3d& getPose() const
	{
		if (!linearized)
		    return pose_linearized.mT_w_i;
		else
		    return T_w_i_current;
    }

    inline const Sophus::SE3d& getPoseLin() const
	{
        return pose_linearized.mT_w_i;
    }

    inline void applyInc(const VecN& inc)
	{
		if (!linearized)
			PoseState::incPose(inc, pose_linearized.mT_w_i);
		else
		{
		    delta += inc;
		    T_w_i_current = pose_linearized.mT_w_i;
		    PoseState::incPose(delta, T_w_i_current);
		}
//xin 20230803	std::cout<<"HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH Correlator::TakeMeasurements(): IN measure(): IN optimize() : IN PoseStateWithLin::applyInc()"<<std::endl;
    }

    inline bool isLinearized() const { return linearized; }
    inline const VecN& getDelta() const { return delta; }
    inline int64_t getT_ns() const { return pose_linearized.miT_ns; }

    inline void backup()
	{
		backup_delta = delta;
		backup_pose_linearized = pose_linearized;
		backup_T_w_i_current = T_w_i_current;
    }

    inline void restore()
	{
		delta = backup_delta;
		pose_linearized = backup_pose_linearized;
		T_w_i_current = backup_T_w_i_current;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private://TODO: standardize these data memebers!!!
    bool         linearized;
    VecN         delta;
    PoseState    pose_linearized;
    Sophus::SE3d T_w_i_current;

    VecN         backup_delta;
    PoseState    backup_pose_linearized;
    Sophus::SE3d backup_T_w_i_current;

/*    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
	{
		ar(pose_linearized.mT_w_i);
		ar(T_w_i_current);
		ar(delta);
		ar(linearized);
		ar(pose_linearized.miT_ns);
*/
};

struct RelLinDataBase//TODO: standardize the data memebers of RelLinDataBase!!!
{
	std::vector<std::pair<TimeCamId, TimeCamId>> order;
	Eigen::aligned_vector<Sophus::Matrix6d> 	 d_rel_d_h;
	Eigen::aligned_vector<Sophus::Matrix6d>		 d_rel_d_t;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FrameRelLinData
{
	Sophus::Matrix6d									Hpp;
	Sophus::Vector6d									bp;
	std::vector<int>									lm_id;
	Eigen::aligned_vector<Eigen::Matrix<double, 6, 3>>	Hpl;

	FrameRelLinData()
	{
		Hpp.setZero();
		bp.setZero();
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct RelLinData : public RelLinDataBase
{
	RelLinData(size_t num_keypoints,
			   size_t num_rel_poses)
	{
		Hll.reserve(num_keypoints);
		bl.reserve(num_keypoints);
		lm_to_obs.reserve(num_keypoints);

		Hpppl.reserve(num_rel_poses);
		order.reserve(num_rel_poses);

		d_rel_d_h.reserve(num_rel_poses);
		d_rel_d_t.reserve(num_rel_poses);

		error = 0;
	}

	void invert_keypoint_hessians()
	{
		for (auto& kv : Hll)
		{
			Eigen::Matrix3d Hll_inv;
			Hll_inv.setIdentity();
			kv.second.ldlt().solveInPlace(Hll_inv);
			kv.second = Hll_inv;
		}
	}

	Eigen::aligned_unordered_map<int, Eigen::Matrix3d>							Hll;
	Eigen::aligned_unordered_map<int, Eigen::Vector3d>							bl;
	Eigen::aligned_unordered_map<int, std::vector<std::pair<size_t, size_t>>>	lm_to_obs;
	Eigen::aligned_vector<FrameRelLinData>										Hpppl;
	double 																		error;
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/******* ba_base.h ********/
/*
inline void backup()
{
	for (auto& kv : frame_states)
		kv.second.backup();

	for (auto& kv : frame_poses)
		kv.second.backup();

	lmdb.backup();
}

inline void restore()
{
	for (auto& kv : frame_states)
		kv.second.restore();

	for (auto& kv : frame_poses)
		kv.second.restore();
	
	lmdb.restore();
}
*/

/******* optical_flow.h ********/
typedef struct OpticalFlowResultReduced
{
	typedef std::shared_ptr<OpticalFlowResultReduced> Ptr;

	std::vector<Eigen::aligned_map<int, cv::Point2f>> observations;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}OpticalFlowResultReduced;


typedef struct MargData
{
	typedef std::shared_ptr<MargData> Ptr;

	AbsOrderMap												mAom;
	Eigen::MatrixXd											mAbs_H;
	Eigen::VectorXd											mAbs_b;
	Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin>	mFrame_states;
	Eigen::aligned_map<int64_t, PoseStateWithLin>			mFrame_poses;
	std::set<int64_t>										mKfs_all;
	std::set<int64_t>										mKfs_to_marg;
	bool													mbUse_imu;

//	std::vector<FeaMeasurement::Ptr>						mOpt_flow_res;
	std::vector<OpticalFlowResultReduced::Ptr>				mOpt_flow_res;
} MargData;
///////////////////// 604 end, started at line 166//////////////////////

#endif /* SDR_STRUCTS_H_ */
