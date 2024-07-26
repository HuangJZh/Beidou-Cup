
























/*----------------------------------------------------------------------------------------------*/

#ifndef CONFIG_H_
#define CONFIG_H_

#define LINUX_OS

/* Software Version */
/*----------------------------------------------------------------------------------------------*/
#define SOFTWARE_VERSION_MAJOR		(2)
#define SOFTWARE_VERSION_MINOR		(6)
#define SOFTWARE_VERSION_POSTFIX	(2)
/*----------------------------------------------------------------------------------------------*/

/* The most important thing, the NUMBER OF CORRELATORS IN THE RECEIVER and the NUMBER OF CPUs */
/*----------------------------------------------------------------------------------------------*/




#define TASK_STACK_SIZE			(2048)						//!< For Nucleus/Linux compatibility
/*----------------------------------------------------------------------------------------------*/


/* MISC GPS Constants */
/*----------------------------------------------------------------------------------------------*/
#define MAX_SV					(1)		//!< Number of PRN codes//xin//#define MAX_SV					(32)		//!< Number of PRN codes
/*----------------------------------------------------------------------------------------------*/

































/* Measurement/PVT Defines */
/*----------------------------------------------------------------------------------------------*/
#define MEASUREMENT_INT			(1)		//!< Packets of ~1ms data//#define MEASUREMENT_INT			(100)		//!< Packets of ~1ms data





#define MEASUREMENT_MOD			(1)		//!< Slow down measurement transmission to the PVT

/*  */
/*----------------------------------------------------------------------------------------------*/
#define ROW						(480)
#define COL						(752)
#define MIN_DIST				(30)
#define MAX_CNT					(150)
#define FOCAL_LENGTH			(460)
#define CAM0					(0)		//!< left camera
#define CAM1					(1)		//!< right camera
/////////////////////// 604 start ////////////////////////////
#define POSE_VEL_BIAS_SIZE      (15)    //!< Dimensionality of the pose-velocity-bias state
#define POSE_VEL_SIZE      		(9)     //!< Dimensionality of the pose-velocity state
#define POSE_SIZE      			(6)     //!< Dimensionality of the pose state
struct Vioconfig
{
	/*
	Vioconfig()://initialized in utils/vio_config.cpp
		miVio_min_frames_after_kf(5),
		mfVio_new_kf_keypoints_thresh(0.7),
		
		mdVio_outlier_threshold(3.0),
		mdVio_obs_std_dev(0.5),
		mdVio_obs_huber_thresh(1.0),	
		mdVio_min_triangulation_dist(0.05),

		miVio_filter_iteration(4),
		
		mbVio_use_lm(false),
		mdVio_lm_lambda_min(1e-32),
		mdVio_lm_lambda_max(1e2),
		
		miVio_max_kfs(7),
		miVio_max_states(3)
*/

	Vioconfig()://initialized in utils/vio_config.cpp
//		mdOptical_flow_epipolar_error(0.005), //TODO: currently, directly used in acquisition.cpp
		miVio_min_frames_after_kf(5),
		mfVio_new_kf_keypoints_thresh(0.7),

		mdVio_outlier_threshold(3.0),
		mdVio_obs_std_dev(0.5),
		mdVio_obs_huber_thresh(1.0),	
		mdVio_min_triangulation_dist(0.05),

		miVio_filter_iteration(4),
		miVio_max_iterations(7),

		mbVio_use_lm(false),
		mdVio_lm_lambda_min(1e-32),
		mdVio_lm_lambda_max(1e2),

		mdVio_init_pose_weight(1e8),
		mdVio_init_ba_weight(1e1),
		mdVio_init_bg_weight(1e2),

		miVio_max_kfs(7),
		miVio_max_states(3)
	{}

//	double	mdOptical_flow_epipolar_error;//line 53

	int		miVio_min_frames_after_kf;//line 58
	float	mfVio_new_kf_keypoints_thresh;//line 59

	double	mdVio_outlier_threshold;//line 61
	double	mdVio_obs_std_dev;//line 62
	double	mdVio_obs_huber_thresh;//line 63
	double	mdVio_min_triangulation_dist;//line 64

	int		miVio_filter_iteration;//line 66
  	int		miVio_max_iterations;//line 67

	bool	mbVio_use_lm;//line 71
	double	mdVio_lm_lambda_min;// line 72
	double	mdVio_lm_lambda_max;// line 73

	double	mdVio_init_pose_weight;// line 75
	double	mdVio_init_ba_weight;// line 76
	double	mdVio_init_bg_weight;// line 77

	size_t	miVio_max_kfs;//moved from keypoint_vio.h
	size_t	miVio_max_states;//moved from keypoint_vio.h
};
/////////////////////// 604 end ////////////////////////////

#endif
