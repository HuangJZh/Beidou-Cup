


























#ifndef ACQUISITION_H_
#define ACQUISITION_H_
#include "includes.h"
#include "sv_select.h"
#include "fifo_imu.h"
#include "fifo_cam.h"
//#include "fft.h"
#include "accessories/PinholeCamera.h"
#include "fifo_fea.h"
#include <iterator>
/*! @ingroup CLASSES
	@brief /xyzzy */
class Acquisition : public Threaded_Object
{
/*
	typedef struct Acq_Result{
	int32	code_phase;
		int32	doppler;
		uint32	magnitude;
		float	relmagn;
	} Acq_Result;
*/
	private:
		ms_packet_imu	packet_imu;					//!< Get IF data
		ms_packet_cam	packet_cam;					//!< Get IF data
		CamMeasurement *mpBuff_cam;
//		ImuMeasurement *mpBuff_imu;
		FeaMeasurement *mpBuff_fea;
		
		int 			state;						//!< Search using this state (STRONG, MEDIUM, or WEAK)
		Acq_Command_S	request;					//!< Acquisition transaction
		Acq_Command_S	results[MAX_SV];			//!< Where to store the results //TODO TODO: MAX_SV: should be maximum allowable number of feature points
	    
		// VIO, previously ONLY left cam; now switched to BOTH cameras by xin 20230715
		cv::Mat										mMask[2];				//!< mask used in cv::goodFeaturesToTrack()
		cv::Mat										prev_img[2];		//!< Current-frame image
		cv::Mat										cur_img[2];			//!< Next-frame image
		std::vector<cv::Point2f>					mvN_pts[2];			//!< Next-frame feature points by cv::goodFeaturesToTrack()
		std::vector<cv::Point2f>					mvPrev_pts[2];		//!< Current-frame feature points
		std::vector<cv::Point2f>					mvPrev_un_pts[2];	//!< Current-frame feature points, undistorted
		std::vector<int>							mvTrack_cnt[2];		//!< count/length for each track id

		//VIO, left & right cam
		std::vector<cv::Point2f>					mvCur_pts[2];	//!< Next-frame feature points
		std::vector<cv::Point2f>					mvCur_un_pts[2];//!< Undistorted current-frame feature points
		std::vector<int>							mvIds[2];		//!< track ids
		std::vector<int>							mvIds_CAM1_tmp;
		
		Eigen::Matrix4d								mmE;			//!< from keypoints.h. Essential matrix
		std::set<int>								lm_to_remove;//xin 20230816

	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Acquisition();								//!< Create and initialize object, need _fsample as a necessary argument
		~Acquisition();								//!< Shutdown gracefully
		void			doAcqStrong(int32 _sv);
		void			Import();					//!< Get a chuck of data to operate on
		void			Export();					//!< Dump results
		void			Acquire();					//!< Acquire with respect to current state
		void			Start();

		// VIO
		size_t          miN_id;
		void			setMask(int camid);
		void			addPoints(int camid);

		// optical_flow
		void			filterPoints();
		bool			updateID(unsigned int i, const std::vector<uchar>& status_cam1);
		bool			updateID(unsigned int i);
};


#endif /*ACQUISITION_H_*/
