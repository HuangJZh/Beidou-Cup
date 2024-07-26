


























#include "acquisition.h"
#include <cstdlib>
#include <cmath>


/*----------------------------------------------------------------------------------------------*/
void *Acquisition_Thread(void *_arg)
{
	Acquisition *aAcquisition = pAcquisition;

	while(grun)
	{
		aAcquisition->Import();
		aAcquisition->Acquire();
		aAcquisition->Export();
		aAcquisition->IncExecTic();
	}

	pthread_exit(0);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void Acquisition::Start()
{

	Start_Thread(Acquisition_Thread, NULL);

//	if(gopt.verbose)
		fprintf(stdout,"Acquisition thread started\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 * Acquisition(): Constructor
 * */
Acquisition::Acquisition():Threaded_Object("ACQ333TASK") //xin//Acquisition::Acquisition(float _fsample, float _fif):Threaded_Object("ACQTASK")
{
	
	miN_id = 0;

//	mpBuff_imu = new ImuMeasurement[1];
	mpBuff_cam = new CamMeasurement[1];
	mpBuff_fea = new FeaMeasurement[1];//xin 0505
	

	Eigen::Matrix<double, 4, 4> Ed;	
	Eigen::Quaterniond q0(0.7123125505904486, -0.007239825785317818, 0.007541278561558601, 0.7017845426564943);
	Eigen::Quaterniond q1(0.7115930283929829, -0.0023360576185881625,0.013000769689092388, 0.7024677108343111);
	Eigen::Vector3d t0(-0.016774788924641534, -0.068938940687127, 0.005139123188382424);
	Eigen::Vector3d t1(-0.01507436282032619,  0.0412627204046637, 0.00316287258752953);
	Sophus::SE3d calib_mT_i_c_0(q0.normalized(), t0);
	Sophus::SE3d calib_mT_i_c_1(q1.normalized(), t1);
	Sophus::SE3d T_i_j = calib_mT_i_c_0.inverse() * calib_mT_i_c_1;
	computeEssential(T_i_j, Ed);
	std::cout<<Ed<<std::endl;
	mmE=Ed;
//	if(gopt.verbose)
		fprintf(stdout,"Creating Acquisition\n");
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 * ~Acquisition(): Deconstructor
 * */
Acquisition::~Acquisition()
{
//	delete [] mpBuff_imu;
	delete [] mpBuff_cam;
	delete [] mpBuff_fea;
//	if(gopt.verbose)
		fprintf(stdout,"Destructing Acquisition\n");

}
/*----------------------------------------------------------------------------------------------*/

bool inBorder(const cv::Point2f &pt)
{
	const int BORDER_SIZE = 1;
	int img_x = cvRound(pt.x);
	int img_y = cvRound(pt.y);
	return BORDER_SIZE<=img_x && img_x<COL-BORDER_SIZE &&
		   BORDER_SIZE<=img_y && img_y<ROW-BORDER_SIZE;
}

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
	int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//inline bool unproject(const Vec2&	proj,
//							Vec4&	p3d,
inline void unproject(const Eigen::aligned_vector<Eigen::Vector2f>& proj,
					  		Eigen::aligned_vector<Eigen::Vector4f>& p3d,
					  		std::vector<bool>& 						unproj_success,
					  		int										cam)
{
	p3d.resize(proj.size());
	unproj_success.resize(proj.size());
	for (auto e : unproj_success)
		e = true;
	
	double fx,fy,cx,cy,xi,alpha;
	double mx,my,r2,xi2_2,xi1_2,sqrt2,norm2,mz,mz2,norm1,sqrt1,k;
	if (cam == CAM0)
	{
		fx = 349.7560023050409;//param[0];
		fy = 348.72454229977037;//param[1];
		cx = 365.89440762590149;//param[2];
		cy = 249.32995565708704;//param[3];
		xi = -0.2409573942178872;//param[4];
		alpha = 0.566996899163044;//param[5];
	}
	else
	{
		fx = 361.6713883800533;//param[0];
		fy = 360.5856493689301;//param[1];
		cx = 379.40818394080869;//param[2];
		cy = 255.9772968522045;//param[3];
		xi = -0.21300835384809328;//param[4];
		alpha = 0.5767008625037023;//param[5];
	}

	for (size_t ii = 0; ii < p3d.size(); ii++)
	{


		mx = (proj[ii][0] - cx) / fx;
		my = (proj[ii][1] - cy) / fy;

		r2 = mx * mx + my * my;

		if (alpha > double(0.5))
		{
//			if (r2 >= double(1) / (double(2) * alpha - double(1))) return false;
			if (r2 >= double(1) / (double(2) * alpha - double(1)))
			{
				unproj_success[ii] = false;
				break;
			}
		}

		xi2_2 = alpha * alpha;
		xi1_2 = xi * xi;

		sqrt2 = sqrt(double(1) - (double(2) * alpha - double(1)) * r2);

		norm2 = alpha * sqrt2 + double(1) - alpha;

		mz = (double(1) - xi2_2 * r2) / norm2;
		mz2 = mz * mz;

		norm1 = mz2 + r2;
		sqrt1 = sqrt(mz2 + (double(1) - xi1_2) * r2);
		k = (mz * xi + sqrt1) / norm1;

		p3d[ii][0] = k * mx;
		p3d[ii][1] = k * my;
		p3d[ii][2] = k * mz - xi;
		p3d[ii][3] = double(0);

//		return true;
//		unproj_success[ii] = true;
	}
}
void Acquisition::filterPoints()
{
//	std::set<int> lm_to_remove;

	lm_to_remove.clear();

	std::vector<int> kpid;
	Eigen::aligned_vector<Eigen::Vector2f> proj0, proj1;

	for (auto cam1_iter = mvIds[CAM1].begin(); cam1_iter < mvIds[CAM1].end(); cam1_iter++) 
	{
		auto cam0_iter = std::find(mvIds[CAM0].begin(), mvIds[CAM0].end(), *cam1_iter);
		size_t cam0_offset = std::distance(mvIds[CAM0].begin(), cam0_iter);
		size_t cam1_offset = std::distance(mvIds[CAM1].begin(), cam1_iter);

		if (cam0_iter != mvIds[CAM0].end())
		{
			proj0.emplace_back(Eigen::Vector2f(mvCur_pts[CAM0].at(cam0_offset).x,
											   mvCur_pts[CAM0].at(cam0_offset).y));
			proj1.emplace_back(Eigen::Vector2f(mvCur_pts[CAM1].at(cam1_offset).x,
											   mvCur_pts[CAM1].at(cam1_offset).y));
			kpid.emplace_back(*cam1_iter);
		}
	}

	Eigen::aligned_vector<Eigen::Vector4f> p3d0, p3d1;
	std::vector<bool> p3d0_success, p3d1_success;

	unproject(proj0, p3d0, p3d0_success, CAM0);
	unproject(proj1, p3d1, p3d1_success, CAM1);

	for (size_t i = 0; i < p3d0_success.size(); i++)
	{
		if (p3d0_success[i] && p3d1_success[i])
		{
			double epipolar_error = std::abs(p3d0[i].transpose().cast<double>() * mmE * p3d1[i].cast<double>());

			if (epipolar_error > 0.005)
				lm_to_remove.emplace(kpid[i]);
		} else {
			lm_to_remove.emplace(kpid[i]);
		}
	}

	for (int id : lm_to_remove)
	{
		auto iter3 = std::find(mvIds[CAM1].begin(), mvIds[CAM1].end(), id);
		size_t iter3_offset = std::distance(mvIds[CAM1].begin(), iter3);
		mvCur_pts[CAM1].erase(mvCur_pts[CAM1].begin() + iter3_offset);
		mvIds[CAM1].erase(iter3);
		mvTrack_cnt[CAM1].erase(mvTrack_cnt[CAM1].begin() + iter3_offset);//20230716
	}
}
/*----------------------------------------------------------------------------------------------*/
/*!
 * doAcqStrong: Acquire using a 1 ms coherent integration
 * */
void Acquisition::doAcqStrong(int32 _sv)
{
/* //xin 20230818
	std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
	std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  Acquisition, execution_tic = "<<execution_tic<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
	std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
*/
	Acq_Command_S *result = &results[_sv];
	cv::Mat equalized_img[2];
	cv::Mat img[2];
		
	cv::Ptr<cv::CLAHE> clache = cv::createCLAHE(3.0, cv::Size(8, 8));
	std::vector<uchar> status[2];
	std::vector<uchar> recovered_status[2];
	std::vector<float> err[2];
	std::vector<float> recovered_err[2];

	for (size_t camid = 0; camid < 2; camid++)
	{
		// read in images to result
		memcpy(&result->miTimestamp_Cam[camid], &(mpBuff_cam[0].mvTimestamp[camid]), sizeof(int64_t));
		memcpy(&result->msImage[camid][0],      &(mpBuff_cam[0].msImage[camid][0]),  128*sizeof(char));
		// read in images to do feature tracking	
		img[camid] = cv::imread(std::string(mpBuff_cam[0].msImage[camid]), cv::IMREAD_ANYCOLOR);
		if (img[camid].channels() > 1)
			cv::cvtColor(img[camid], img[camid], cv::COLOR_BGR2GRAY);
		clache->apply(img[camid], equalized_img[camid]);
		if (cur_img[camid].empty())
			prev_img[camid] = cur_img[camid] = equalized_img[camid];
		else
			cur_img[camid] = equalized_img[camid];
		
		// some preparations
		mvCur_pts[camid].clear();
		mvN_pts[camid].clear();
	}

	if (execution_tic == 0)//first run of Acquisition of the left camera::Import->Acquire->Export 
	{
		/////// step 0.1 detect points in CAM0
		if (!cur_img[CAM0].empty())
			cv::goodFeaturesToTrack(cur_img[CAM0],
									mvN_pts[CAM0],
									MAX_CNT,
									0.01,
									MIN_DIST,
									mMask[CAM0]);
		else
			std::cout<<"[execution_tic = 0][if (!cur_img[CAM0].empty())] fails"<<std::endl;



		/////// step 0.2 detect points in CAM1 by tracking CAM0 & //////// step 0.3 addPoints of BOTH cameras
		if (!cur_img[CAM1].empty())
		{
			if (mvN_pts[CAM0].size() > 0)
			{
				cv::calcOpticalFlowPyrLK(cur_img[CAM0],
										 cur_img[CAM1],
										 mvN_pts[CAM0], //mvCur_pts[CAM0],
										 mvN_pts[CAM1],
										 status[CAM1],
										 err[CAM1],
										 cv::Size(21,21),
										 3);
				for (size_t i = 0; i < mvN_pts[CAM1].size(); i++)
					if (status[CAM1][i] && !inBorder(mvN_pts[CAM1][i]))
						status[CAM1][i] = 0;
				//addPoints(CAM1);
				if (mvN_pts[CAM0].size() != mvN_pts[CAM1].size()) {std::cout<<"execution_tic = 0 : mvN_pts[CAM0].size() != mvN_pts[CAM1].size()"<<std::endl;exit(-1);}
				for (size_t i = 0; i < mvN_pts[CAM0].size(); i++)
				{
					for (size_t camid = 0; camid < 2; camid++)
					{
						mvCur_pts[camid].push_back(mvN_pts[camid][i]);
						mvIds[camid].push_back(miN_id);
						mvTrack_cnt[camid].push_back(1);
					}
					miN_id++;
				}
				reduceVector(mvIds[CAM1], status[CAM1]);
				reduceVector(mvCur_pts[CAM1], status[CAM1]);
				reduceVector(mvTrack_cnt[CAM1], status[CAM1]);
			}
			else
				std::cout<<"[execution_tic = 0][if (mvN_pts[CAM0]).size() > 0] fails"<<std::endl;
		}
		else
			std::cout<<"[execution_tic = 0][if (!cur_img[CAM1].empty())] fails"<<std::endl;
		/// stdcout
/* //xin 20230818
		std::cout<<"-------------------------------- Acquisition: execution_tic = 0.2 & 0.3 (AFTER calcOpticalFlowPyrLKing CAM1 & addPoints of BOTH cameras);"<<std::endl
			<<"mvN_pts[CAM0].size() = (150)"<<mvN_pts[CAM0].size()
			<<", mvN_pts[CAM1].size() = (150)"<<mvN_pts[CAM1].size()<<std::endl
			<<", mvCur_pts[CAM0].size() = (150)"<<mvCur_pts[CAM0].size() 
			<<", mvCur_pts[CAM1].size() = (150)"<<mvCur_pts[CAM1].size()<<std::endl
			<<", status[CAM0].size() = (0)"<<status[CAM0].size()
			<<", status[CAM1].size() = (150)"<<status[CAM1].size()<<std::endl
			<<", mvIds[CAM0].size() = (150)"<<mvIds[CAM0].size()
			<<", mvIds[CAM1].size() = (0)"<<mvIds[CAM1].size()<<std::endl
			<<", mvTrack_cnt[CAM1].size() = (150)"<<mvTrack_cnt[CAM0].size()
			<<", mvTrack_cnt[CAM1].size() = (150)"<<mvTrack_cnt[CAM1].size()<<std::endl
			<<"----------------------------------"<<std::endl;
		for (size_t camid = 0; camid < 2; camid++)
		{
			std::cout<<"CAM"<<camid<<":";
			size_t lcv = 0;
			if (mvCur_pts[camid].size() == 0) std::cout<<"empty"<<std::endl; else std::cout<<std::endl;
			for (auto pt : mvCur_pts[camid])
			{
				std::cout<<lcv<<" : (un_pt.x, un_pt.y) = ("<<pt.x<<", "<<pt.y<<")    kpt_id = "<<mvIds[camid][lcv]<<std::endl;
				lcv++;
			}
			std::cout<<std::endl;
		}
*/

		/////// step 0.4 check if points between CAM0(t) and CAM1(t) constrained by epipole
		filterPoints();
		/// stdcout
/* //xin 20230818
		std::cout<<"-------------------------------- Acquisition: execution_tic = 0.4; (AFTER filterPoints)"<<std::endl
			<<"mvN_pts[CAM0].size() = (150)"<<mvN_pts[CAM0].size()
			<<", mvN_pts[CAM1].size() = (150)"<<mvN_pts[CAM1].size()<<std::endl// we did not reduceVectoring mvN_pts[CAM1]
			<<", mvCur_pts[CAM0].size() = (150)"<<mvCur_pts[CAM0].size() 
			<<", mvCur_pts[CAM1].size() = (58)"<<mvCur_pts[CAM1].size()<<std::endl
			<<", status[CAM0].size() = (150)"<<status[CAM0].size()
			<<", status[CAM1].size() = (150)"<<status[CAM1].size()<<std::endl
			<<", mvIds[CAM0].size() = (150)"<<mvIds[CAM0].size()
			<<", mvIds[CAM1].size() = (58)"<<mvIds[CAM1].size()<<std::endl
			<<", mvTrack_cnt[CAM1].size() = (150)"<<mvTrack_cnt[CAM0].size()
			<<", mvTrack_cnt[CAM1].size() = (58)"<<mvTrack_cnt[CAM1].size()<<std::endl
			<<"----------------------------------"<<std::endl;
		for (size_t camid = CAM1; camid < 2; camid++)//xin 20230816
		{
			std::cout<<"CAM"<<camid<<":";
			size_t lcv = 0;
			if (mvCur_pts[camid].size() == 0) std::cout<<"empty"<<std::endl; else std::cout<<std::endl;
			for (auto pt : mvCur_pts[camid])
			{
				std::cout<<lcv<<" : (filtered.x, filtered.y) = ("<<pt.x<<", "<<pt.y<<")    kpt_id = "<<mvIds[camid][lcv]<<std::endl;
				lcv++;
			}
			std::cout<<std::endl;
		}
		for (auto pt1 : mvIds[CAM1])
			std::cout<<pt1<<", ";
		std::cout<<std::endl;

		if (lm_to_remove.size() > 0)
		{
			std::cout<<"lm_to_remove.size() = "<<lm_to_remove.size()<<". Remove:"<<std::endl;
			for (auto kpt_id : lm_to_remove)
				std::cout <<kpt_id<< ", "<<std::endl;
		}
		else
			std::cout<<"lm_to_remove.size() = 0"<<std::endl;
		std::cout<<std::endl<<std::endl;
*/
	}
	else //if (execution_tic == 0)
	{
		std::vector<cv::Point2f>					recovered_prev_pts[2];		
		/////// Step 1. cv::calcOpticalFlowPyrLK
		for (size_t camid = 0; camid < 2; camid++)
		{

			if (mvPrev_pts[camid].size() > 0)
			{
				status[camid].clear();
				err[camid].clear();
				cv::calcOpticalFlowPyrLK(prev_img[camid],
										 cur_img[camid],
										 mvPrev_pts[camid],
										 mvCur_pts[camid],
										 status[camid],
										 err[camid],
										 cv::Size(21,21),
										 3);
				for (size_t i = 0; i < mvCur_pts[camid].size(); i++)
					if (status[camid][i] && !inBorder(mvCur_pts[camid][i]))
						status[camid][i] = 0;

				recovered_status[camid].clear();
				recovered_err[camid].clear();
				recovered_prev_pts[camid].clear();
				cv::calcOpticalFlowPyrLK(cur_img[camid],
										 prev_img[camid],
										 mvCur_pts[camid],
										 recovered_prev_pts[camid],
										 recovered_status[camid],
										 recovered_err[camid],
										 cv::Size(21, 21),
										 3);
				for (size_t i = 0; i < recovered_prev_pts[camid].size(); i++)
				{
					double dist2 = cv::norm(mvPrev_pts[camid][i] - recovered_prev_pts[camid][i]);
					if (recovered_status[camid][i] && inBorder(recovered_prev_pts[camid][i]))
					{
						if (dist2 > 0.2) //note squaredNorm should use 0.04. TODO: do it in config file
							status[camid][i] = 0;
					}
				}

				reduceVector(mvPrev_pts[camid], status[camid]);
				reduceVector(mvCur_pts[camid], status[camid]);
				reduceVector(mvIds[camid], status[camid]);
				reduceVector(mvTrack_cnt[camid], status[camid]);
				
				for (auto &n : mvTrack_cnt[camid])
					n++;
			}//if (mvPrev_pts[camid].size() > 0)
		}//for (size_t camid = 0; camid < 2; camid++)
		/// stdcout
/* //xin 20230818
		std::cout<<"-------------------------------- Acquisition: execution_tic = "<<execution_tic<<".1 (AFTER calcOpticalFlowPyrLKing CAM0(t-1)->CAM0(t) & CAM1(t-1)->CAM1(t));"<<std::endl
			<<"mvN_pts[CAM0].size() = (0) don't care"<<mvN_pts[CAM0].size()
			<<", mvN_pts[CAM1].size() = (0) dont't care"<<mvN_pts[CAM1].size()<<std::endl
			<<", mvCur_pts[CAM0].size() = (not 150)"<<mvCur_pts[CAM0].size() 
			<<", mvCur_pts[CAM1].size() = (58)"<<mvCur_pts[CAM1].size()<<std::endl
			<<", status[CAM0].size() = (not 150)"<<status[CAM0].size()
			<<", status[CAM1].size() = (58)"<<status[CAM1].size()<<std::endl
			<<", mvIds[CAM0].size() = (not 150)"<<mvIds[CAM0].size()
			<<", mvIds[CAM1].size() = (58)"<<mvIds[CAM1].size()<<std::endl
			<<", mvTrack_cnt[CAM1].size() = (not 150)"<<mvTrack_cnt[CAM0].size()
			<<", mvTrack_cnt[CAM1].size() = (58)"<<mvTrack_cnt[CAM1].size()<<std::endl
			<<"----------------------------------"<<std::endl;
*/
		/////// Step 2. CAM0: new feature candidates
		setMask(CAM0);//only for CAM0		
		
		int n_max_cnt = MAX_CNT - static_cast<int>(mvCur_pts[CAM0].size());
		std::cout<<"mvCur_pts[CAM0].size() = "<<mvCur_pts[CAM0].size()<<", n_max_cnt = "<<n_max_cnt<<std::endl;
		if (n_max_cnt > 0)
		{
			if (!cur_img[CAM0].empty())
				cv::goodFeaturesToTrack(cur_img[CAM0],
										mvN_pts[CAM0],
										n_max_cnt,
										0.01,
										MIN_DIST,
										mMask[CAM0]);
			else
				std::cout<<"[execution_tic > 0][if (!cur_img[CAM0].empty())] fails"<<std::endl;
			/// stdcout
/* //xin 20230818
			std::cout<<"-------------------------------- Acquisition: execution_tic = "<<execution_tic<<".2 (AFTER goodFeaturesToTracking CAM0(t));"<<std::endl
				<<"mvN_pts[CAM0].size() = (some small values)"<<mvN_pts[CAM0].size()
				<<", mvN_pts[CAM1].size() = (0)"<<mvN_pts[CAM1].size()<<std::endl
				<<", mvCur_pts[CAM0].size() = (not 150 + some small values)"<<mvCur_pts[CAM0].size() 
				<<", mvCur_pts[CAM1].size() = (no changes)"<<mvCur_pts[CAM1].size()<<std::endl
				<<", status[CAM0].size() = (no changes)"<<status[CAM0].size()
				<<", status[CAM1].size() = (no changes)"<<status[CAM1].size()<<std::endl
				<<", mvIds[CAM0].size() = (not 150 + some small values)"<<mvIds[CAM0].size()
				<<", mvIds[CAM1].size() = (no changes)"<<mvIds[CAM1].size()<<std::endl
				<<", mvTrack_cnt[CAM1].size() = (not 150 + some small values)"<<mvTrack_cnt[CAM0].size()
				<<", mvTrack_cnt[CAM1].size() = (no changes)"<<mvTrack_cnt[CAM1].size()<<std::endl
				<<"----------------------------------"<<std::endl;
			for (size_t camid = 0; camid < 2; camid++)
			{
				std::cout<<"CAM"<<camid<<":";
				size_t lcv = 0;
				if (mvCur_pts[camid].size() == 0) std::cout<<"empty"<<std::endl; else std::cout<<std::endl;
				for (auto pt : mvCur_pts[camid])
				{
					std::cout<<lcv<<" : (un_pt.x, un_pt.y) = ("<<pt.x<<", "<<pt.y<<")    kpt_id = "<<mvIds[camid][lcv]<<std::endl;
					lcv++;
				}
				std::cout<<std::endl;
			}
*/			
			/////// Step 3. CAM1: new feature candidates
			status[CAM1].clear();
			err[CAM1].clear();
			if (!cur_img[CAM1].empty())
			{
				if (mvN_pts[CAM0].size() > 0)
				{
					if (mvN_pts[CAM1].size() > 0) {std::cout<<"mvN_pts[CAM1] should be cleared before usage !"<<std::endl; exit(-1);}
					cv::calcOpticalFlowPyrLK(cur_img[CAM0],
											 cur_img[CAM1],
											 mvN_pts[CAM0],
											 mvN_pts[CAM1],
											 status[CAM1],
											 err[CAM1],
											 cv::Size(21,21),
											 3);
					for (size_t i = 0; i < mvN_pts[CAM1].size(); i++)
						if (status[CAM1][i] && !inBorder(mvN_pts[CAM1][i]))
							status[CAM1][i] = 0;
					if (mvN_pts[CAM0].size() != mvN_pts[CAM1].size()) {std::cout<<"mvN_pts[CAM0].size() != mvN_pts[CAM1].size()"<<std::endl;exit(-1);}
					for (size_t i = 0; i < mvN_pts[CAM0].size(); i++)
					{
						mvCur_pts[CAM0].push_back(mvN_pts[CAM0][i]);
						mvIds[CAM0].push_back(miN_id);
						mvTrack_cnt[CAM0].push_back(1);
						if (status[CAM1][i] == 1 && mvCur_pts[CAM1].size() < MAX_CNT)
						{
							mvCur_pts[CAM1].push_back(mvN_pts[CAM1][i]);
							mvIds[CAM1].push_back(miN_id);
							mvTrack_cnt[CAM1].push_back(1);
						}
						miN_id++;
					}
				}
			}
			/// stdcout
/* //xin 20230818
			std::cout<<"-------------------------------- Acquisition: execution_tic = "<<execution_tic<<".3 & .4 (AFTER calcOpticalFlowPyrLKing CAM0(t)->CAM1(t) & addPoints of BOTH cameras);"<<std::endl
				<<"mvN_pts[CAM0].size() = (no changes)"<<mvN_pts[CAM0].size()
				<<", mvN_pts[CAM1].size() = (some small values #1)"<<mvN_pts[CAM1].size()<<std::endl
				<<", mvCur_pts[CAM0].size() = (no changes)"<<mvCur_pts[CAM0].size() 
				<<", mvCur_pts[CAM1].size() = (previous values + some small values #1)"<<mvCur_pts[CAM1].size()<<std::endl
				<<", status[CAM0].size() = (no changes)"<<status[CAM0].size()
				<<", status[CAM1].size() = (mvN_pts[CAM0.size()])"<<status[CAM1].size()<<std::endl
				<<", mvIds[CAM0].size() = (no changes)"<<mvIds[CAM0].size()
				<<", mvIds[CAM1].size() = (no changes)"<<mvIds[CAM1].size()<<std::endl
				<<", mvTrack_cnt[CAM1].size() = (no changes)"<<mvTrack_cnt[CAM0].size()
				<<", mvTrack_cnt[CAM1].size() = (no changes + some small values #1)"<<mvTrack_cnt[CAM1].size()<<std::endl
				<<"----------------------------------"<<std::endl;
			for (size_t camid = 0; camid < 2; camid++)
			{
				std::cout<<"CAM"<<camid<<":";
				size_t lcv = 0;
				if (mvCur_pts[camid].size() == 0) std::cout<<"empty"<<std::endl; else std::cout<<std::endl;
				for (auto pt : mvCur_pts[camid])
				{
					std::cout<<lcv<<" : (un_pt.x, un_pt.y) = ("<<pt.x<<", "<<pt.y<<")    kpt_id = "<<mvIds[camid][lcv]<<std::endl;
					lcv++;
				}
				std::cout<<std::endl;
			}
*/			
			/////// step 5 check if points between CAM0(t) and CAM1(t) constrained by epipole
			filterPoints();
			/// stdcout
/* //xin 20230818
			std::cout<<"-------------------------------- Acquisition: execution_tic = "<<execution_tic<<".5; (AFTER filterPoints)"<<std::endl
				<<"mvN_pts[CAM0].size() = (no changes)"<<mvN_pts[CAM0].size()
				<<", mvN_pts[CAM1].size() = (no changes)"<<mvN_pts[CAM1].size()<<std::endl
				<<", mvCur_pts[CAM0].size() = (no changes)"<<mvCur_pts[CAM0].size() 
				<<", mvCur_pts[CAM1].size() = (altered)"<<mvCur_pts[CAM1].size()<<std::endl
				<<", status[CAM0].size() = (no changes)"<<status[CAM0].size()
				<<", status[CAM1].size() = (no changes)"<<status[CAM1].size()<<std::endl
				<<", mvIds[CAM0].size() = (no changes)"<<mvIds[CAM0].size()
				<<", mvIds[CAM1].size() = (altered)"<<mvIds[CAM1].size()<<std::endl
				<<", mvTrack_cnt[CAM1].size() = (no changes)"<<mvTrack_cnt[CAM0].size()
				<<", mvTrack_cnt[CAM1].size() = (altered)"<<mvTrack_cnt[CAM1].size()<<std::endl
				<<"----------------------------------"<<std::endl;
			for (size_t camid = CAM1; camid < 2; camid++)//xin 20230816
			{
				std::cout<<"CAM"<<camid<<":";
				size_t lcv = 0;
				if (mvCur_pts[camid].size() == 0) std::cout<<"empty"<<std::endl; else std::cout<<std::endl;
				for (auto pt : mvCur_pts[camid])
				{
					std::cout<<lcv<<" : (filtered.x, filtered.y) = ("<<pt.x<<", "<<pt.y<<")    kpt_id = "<<mvIds[camid][lcv]<<std::endl;
					lcv++;
				}
				std::cout<<std::endl;
			}
			for (auto pt1 : mvIds[CAM1])
				std::cout<<pt1<<", ";
			std::cout<<std::endl;

			if (lm_to_remove.size() > 0)
			{
				std::cout<<"lm_to_remove.size() = "<<lm_to_remove.size()<<". Remove:"<<std::endl;
				for (auto kpt_id : lm_to_remove)
					std::cout <<kpt_id<< ", "<<std::endl;
			}
			else
				std::cout<<"lm_to_remove.size() = 0"<<std::endl;
			std::cout<<std::endl<<std::endl;
*/
		}//if (n_max_cnt > 0)


		// Step 6 TODO: should we also check fundamental matrix between CAM1(t-1) and CAM1(t)? between CAM0(t-1) and CAM0(t)?

	}//if (execution_tic == 0) else
	
	// Step 7 time evolves
	for (size_t camid = 0; camid < 2; camid++)
	{
		prev_img[camid] = cur_img[camid];
		mvPrev_pts[camid] = mvCur_pts[camid];
		mvPrev_un_pts[camid] = mvCur_un_pts[camid];//TODO: 'mvPrev_un_pts' is unused as of 2023-6-1
	}


	// Step 8. install packet for delivering left and right keypoints to svs
	for (size_t camid = 0; camid < 2; camid++)
	{
		size_t lcv = 0;
/*//xin 20230802
		std::cout<<"------------------ Acquisition::camid = "<<camid<<" : muCnt_un_pts["<<camid<<"].size() = "<<mvCur_pts[camid].size()
			<<",  mvIds["<<camid<<"].size() = "<<mvIds[camid].size()
			<<" -------------------"<<std::endl;
*/
		for (auto pt : mvCur_pts[camid])//dummy only used for visualization, i.e. gps-gse
//			for (auto pt : mvCur_un_pts[camid])//TODO: interface for vins_mono
		{
/*//xin 20230802
			std::cout<<lcv<<" : (un_pt.x, un_pt.y) = ("<<pt.x<<", "<<pt.y<<")    kpt_id = "<<mvIds[camid][lcv]<<std::endl;
*/
			memcpy(&result->mfCur_un_pts_x[camid][lcv],			&pt.x,				sizeof(float));
			memcpy(&result->mfCur_un_pts_y[camid][lcv],			&pt.y,				sizeof(float));
			memcpy(&result->mvIds[camid][lcv],					&mvIds[camid][lcv], sizeof(size_t));
			memcpy(&mpBuff_fea[0].mfCur_un_pts_x[camid][lcv],	&pt.x,				sizeof(float));//xin 0505//TODO: install packet for fifo_fea// only for visualization, i.e., gps-gse
			memcpy(&mpBuff_fea[0].mfCur_un_pts_y[camid][lcv],	&pt.y,				sizeof(float));//xin 0505//TODO: install packet for fifo_fea// only for visualization, i.e., gps-gse
			memcpy(&mpBuff_fea[0].mvIds[camid][lcv],			&mvIds[camid][lcv], sizeof(size_t));
			
			lcv++;
		}
/*//xin 20230802
		std::cout<<std::endl;
*/
		result->muCnt_un_pts[camid] = mvCur_pts[camid].size();
		mpBuff_fea[0].muCnt_un_pts[camid] = mvCur_pts[camid].size();//xin 0505
		mpBuff_fea[0].mvTimestamp[camid] = mpBuff_cam[0].mvTimestamp[camid];//xin 0505
	}
/*//xin 20230802
	std::cout<<std::endl;
*/

	if (mvPrev_un_pts[CAM0].size() > 0)//TODO: if(result->magnitude > THRESH_STRONG)
		result->success = 1;
	else
		result->success = 0;

}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
void Acquisition::setMask(int camid)
{
	mMask[camid] = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
	std::vector<std::pair<int,							// mvTrack_cnt[camid][i]
		                  std::pair<cv::Point2f,		// mvCur_pts[camid][i]
		                            int>>> cnt_pts_id;	// mvIds[camid][i]

    for (size_t i = 0; i < mvCur_pts[camid].size(); i++)
	{
        cnt_pts_id.push_back(std::make_pair(mvTrack_cnt[camid][i], 
					                        std::make_pair(mvCur_pts[camid][i],
						                                   mvIds[camid][i])));
	}

	std::sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, 
				                                  const std::pair<int, std::pair<cv::Point2f, int>> &b)
         {
			return a.first > b.first;
         });

    mvCur_pts[camid].clear();
    mvIds[camid].clear();
    mvTrack_cnt[camid].clear();

    for (auto &it : cnt_pts_id)
        if (mMask[camid].at<uchar>(it.second.first) == 255)
        {
            mvCur_pts[camid].push_back(it.second.first);
            mvIds[camid].push_back(it.second.second);
            mvTrack_cnt[camid].push_back(it.first);
            cv::circle(mMask[camid],    //input image
					   it.second.first, //center of the circle
					   MIN_DIST,        //radius
					   0,				//color: black
					   -1);             //thickness: negative value for 'filled'
        }
	
	//--------------------  for easy debugging ----------------------------
	std::vector<std::pair<int,							// mvIds[camid][i]
						  std::pair<cv::Point2f,		// mvCur_pts[camid][i]
									int>>> id_pts_cnt;	// mvTrack_cnt[i]
    for (size_t i = 0; i < mvCur_pts[camid].size(); i++)
	{
        id_pts_cnt.push_back(std::make_pair(mvIds[camid][i], 
					                        std::make_pair(mvCur_pts[camid][i],
						                                   mvTrack_cnt[camid][i])));
	}

	std::sort(id_pts_cnt.begin(), id_pts_cnt.end(), [](const std::pair<int, std::pair<cv::Point2f, int>> &a, 
				                                  const std::pair<int, std::pair<cv::Point2f, int>> &b)
         {
			return a.first < b.first;
         });

    mvCur_pts[camid].clear();
    mvIds[camid].clear();
    mvTrack_cnt[camid].clear();
	
	for (auto &it : id_pts_cnt)
	{
		mvCur_pts[camid].push_back(it.second.first);
		mvIds[camid].push_back(it.first);
		mvTrack_cnt[camid].push_back(it.second.second);
	}
}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
bool Acquisition::updateID(unsigned int i)
{
	if (i < mvIds[CAM0].size())
	{
		if (mvIds[CAM0][i] == -1)
			mvIds[CAM0][i] = miN_id++;
		return true;
	}
	else
		return false;
}

/*----------------------------------------------------------------------------------------------*/
bool Acquisition::updateID(unsigned int i, const std::vector<uchar>& status_cam1)
{
	if (i < mvIds[CAM0].size())
	{
		if (mvIds[CAM0][i] == -1)
		{
			mvIds[CAM0][i] = miN_id++;
			if (status_cam1.size() > 0 && status_cam1[i] == 1)
				mvIds_CAM1_tmp.emplace_back(mvIds[CAM0][i]);
		}
		return true;
	}
	else
		return false;
}

/*----------------------------------------------------------------------------------------------*/
void Acquisition::addPoints(int camid)
{
	for (auto &p : mvN_pts[camid])
	{
		mvCur_pts[camid].push_back(p);
		if (camid == 0)
			mvIds[camid].push_back(-1); //not yet assigned an id
		mvTrack_cnt[camid].push_back(1);
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 *
 * */
void Acquisition::Acquire()
{
	IncStartTic();

	switch(request.type)
	{
		case ACQ_TYPE_STRONG:
			doAcqStrong(request.sv);
			break;
		default:
			doAcqStrong(request.sv);
			fprintf(stderr, "invalid request type received: %i\n", request.type);
	}

	IncStopTic();
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 * Import:
 * */
void Acquisition::Import()
{
	ssize_t bread;

	/* First wait for a request */
	bread = read(SVS_2_ACQ_P[READ], &request, sizeof(Acq_Command_S)); if (bread != sizeof(Acq_Command_S)) std::cout<<"Acquisition::Import():read(SVS_2_ACQ_P[READ], &request, sizeof(Acq_Command_S)) failed: "<<strerror(errno)<<std::endl;
	memcpy(&results[request.sv],&request,sizeof(Acq_Command_S));

	/* Read a packet in */
	bread = read(FIFO_IMU_2_ACQ_P[READ], &packet_imu, sizeof(ms_packet_imu)); if (bread != sizeof(ms_packet_imu)) std::cout<<"Acquisition::Import():read(FIFO_IMU_2_ACQ_P[READ], &packet_imu, sizeof(ms_packet_imu)) failed: "<<strerror(errno)<<std::endl;
	bread = read(FIFO_CAM_2_ACQ_P[READ], &packet_cam, sizeof(ms_packet_cam)); if (bread != sizeof(ms_packet_cam)) std::cout<<"Acquisition::Import():read(FIFO_CAM_2_ACQ_P[READ], &packet_cam, sizeof(ms_packet_cam)) failed: "<<strerror(errno)<<std::endl;
	memcpy(&mpBuff_cam[0], &packet_cam.mData[0], sizeof(CamMeasurement));

}
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/
/*!
 * Export:
 * */
void Acquisition::Export()
{
	ssize_t bwrite;

	/* Write result to the tracking task */
	results[request.sv].count = request.count;
	bwrite = write(ACQ_2_SVS_P[WRITE], &results[request.sv], sizeof(Acq_Command_S));if (bwrite != sizeof(Acq_Command_S)) std::cout<<"Acquisition::Export():write(ACQ_2_SVS_P[WRITE], &results[request.sv], sizeof(Acq_Command_S)) failed: "<<strerror(errno)<<std::endl;

	bwrite = write(ACQ_2_FIFO_FEA_P[WRITE], &mpBuff_fea[0], sizeof(FeaMeasurement));if (bwrite != sizeof(FeaMeasurement)) std::cout<<"acquisition.cpp::Export(): write(ACQ_2_FIFO_FEA_P) failed: "<<strerror(errno)<<std::endl;
}
/*----------------------------------------------------------------------------------------------*/
