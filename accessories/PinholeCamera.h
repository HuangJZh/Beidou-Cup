







#ifndef PINHOLECAMERA_H_
#define PINHOLECAMERA_H_
#include "includes.h"

class PinholeCamera
{
public:
	class Parameters
	{
    public:
        Parameters();
        Parameters(//const std::string& cameraName,
                   //int w, int h,
                   double k1, double k2, double p1, double p2,
                   double fx, double fy, double cx, double cy);

        double& k1(void);
        double& k2(void);
        double& p1(void);
        double& p2(void);
        double& fx(void);
        double& fy(void);
        double& cx(void);
        double& cy(void);

 //       double xi(void) const;
        double k1(void) const;
        double k2(void) const;
        double p1(void) const;
        double p2(void) const;
        double fx(void) const;
        double fy(void) const;
        double cx(void) const;
        double cy(void) const;










    private:
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;
	};

	PinholeCamera();

    /**
    * \brief Constructor from the projection model parameters
    */
    PinholeCamera(//xin//const std::string& cameraName,
                  //xin//int imageWidth, int imageHeight,
                  double k1, double k2, double p1, double p2,
                  double fx, double fy, double cx, double cy);
    
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	// Lift points from the image plane to the projective space
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    //%output P





















    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;






















private:
    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

#endif
