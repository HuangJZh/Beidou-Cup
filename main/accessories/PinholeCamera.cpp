#include "PinholeCamera.h"














PinholeCamera::Parameters::Parameters()
/* : Camera::Parameters(PINHOLE)
 , m_k1(0.0)
 , m_k2(0.0)
 , m_p1(0.0)
 , m_p2(0.0)
 , m_fx(0.0)
 , m_fy(0.0)
 , m_cx(0.0)
 , m_cy(0.0)*/
{
	m_k1 = 0.0;
	m_k2 = 0.0;
	m_p1 = 0.0;
	m_p2 = 0.0;
	m_fx = 0.0;
	m_fy = 0.0;
	m_cx = 0.0;
	m_cy = 0.0;
}

PinholeCamera::Parameters::Parameters(//xin//const std::string& cameraName,
                                      //xin//int w, int h,
                                      double k1, double k2,
                                      double p1, double p2,
                                      double fx, double fy,
                                      double cx, double cy)
/* : Camera::Parameters(PINHOLE, cameraName, w, h)
 , m_k1(k1)
 , m_k2(k2)
 , m_p1(p1)
 , m_p2(p2)
 , m_fx(fx)
 , m_fy(fy)
 , m_cx(cx)
 , m_cy(cy)*/
{
	m_k1 = k1;
	m_k2 = k2;
	m_p1 = p1;
	m_p2 = p2;
	m_fx = fx;
	m_fy = fy;
	m_cx = cx;
	m_cy = cy;
}

double&
PinholeCamera::Parameters::k1(void)
{
    return m_k1;
}

double&
PinholeCamera::Parameters::k2(void)
{
    return m_k2;
}

double&
PinholeCamera::Parameters::p1(void)
{
    return m_p1;
}

double&
PinholeCamera::Parameters::p2(void)
{
    return m_p2;
}

double&
PinholeCamera::Parameters::fx(void)
{
    return m_fx;
}

double&
PinholeCamera::Parameters::fy(void)
{
    return m_fy;
}

double&
PinholeCamera::Parameters::cx(void)
{
    return m_cx;
}

double&
PinholeCamera::Parameters::cy(void)
{
    return m_cy;
}

double
PinholeCamera::Parameters::k1(void) const
{
    return m_k1;
}

double
PinholeCamera::Parameters::k2(void) const
{
    return m_k2;
}

double
PinholeCamera::Parameters::p1(void) const
{
    return m_p1;
}

double
PinholeCamera::Parameters::p2(void) const
{
    return m_p2;
}

double
PinholeCamera::Parameters::fx(void) const
{
    return m_fx;
}

double
PinholeCamera::Parameters::fy(void) const
{
    return m_fy;
}

double
PinholeCamera::Parameters::cx(void) const
{
    return m_cx;
}

double
PinholeCamera::Parameters::cy(void) const
{
    return m_cy;
}























































































































PinholeCamera::PinholeCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
 , m_noDistortion(true)
{

}

PinholeCamera::PinholeCamera(//xin//const std::string& cameraName,
                             //xin//int imageWidth, int imageHeight,
                             double k1, double k2, double p1, double p2,
                             double fx, double fy, double cx, double cy)
 : mParameters(//xin//cameraName, imageWidth, imageHeight,
               k1, k2, p1, p2, fx, fy, cx, cy)
{
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}


















































































































































/**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void
PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    double mx_d, my_d, mx_u, my_u;
//    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
//    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

/*	std::cout<<"zzzzzzzzzzzzzzzzzzzz Acquisition: liftProjective(): p(0) = "<<p(0)<<", p(1) = "<<p(1)<<" <><<><><><><>"<<std::endl;
	std::cout<<"zzzzzzzzzzzzzzzzzzzz Acquisition: liftProjective(): m_inv_K11 = "<<m_inv_K11<<" <><<><><><><>"<<std::endl;
	std::cout<<"zzzzzzzzzzzzzzzzzzzz Acquisition: liftProjective(): m_inv_K13 = "<<m_inv_K13<<" <><<><><><><>"<<std::endl;
	std::cout<<"zzzzzzzzzzzzzzzzzzzz Acquisition: liftProjective(): m_inv_K22 = "<<m_inv_K22<<" <><<><><><><>"<<std::endl;
	std::cout<<"zzzzzzzzzzzzzzzzzzzz Acquisition: liftProjective(): m_inv_K23 = "<<m_inv_K23<<" <><<><><><><>"<<std::endl;
*/    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;

//	std::cout<<"zzzzzzzzzzzzzzzzzzzz Acquisition: liftProjective():m_noDistortion = "<<m_noDistortion<<" <><<><><><><>"<<std::endl;
    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
 /*       if (0)
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        else
        {
*/            // Recursive distortion model
            int n = 8; //TODO TODO: to make it not a magic number
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i)
            {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
//        }
    }

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}































































































































/**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}





















