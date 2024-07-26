#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H



#include <map>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "se3.hpp"
#include "sim3.hpp"
#include "faim_assert.h"

//#include "accessories/calibration.h"


namespace Eigen {
template <typename T> using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;
template <typename K, typename V> using aligned_map = std::map<K, V, std::less<K>,
		 														Eigen::aligned_allocator<std::pair<K const, V>>>;
template <typename K, typename V> using aligned_unordered_map = std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
		 														Eigen::aligned_allocator<std::pair<K const, V>>>;
}// namespace Eigen


namespace Sophus {
// Note on the use of const_cast in the following functions: The output
// parameter is only marked 'const' to make the C++ compiler accept a temporary
// expression here. These functions use const_cast it, so constness isn't
// honored here. See:
// https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html

/// @brief Right Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides a Jacobian that approximates the sum
/// under expmap with a right multiplication of expmap for small \f$ \epsilon
/// \f$.  Can be used to compute:  \f$ \exp(\phi + \epsilon) \approx \exp(\phi)
/// \exp(J_{\phi} \epsilon)\f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianSO3(const Eigen::MatrixBase<Derived1> &phi,
                             const Eigen::MatrixBase<Derived2> &J_phi)
{
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
	EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

	using Scalar = typename Derived1::Scalar;

	Eigen::MatrixBase<Derived2> &J = const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

	Scalar phi_norm2 = phi.squaredNorm();
	Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
	Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

	J.setIdentity();

	if (phi_norm2 > Sophus::Constants<Scalar>::epsilon())
	{
		Scalar phi_norm = std::sqrt(phi_norm2);
		Scalar phi_norm3 = phi_norm2 * phi_norm;

		J -= phi_hat * (1 - std::cos(phi_norm)) / phi_norm2;
		J += phi_hat2 * (phi_norm - std::sin(phi_norm)) / phi_norm3;
	} else {
		// Taylor expansion around 0
		J -= phi_hat / 2;
		J += phi_hat2 / 6;
	}
}


/// @brief Right Inverse Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides an inverse Jacobian that approximates
/// the logmap of the right multiplication of expmap of the arguments with a sum
/// for small \f$ \epsilon \f$.  Can be used to compute:  \f$ \log
/// (\exp(\phi) \exp(\epsilon)) \approx \phi + J_{\phi} \epsilon\f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                                const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J = const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    Scalar phi_norm2 = phi.squaredNorm();
    Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
    Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

    J.setIdentity();
    J += phi_hat / 2;

    if (phi_norm2 > Sophus::Constants<Scalar>::epsilon())
	{
		Scalar phi_norm = std::sqrt(phi_norm2);

		// We require that the angle is in range [0, pi]. We check if we are close
		// to pi and apply a Taylor expansion to scalar multiplier of phi_hat2.
		// Technically, log(exp(phi)exp(epsilon)) is not continuous / differentiable
		// at phi=pi, but we still aim to return a reasonable value for all valid
		// inputs.
		FAIM_ASSERT(phi_norm <= M_PI + Sophus::Constants<Scalar>::epsilon());

		if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt())
		    // regular case for range (0,pi)
		    J += phi_hat2 * (1 / phi_norm2 - (1 + std::cos(phi_norm)) / (2 * phi_norm * std::sin(phi_norm)));
		else
		    // 0th-order Taylor expansion around pi
		    J += phi_hat2 / (M_PI * M_PI);
    }
	else
		// Taylor expansion around 0
		J += phi_hat2 / 12;
}


/// @brief Left Inverse Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides an inverse Jacobian that approximates
/// the logmap of the left multiplication of expmap of the arguments with a sum
/// for small \f$ \epsilon \f$.  Can be used to compute:  \f$ \log
/// (\exp(\epsilon) \exp(\phi)) \approx \phi + J_{\phi} \epsilon\f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void leftJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                               const Eigen::MatrixBase<Derived2> &J_phi)
{
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
	EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

	using Scalar = typename Derived1::Scalar;

	Eigen::MatrixBase<Derived2> &J = const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

	Scalar phi_norm2 = phi.squaredNorm();
	Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
	Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

	J.setIdentity();
	J -= phi_hat / 2;

	if (phi_norm2 > Sophus::Constants<Scalar>::epsilon())
	{
		Scalar phi_norm = std::sqrt(phi_norm2);

		// We require that the angle is in range [0, pi]. We check if we are close
		// to pi and apply a Taylor expansion to scalar multiplier of phi_hat2.
		// Technically, log(exp(phi)exp(epsilon)) is not continuous / differentiable
		// at phi=pi, but we still aim to return a reasonable value for all valid
		// inputs.
		FAIM_ASSERT(phi_norm <= M_PI + Sophus::Constants<Scalar>::epsilon());

		if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt())
		{
			// regular case for range (0,pi)
			J += phi_hat2 * (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
											   (2 * phi_norm * std::sin(phi_norm)));
		} else {
			// 0th-order Taylor expansion around pi
			J += phi_hat2 / (M_PI * M_PI);
		}
	} else {
		// Taylor expansion around 0
		J += phi_hat2 / 12;
	}
}

}// namespace Sophus

inline void computeEssential(const Sophus::SE3d& T_0_1, Eigen::Matrix4d& E)
//inline void computeEssential(const Sophus::SE3d& T_0_1, Eigen::Matrix<double, 4, 4>& E)
{
	E.setZero();
	const Eigen::Vector3d t_0_1 = T_0_1.translation();
	const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

	E.topLeftCorner<3, 3>() = Sophus::SO3d::hat(t_0_1.normalized()) * R_0_1;
}

#endif
