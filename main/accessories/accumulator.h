#ifndef ACCUMULATOR_H_
#define ACCUMULATOR_H_

//#include "faim_assert.h"
#include <Eigen/Core>

template <typename Scalar = double>
class DenseAccumulator
{
	using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

public:
	inline const MatrixX& getH() const { return H; }
    inline const VectorX& getB() const { return b; }

    inline MatrixX& getH() { return H; }
    inline VectorX& getB() { return b; }

    inline void setup_solver(){};//TODO: strange. nothing to do?
    
	inline VectorX Hdiagonal() const { return H.diagonal(); }
    
	// inline VectorX solve() const { return H.ldlt().solve(b); }
    inline VectorX solve(const VectorX* diagonal) const
	{
		if (diagonal == nullptr)
		    return H.ldlt().solve(b);
		else
		{
		    MatrixX HH = H;
		    HH.diagonal() += *diagonal;
		    return HH.ldlt().solve(b);
		}
    }

	inline void reset(int opt_size)
	{
		H.setZero(opt_size, opt_size);
		b.setZero(opt_size);
    }

	template <int ROWS, int COLS, typename Derived>
    inline void addH(      int                         i,
			               int                         j,
					 const Eigen::MatrixBase<Derived>& data)
	{
//		FAIM_ASSERT_STREAM(i >= 0, "i " << i);
//		FAIM_ASSERT_STREAM(j >= 0, "j " << j);

//		FAIM_ASSERT_STREAM(i + ROWS <= H.cols(), "i " << i << " ROWS " << ROWS << " H.rows() " << H.rows());
//		FAIM_ASSERT_STREAM(j + COLS <= H.rows(), "j " << j << " COLS " << COLS << " H.cols() " << H.cols());

		H.template block<ROWS, COLS>(i, j) += data;
    }

	inline void join (const DenseAccumulator<Scalar>& other)
	{
		H += other.H;
		b += other.b;
	}
	
	template <int ROWS, typename Derived>
    inline void addB(      int                         i,
			         const Eigen::MatrixBase<Derived>& data)
	{
//		FAIM_ASSERT_STREAM(i >= 0, "i " << i);

//		FAIM_ASSERT_STREAM(i + ROWS <= H.cols(), "i " << i << " ROWS " << ROWS << " H.rows() " << H.rows());

		b.template segment<ROWS>(i) += data;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	MatrixX H;
	VectorX b;
};
#endif
