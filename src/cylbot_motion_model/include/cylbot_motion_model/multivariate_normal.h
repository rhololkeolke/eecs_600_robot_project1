#ifndef CYLBOT_MOTION_MODEL__MULTIVARIATE_NORMAL_H_
#define CYLBOT_MOTION_MODEL__MULTIVARIATE_NORMAL_H_

#include <Eigen/Dense>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

namespace Eigen {
	namespace internal {
		template<typename Scalar> struct scalar_normal_dist_op
								  {
									  static boost::mt19937 rng; // uniform pseudo-random number generator
									  mutable boost::normal_distribution<Scalar> norm; // gaussian distribution

									  EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

									  template<typename Index> inline const Scalar operator() (Index, Index=0) const {return norm(rng); }
		};

		template<typename Scalar> boost::mt19937 scalar_normal_dist_op<Scalar>::rng;

		template<typename Scalar> struct functor_traits<scalar_normal_dist_op<Scalar> >
		{
			enum { Cost = 50*NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false };
		};
	}
}

#endif
