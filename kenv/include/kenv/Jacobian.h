#ifndef KENV_JACOBIAN_H_
#define KENV_JACOBIAN_H_

#include <Eigen/Core>
#include <boost/multi_array.hpp>

namespace kenv {

	/**
	 * Wrapper for a jacobian calculated by OpenRAVE.  Provides functionality such as transpose and psuedo-inverse
	 */
	class Jacobian {

	public:
		/**
		 * Shared Ptr
		 */
		typedef boost::shared_ptr<Jacobian> Ptr;

		/**
		 * Const Shared Ptr
		 */
		typedef boost::shared_ptr<Jacobian const> ConstPtr;

		/**
		 * Constructor
		 *
		 * @param jacobian_trans A multi-dimensional array representing the translational jacobian
		 * @param jacobian_rot A multi-dimensional array representing the axis angle jacobian
		 */
		Jacobian(const boost::multi_array<double, 2> &jacobian_trans,
				 const boost::multi_array<double, 2> &jacobian_rot);

		/**
		 * Returns the Eigen representation of the jacobian
		 *
		 * @return The eigen representation of the jacobian
		 */
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> asEigen() const { return _jacobian; }

		/**
		 * Calculates and returns the transpose of the jacobian
		 *
		 * @return The jacobian transpose
		 */
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> transpose() const { return _jacobian.transpose(); }

		/**
		 * Calculates the psuedo-inverse of the jacobian
		 *
		 * @return The jacobian psuedo-inverse
		 */
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> pinv() const;

		/**
		 * Calculates the nullspace projector of the jacobian
		 *
		 * @return The jacobian nullspace projector
		 */
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> nullspace_projector() const;

	private:
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _jacobian;
	};

}


#endif
