#ifndef GAZEBO_CONVERSIONS_H_
#define GAZEBO_CONVERSIONS_H_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
namespace kenv {
	inline Eigen::Vector3d toEigen(math::Vector3 const &gz_v)
	{
		Eigen::Vector3d eigen_v;
		eigen_v << gz_v.x, gz_v.y, gz_v.z;
		return eigen_v;
	}
	inline Eigen::Affine3d toEigen(math::Pose const &gz_tf)
	{
		math::Pose p(gz_tf);
		Eigen::Affine3d eigen_tf = Eigen::Affine3d::Identity();
		eigen_tf.linear() << p.rot.GetXAxis().x, p.rot.GetYAxis().x, p.rot.GetZAxis().x,
						     p.rot.GetXAxis().y, p.rot.GetYAxis().y, p.rot.GetZAxis().y,
						     p.rot.GetXAxis().z, p.rot.GetYAxis().z, p.rot.GetZAxis().z;
		eigen_tf.translation() << p.pos.x, p.pos.y, p.pos.z;
		return eigen_tf;
	}
	template <typename Derived>
	inline math::Vector3 toGZ(Derived const &eigen_v)
	{
		BOOST_STATIC_ASSERT(Derived::IsVectorAtCompileTime);
		if(eigen_v.size() == 3) {
			return math::Vector3(eigen_v[0], eigen_v[1], eigen_v[2]);
		} else if (eigen_v.size() == 4) {
			return math::Vector4(eigen_v[0], eigen_v[1], eigen_v[2], eigen_v[3]);
		} else {
			throw std::invalid_argument(boost::str(
				boost::format("Expected 3 or 4 element vector; got %d element(s).") 
					% eigen_v.size()));
		}
	}
	template <typename Derived>
	inline math::Pose toGZ(Eigen::Transform<Derived, 3, Eigen::Affine> const &tf)
	{
		math::Pose p;
		double Qx;
		double Qy;
		double Qz;
		double Qw;
		Eigen::Quaternion<double> q(tf.rotation());
		Eigen::Vector3d t(tf.translation());
		Qx = q.x();
		Qy = q.y();
		Qz = q.z();
		Qw = q.w();
		math::Quaternion rot(Qx, Qy, Qz, Qw);
		math::Vector3 pos(t.x(), t.y(), t.z());
		return math::Pose(pos, rot);


	// 	double T = tf.matrix()(0,0) + tf.matrix()(1,1) + tf.matrix()(2,2) + 1;
	// 	if(T>0){
	// 		double S = .5/sqrt(T);
	// 		Qw = .25/S;
	// 		Qx = (tf.matrix()(2,1)-tf.matrix()(1,2))*S;
	// 		Qy = (tf.matrix()(0,2)-tf.matrix()(2,0))*S;
	// 		Qz = (tf.matrix()(1,0)-tf.matrix()(0,1))*S;


	// 	} else {
	// 		double diag0 = tf.matrix()(0,0);
	// 		double diag1 = tf.matrix()(1,1);
	// 		double diag2 = tf.matrix()(2,2);

	// 		if(diag0 > diag1 && diag0 > diag2){
	// 			double S = sqrt(1 + tf.matrix()(0,0) - tf.matrix()(1,1) - tf.matrix()(2,2))*2;
	// 			Qx = .5/S;
	// 			Qy = (tf.matrix()(0,1)+tf.matrix()(1,0))/S;
	// 			Qz = (tf.matrix()(0,2)+tf.matrix()(2,0))/S;
	// 			Qw = (tf.matrix()(1,2)+tf.matrix()(2,1))/S;
	// 		} else if (diag1> diag0 && diag1 > diag2){
	// 			double S = sqrt(1 + tf.matrix()(1,1) - tf.matrix()(0,0) - tf.matrix()(2,2))*2;
	// 			Qy = .5/S;
	// 			Qx = (tf.matrix()(0,1)+tf.matrix()(1,0))/S;
	// 			Qw = (tf.matrix()(0,2)+tf.matrix()(2,0))/S;
	// 			Qz = (tf.matrix()(1,2)+tf.matrix()(2,1))/S;
	// 		} else {
	// 			double S = sqrt(1 + tf.matrix()(2,2) - tf.matrix()(1,1) - tf.matrix()(0,0))*2;
	// 			Qz = .5/S;
	// 			Qw = (tf.matrix()(0,1)+tf.matrix()(1,0))/S;
	// 			Qx = (tf.matrix()(0,2)+tf.matrix()(2,0))/S;
	// 			Qy = (tf.matrix()(1,2)+tf.matrix()(2,1))/S;
	// 		}
	// 	}
	// 	math::Quaternion rot(Qx, Qy, Qz, Qw);
	// 	math::Vector3 pos(tf.matrix()(0,3), tf.matrix()(1,3), tf.matrix()(2,3));
	// 	return math::Pose(pos, rot);
	}

	
}
#endif
