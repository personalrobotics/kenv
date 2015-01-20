#include <kenv/Jacobian.h>
#include <kenv/Environment.h>
#include <iostream>
#include <Eigen/LU>

#include <boost/format.hpp>

using namespace kenv;

Jacobian::Jacobian(const boost::multi_array<double, 2> &jacobian_trans,
                   const boost::multi_array<double, 2> &jacobian_rot) {

    unsigned int rows_trans = jacobian_trans.shape()[0];
    unsigned int cols_trans = jacobian_trans.shape()[1];

    unsigned int rows_rot = jacobian_rot.shape()[0];
    unsigned int cols_rot = jacobian_rot.shape()[1];

    if(cols_trans != cols_rot){
		LOG_ERROR(boost::str(boost::format("[Jacobian] Failed to create jacobian. "
										   "Translational and rotation jacobians do not have same size "
										   "(trans: %d, rot: %d)") % cols_trans % cols_rot));
        return;
    }

    _jacobian.resize(rows_trans + rows_rot, cols_rot);

    for(unsigned int i=0; i < rows_trans; i++){
        for(unsigned int j=0; j < cols_trans; j++){
            _jacobian(i,j) = jacobian_trans[i][j];
        }
    }

    for(unsigned int i=0; i < rows_rot; i++){
        for(unsigned int j=0; j < cols_rot; j++){
            _jacobian(i + rows_trans,j) = jacobian_rot[i][j];
        }
    }
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Jacobian::pinv() const {

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jtrans = transpose();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jjt = _jacobian * jtrans;
    return jtrans * jjt.inverse();

}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Jacobian::nullspace_projector() const {

    Eigen::MatrixXd iden = Eigen::MatrixXd::Identity(_jacobian.cols(), _jacobian.cols());
    
    return iden - (pinv() * _jacobian);
}

