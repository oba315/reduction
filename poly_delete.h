#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

namespace poldel {

	//�}�X�N�ɉ����Ėʂ��폜
	void poly_delete(
		Eigen::MatrixXd& V,
		Eigen::MatrixXi& F,
		Eigen::MatrixXd& C,
		Eigen::VectorXi& mask, //1�̖ʂ��폜
		bool debug = true
	);

}
