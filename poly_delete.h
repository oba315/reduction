#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

namespace poldel {

	//マスクに応じて面を削除
	void poly_delete(
		Eigen::MatrixXd& V,
		Eigen::MatrixXi& F,
		Eigen::MatrixXd& C,
		Eigen::VectorXi& mask, //1の面を削除
		bool debug = true
	);

}
