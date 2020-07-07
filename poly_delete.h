#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

namespace poldel {

	//ƒ}ƒXƒN‚É‰‚¶‚Ä–Ê‚ğíœ
	void poly_delete(
		Eigen::MatrixXd& V,
		Eigen::MatrixXi& F,
		Eigen::MatrixXd& C,
		Eigen::VectorXi& mask, //1‚Ì–Ê‚ğíœ
		bool debug = true
	);

}
