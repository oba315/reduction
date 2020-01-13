#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

void poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const double ratio,
	bool is_remesh = true
);

void limited_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask,  // �ʔԍ��������̃}�b�v
	const double ratio,
	bool is_remesh = true
);

