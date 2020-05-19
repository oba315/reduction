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

void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask, // �i�K�t�������}�b�v
	const double ratio = 1,
	bool is_remesh = true
);

void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);