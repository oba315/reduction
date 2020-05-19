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
	const Eigen::VectorXi& prim_mask,  // 面番号→可視性のマップ
	const double ratio,
	bool is_remesh = true
);

void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask, // 段階付き可視性マップ
	const double ratio = 1,
	bool is_remesh = true
);

void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);