#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

struct RowVecter2i_Float {
	Eigen::RowVector2i befor;
	double after;
};



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
void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::MatrixXi& InfRefMAP, // 段階付き可視性マップ
	std::vector<RowVecter2i_Float>& filter,
	bool is_remesh = false);


void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C);


/* ライブラリを使用したリダクション 毎回ソートしている 範囲の制限なし*/
void reduction(Eigen::MatrixXd &V, Eigen::MatrixXi &F, float ratio, Eigen::VectorXd face_weight);
