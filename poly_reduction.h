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
void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::MatrixXi& InfRefMAP, // �i�K�t�������}�b�v
	std::vector<RowVecter2i_Float>& filter,
	bool is_remesh = false);


void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C);


/* ���C�u�������g�p�������_�N�V���� ����\�[�g���Ă��� �͈͂̐����Ȃ�*/
void reduction(Eigen::MatrixXd &V, Eigen::MatrixXi &F, float ratio, Eigen::VectorXd face_weight);
