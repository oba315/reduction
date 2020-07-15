#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif


#include<vector>


void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi& SVJ, int c);
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi& SVJ, std::vector<int> c);
void remove_row(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, int c);
void remove_row(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, std::vector<int> c);

void remove_row2(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, std::vector<int> c);
void remove_row2(Eigen::MatrixXd& X, Eigen::MatrixXi& SVJ, std::vector<int> c);

// �s�K�v�ȍs���폜�BSVJ�̓A�E�g�v�b�g�B�Â��s��ƐV�����s��Ƃ̍s�ԍ��̑Ή���Ԃ��Bc�͏��������Ƀ\�[�g����Ă��Ȃ��ƃ_���B
// SVJ : �����̃��O


void vec_normalize(Eigen::VectorXd& X);