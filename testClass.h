#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

// ���_�ʒu�����R�[��
void call_vertex_position(Eigen::MatrixXd V);

// ���b�V���̈ړ�
Eigen::MatrixXd move_mash(Eigen::MatrixXd &V, Eigen::RowVector3d m);