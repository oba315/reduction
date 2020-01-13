#pragma once

#include<iostream>

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

// 頂点位置情報をコール
void call_vertex_position(Eigen::MatrixXd V);

// メッシュの移動
Eigen::MatrixXd move_mash(Eigen::MatrixXd &V, Eigen::RowVector3d m);