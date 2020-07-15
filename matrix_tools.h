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

// 不必要な行を削除。SVJはアウトプット。古い行列と新しい行列との行番号の対応を返す。cは小さい順にソートされていないとダメ。
// SVJ : 処理のログ


void vec_normalize(Eigen::VectorXd& X);