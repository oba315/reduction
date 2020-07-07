#include "matrix_tools.h"


// 行を削除
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi &SVJ, int c ) {

	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++)   SVJ(i, 0) = i;

	Eigen::MatrixXd A = X.topRows(c);
	Eigen::MatrixXd B = X.bottomRows(X.rows() - c - 1);

	X.resize(A.rows() + B.rows(), A.cols());
	X << A,
		 B;

	// SVJを修正
	SVJ(c, 0) = -1;
	for (int k = c + 1; k < n; k++) SVJ(k, 0) -= 1;
	
}
void remove_row(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, int c) {

	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++)   SVJ(i, 0) = i;

	Eigen::MatrixXi A = X.topRows(c);
	Eigen::MatrixXi B = X.bottomRows(X.rows() - c - 1);

	X.resize(A.rows() + B.rows(), A.cols());
	X << A,
		B;

	// SVJを修正
	SVJ(c, 0) = -1;
	for (int k = c + 1; k < n; k++) SVJ(k, 0) -= 1;

}

// 複数行を削除
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi &SVJ, std::vector<int> c) {

	//んまあ倍くらい早いけどびみょ

	// 削除後の頂点番号の対応関係

	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1); // SVJ : 元の頂点番号→削除後の頂点番号
	for (int i = 0; i < n; i++) { SVJ(i, 0) = i; }

	std::cout << "1" << std::endl;

	// std::cout << SVJ << std::endl;
	int newrows = X.rows() - c.size();
	bool skip = false;
	int index = 0;
	for (int i = 0; i < X.rows(); i++) {
		for (int cc : c) {
			if (i == cc) {
				skip = true;
				continue;
			}
		}
		if (skip) {
			skip = false;
			SVJ(i, 0) = -1;
			continue;
		}
		//std::cout << i << "->" << index << std::endl;
		if (i != index) X.row(index) = X.row(i);
		index++;
		SVJ(i, 0) = X(index);
	}
	std::cout << "2" << std::endl;

	// Xに一度に入れるとなんかエラー
	Eigen::MatrixXd temp = X.topRows(newrows);
	X = temp; 
	return;

	
	
}
#include <complex>
void remove_row(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, std::vector<int> c) {
	
	//んまあ倍くらい早いけどびみょ
	
	// 削除後の頂点番号の対応関係
	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++) { SVJ(i, 0) = i; }
	
std::cout << "1" << std::endl;
	// std::cout << SVJ << std::endl;
	int newrows = X.rows() - c.size();
	bool skip = false;
	int index = 0;
	for (int i = 0; i < X.rows(); i++) {
		for (int cc : c) {
			if (i == cc) {
				skip = true;
				continue;
			}
		}
		if (skip) {
			skip = false;
			continue;
		}
		//std::cout << i << "->" << index << std::endl;
		if (i != index) X.row(index) = X.row(i);
		index++;
	}
	std::cout << "2" << std::endl;

	// Xに一度に入れるとなんかエラー
	Eigen::MatrixXi temp = X.topRows(newrows);
	X = temp;
	return;
	
	
	
}



