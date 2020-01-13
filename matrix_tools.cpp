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

// 複数行を削除
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi &SVJ, std::vector<int> c) {

	const std::vector<int> constc = c;

	// 削除後の頂点番号の対応関係
	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++)   SVJ(i, 0) = i;
	
	// std::cout << SVJ << std::endl;

	for (int i = 0; i < c.size(); i++) {

		// 行を一つ削除
		if (c[i] < X.rows()) {
			Eigen::MatrixXd A = X.topRows(c[i]);
			Eigen::MatrixXd B = X.bottomRows(X.rows() - c[i] - 1);

			X.resize(A.rows() + B.rows(), A.cols());
			X << A,
				B;

			// 残りの行番号を修正
			for (int j = i + 1; j < c.size(); j++) {
				c[j] -= 1;
			}

			// SVJを修正
			SVJ(constc[i], 0) = -1;
			for (int k = constc[i] + 1; k < n; k++) SVJ(k, 0) -= 1;
		}

		else {
			std::cerr << "ERROR : remove_row : 行番号が行数を上回っている、もしくはcがソートされていません" << std::endl << std::endl;
			break;
		}
		//std::cout << SVJ << std::endl << std::endl;
	}
	
}

