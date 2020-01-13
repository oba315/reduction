#include "matrix_tools.h"


// �s���폜
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi &SVJ, int c ) {

	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++)   SVJ(i, 0) = i;

	Eigen::MatrixXd A = X.topRows(c);
	Eigen::MatrixXd B = X.bottomRows(X.rows() - c - 1);

	X.resize(A.rows() + B.rows(), A.cols());
	X << A,
		 B;

	// SVJ���C��
	SVJ(c, 0) = -1;
	for (int k = c + 1; k < n; k++) SVJ(k, 0) -= 1;
	
}

// �����s���폜
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi &SVJ, std::vector<int> c) {

	const std::vector<int> constc = c;

	// �폜��̒��_�ԍ��̑Ή��֌W
	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++)   SVJ(i, 0) = i;
	
	// std::cout << SVJ << std::endl;

	for (int i = 0; i < c.size(); i++) {

		// �s����폜
		if (c[i] < X.rows()) {
			Eigen::MatrixXd A = X.topRows(c[i]);
			Eigen::MatrixXd B = X.bottomRows(X.rows() - c[i] - 1);

			X.resize(A.rows() + B.rows(), A.cols());
			X << A,
				B;

			// �c��̍s�ԍ����C��
			for (int j = i + 1; j < c.size(); j++) {
				c[j] -= 1;
			}

			// SVJ���C��
			SVJ(constc[i], 0) = -1;
			for (int k = constc[i] + 1; k < n; k++) SVJ(k, 0) -= 1;
		}

		else {
			std::cerr << "ERROR : remove_row : �s�ԍ����s���������Ă���A��������c���\�[�g����Ă��܂���" << std::endl << std::endl;
			break;
		}
		//std::cout << SVJ << std::endl << std::endl;
	}
	
}

