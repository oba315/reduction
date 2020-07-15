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
void remove_row(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, int c) {

	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1);
	for (int i = 0; i < n; i++)   SVJ(i, 0) = i;

	Eigen::MatrixXi A = X.topRows(c);
	Eigen::MatrixXi B = X.bottomRows(X.rows() - c - 1);

	X.resize(A.rows() + B.rows(), A.cols());
	X << A,
		B;

	// SVJ���C��
	SVJ(c, 0) = -1;
	for (int k = c + 1; k < n; k++) SVJ(k, 0) -= 1;

}

// �����s���폜
// c�͍폜�������ӂ̃C���f�b�N�X
void remove_row(Eigen::MatrixXd& X, Eigen::MatrixXi &SVJ, std::vector<int> c) {

	//��܂��{���炢�������ǂт݂�

	// �폜��̒��_�ԍ��̑Ή��֌W

	const int n = X.rows();
	SVJ = Eigen::MatrixXi::Zero(n, 1); // SVJ : ���̒��_�ԍ����폜��̒��_�ԍ�
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
		SVJ(i, 0) = index;
		index++;
		
	}
	std::cout << "2" << std::endl;

	// X�Ɉ�x�ɓ����ƂȂ񂩃G���[
	Eigen::MatrixXd temp = X.topRows(newrows);
	X = temp; 
	return;

	
	
}
#include <complex>
void remove_row(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, std::vector<int> c) {
	
	//��܂��{���炢�������ǂт݂�
	
	// �폜��̒��_�ԍ��̑Ή��֌W
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

	// X�Ɉ�x�ɓ����ƂȂ񂩃G���[
	Eigen::MatrixXi temp = X.topRows(newrows);
	X = temp;
	return;
	
	
	
}


// �w�肵����𔲂����s��ւ̕ϊ��s����쐬���ď����D�͂₭�Ȃ肻���ȋC���������ǁC�s��H���ł������ă������G���[
void remove_row2(Eigen::MatrixXi& X, Eigen::MatrixXi& SVJ, std::vector<int> c) {
	int cs = c.size();
	std::cout << X.rows() - cs << " " << X.rows();
	int a = X.rows() - cs;
	int b = X.rows();
	Eigen::MatrixXi H = Eigen::MatrixXi::Zero(a,b);
	
	auto iter = c.begin();
	int inc = 0;
	for (int i = 0; i < X.rows(); i++) {
		if (i == *iter) {
			++iter;
			continue;
		}
		H(i, inc) = 1;
		inc++;
	}
	Eigen::MatrixXi P = H * X;
	X = P;
}

void remove_row2(Eigen::MatrixXd& X, Eigen::MatrixXi& SVJ, std::vector<int> c) {
	/*
	int cs = c.size();
	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(X.rows() - cs, X.rows());
	auto iter = c.begin();
	int inc = 0;

	for (int i = 0; i < X.rows(); i++) {
		if (i == *iter) {
			++iter;
			continue;
		}
		H(i, inc) = 1;
		inc++;
	}
	//Eigen::MatrixXd P = H * X;
	//X = P;*/
}



void vec_normalize(Eigen::VectorXd& X) {
	double maxx = 0;
	double minn = DBL_MAX;
	for (int i = 0; i < X.size(); i++) {
		if (X(i) > maxx) maxx = X(i);
		if (X(i) < minn) minn = X(i);
	}
	double range = maxx - minn;
	X.array() = (X.array() - minn) / range;
}