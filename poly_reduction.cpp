#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/collapse_edge.h>
#include <vector>
#include "poly_reduction.h"
#include <igl/remove_duplicate_vertices.h>
#include <Eigen/Dense>
#include "matrix_tools.h"

struct edge_info {
	int index;					 // E�̉��s�ڂɂ����邩
	double cost;				 // �ӂ̒���
	Eigen::RowVectorXd midpoint; // �ӂ̒��_
};

// �\�[�g�p��r�֐�
bool cmp(const edge_info& a, const edge_info& b)
{
	return a.cost < b.cost;
}

Eigen::MatrixXi VecToMat(const std::vector<double> vec)
{
	int rows(vec.size() / 3), cols(3);
	Eigen::MatrixXi temp(rows, cols);
	int count(0);
	for (int i(0); i < rows; ++i)
	{
		temp(i, 0) = vec[count];
		temp(i, 1) = vec[count + 1];
		temp(i, 2) = vec[count + 2];
		count += 3;
	}

	return temp;
}

// �d�����_�̍폜(�v�C��)
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {



	// �d�����_�̍폜

	//http://sssiii.seesaa.net/article/434541169.html
	Eigen::MatrixXd SV;
	Eigen::MatrixXi SVI, SVJ;
	igl::remove_duplicate_vertices(V, 0.0000001, SV, SVI, SVJ);     // SV�ɍ폜��̒��_
	V = SV;
	// SVJ�Ɍ��̒��_�ԍ����폜��̒��_�ԍ����i�[����Ă���̂ŁA
	// �C���f�b�N�X���X�g���X�V
	// ���[�v���񂳂��ꊇ����������@������͂��H
	for (int i = 0; i < F.rows(); i++)
	{
		for (int j = 0; j < F.cols(); j++)
		{
			int temp = F(i, j);
			F(i, j) = SVJ(temp);
		}
	}

	//�@�\�����_�ɓ���̒��_���܂ޖʂ̍폜

	std::vector<double> tempvec;
	for (int i = 0; i < F.rows(); i++) {
		if (F(i, 0) != F(i, 1) && F(i, 1) != F(i, 2) && F(i, 2) != F(i, 0)) {
			tempvec.push_back(F(i, 0));
			tempvec.push_back(F(i, 1));
			tempvec.push_back(F(i, 2));
		}
	}
	Eigen::MatrixXi temp = VecToMat(tempvec);

	F = temp;

	// �s�����_�̃`�F�b�N
	std::vector<int> flag;
	for (int i = 0; i < V.rows(); i++) {
		if ((F.array() != i).all()) {
			flag.push_back(i);
		}
	}
	if(flag.size() == 0)  std::cout << "�s�����_�͂���܂���." << std::endl;
	else {
		std::cout << "�s�����_��" << flag.size() << "����܂��B�C�����܂��B" << std::endl;

		// ���_���폜
		Eigen::MatrixXi SVJ;
		remove_row(V, SVJ, flag);

		// �ʂ̑Ή����C��
		for (int i = 0; i < F.rows(); i++)
		{
for (int j = 0; j < F.cols(); j++)
{
	int temp = F(i, j);
	F(i, j) = SVJ(temp);
}
		}
	}


}



void poly_reduction(Eigen::MatrixXd& V, Eigen::MatrixXi& F, const double ratio, bool is_remesh)
{
	// �팸���钸�_�̐�
	const int max_iter = std::ceil(V.rows() * ratio);
	std::cout << "max_iter : " << max_iter << std::endl;


	// �Ӎs����쐬
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	igl::edge_flaps(F, E, EMAP, EF, EI);


	// �ӂ̒����ƒ��_�𒲂ׂ�
	std::vector< edge_info > Q;
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}

	std::sort(Q.begin(), Q.end(), cmp);		//��r�֐�cmp���g�p����sort


	// for (auto itr : Q){ std::cout << "index:" << itr.index << " cost:" << itr.cost << std::endl; }

	// �ł��Z���ӂ��璸�_������
	// �������ʒu�𓯂��ɂ��邾���Ȃ̂ŁA�ʂ̍č\�����K�v
	std::vector<int> collapsed_edge;
	for (int i = 0; i < max_iter; i++) {

		igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);

		collapsed_edge.push_back(Q[i].index);	// legacy
	}


	// �ʂ��č\��
	if (is_remesh) {

		remesh(V, F);

	}
}

// �͈͂��w�肵�ă��_�N�V����
void limited_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask,  // �ʔԍ��������̃}�b�v
	const double ratio,
	bool is_remesh
) {
	std::cout << "�͈͂��w�肵�ă��_�N�V����" << std::endl;

	// �Ӎs����쐬
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	igl::edge_flaps(F, E, EMAP, EF, EI);

	
	// �ӂ̒����ƒ��_�𒲂ׂ�
	std::vector< edge_info > Q;
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}
	std::sort(Q.begin(), Q.end(), cmp);		//��r�֐�cmp���g�p����sort

	
											// prim_mask���璸�_��mask���쐬
	Eigen::VectorXi vert_mask = Eigen::VectorXi::Zero(V.rows());
	for (int i = 0; i < prim_mask.rows(); i++) {
		if (prim_mask(i) == 1) {
			vert_mask(F(i, 0)) = 1;
			vert_mask(F(i, 1)) = 1;
			vert_mask(F(i, 2)) = 1;
		}
	}
	//std::cout << "vert_mask (" << vert_mask.rows() << ") : " << vert_mask << std::endl;

	// �ł��Z���ӂ��璸�_������
	// �������ʒu�𓯂��ɂ��邾���Ȃ̂ŁA�ʂ̍č\�����K�v
	
	// �ȗ��̓x�����͌����Ă��Ȃ������̒��̊����Ŏw��
	//for (int i = 0; i < );
	double max_iter = int( E.rows() * ratio );
	std::cout << "max_iter : " << max_iter  << " / E.rows() : " << E.rows() << std::endl;

	std::vector<int> collapsed_edge;
	int i = 1;		
	int count = 0;  // �ȗ����ꂽ�ӂ̐�
	while (count < max_iter) {
		//std::cout << "cllps " << i << std::endl;
		if (vert_mask(E(Q[i].index, 0)) != 1 && vert_mask(E(Q[i].index, 1)) != 1){
			igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);
			collapsed_edge.push_back(Q[i].index);
			count++;
		}
		i++;
		if (i >= E.rows()) {
			std::cout << "ERROR" << std::endl;
			break;
		}
	}
	std::cout << count << "�̕ӂ��ȗ����܂���" << std::endl;

	// �ʂ��č\��
	if (is_remesh) {
		remesh(V, F);
	}
}