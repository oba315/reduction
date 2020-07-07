#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/collapse_edge.h>
#include <vector>
#include "poly_reduction.h"
#include <igl/remove_duplicate_vertices.h>
#include <Eigen/Dense>
#include "matrix_tools.h"
#include <cassert>
#include <time.h>
#include <set>

//------------------------------------------------------------------------------------------------//
// �S�̂����̊����ō팸
void poly_reduction(Eigen::MatrixXd& V, Eigen::MatrixXi& F, const double ratio, bool is_remesh);


//------------------------------------------------------------------------------------------------//




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
Eigen::MatrixXd VecToMatXd(const std::vector<double> vec)
{
	int rows(vec.size() / 3), cols(3);
	Eigen::MatrixXd temp(rows, cols);
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

// �d�����_�̍폜
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
	clock_t start = clock();

	//- - - - - - �d�����_�̍폜 - - - - - - - - - -
	int vn = V.rows();
	//http://sssiii.seesaa.net/article/434541169.html
	Eigen::MatrixXd SV;
	Eigen::MatrixXi SVI, SVJ;
	igl::remove_duplicate_vertices(V, 0.0000001, SV, SVI, SVJ);     //�d�����_�̍폜 SV�ɍ폜��̒��_
	
	if (V.rows() == SV.rows()) {
		std::cout << "�s�����_�͂���܂���." << std::endl;
		return;
	}
	else  std::cout << "�s�����_��" << V.rows() - SV.rows() << "����܂��B�C�����܂��B" << std::endl;
	
	V = SV;
	
	//- - - - - �ʂ��\�����钸�_�̃C���f�b�N�X���X�V - - - - - 
	// SVJ�Ɍ��̒��_�ԍ����폜��̒��_�ԍ����i�[����Ă���̂ŁA
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
	std::cout << "	finish (" << vn << "->" << V.rows() << ")\n	   time:" << clock() - start << "[ms]" << std::endl;
	
	/*
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
		//Eigen::MatrixXi SVJ;
		//remove_row(V, SVJ, flag);
		std::cout << "V.rows()" << V.rows();
		// �ʂ̑Ή����C��(�����_�ԍ���V���_�ԍ��ɕύX)
		for (int i = 0; i < F.rows(); i++)
		{
			for (int j = 0; j < F.cols(); j++)
			{
				int temp = F(i, j);
				F(i, j) = SVJ(temp);
			}
		}
	}std::cout << "time:" << clock() - start << "[ms]";
	*/
}
// �d�����_�̍폜
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C) {
	clock_t start = clock();

	//- - - - - - �d�����_�̍폜 - - - - - - - - - -
	int vn = V.rows();
	//http://sssiii.seesaa.net/article/434541169.html
	Eigen::MatrixXd SV;
	Eigen::MatrixXi SVI, SVJ;
	igl::remove_duplicate_vertices(V, 0.0000001, SV, SVI, SVJ);     //�d�����_�̍폜 SV�ɍ폜��̒��_

	if (V.rows() == SV.rows()) {
		std::cout << "�s�����_�͂���܂���." << std::endl;
		return;
	}
	else  std::cout << "�s�����_��" << V.rows() - SV.rows() << "����܂��B�C�����܂��B" << std::endl;

	V = SV;

	//- - - - - �ʂ��\�����钸�_�̃C���f�b�N�X���X�V - - - - - 
	// SVJ�Ɍ��̒��_�ԍ����폜��̒��_�ԍ����i�[����Ă���̂ŁA
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
	std::vector<double> tempvecC;
	for (int i = 0; i < F.rows(); i++) {
		if (F(i, 0) != F(i, 1) && F(i, 1) != F(i, 2) && F(i, 2) != F(i, 0)) {
			tempvec.push_back(F(i, 0));
			tempvec.push_back(F(i, 1));
			tempvec.push_back(F(i, 2));
			tempvecC.push_back(C(i, 0));
			tempvecC.push_back(C(i, 1));
			tempvecC.push_back(C(i, 2));

		}
	}
	Eigen::MatrixXi temp = VecToMat(tempvec);
	Eigen::MatrixXd tempC = VecToMatXd(tempvecC);
	F = temp;
	C = tempC;
	std::cout << "	finish (" << vn << "->" << V.rows() << ")\n	   time:" << clock() - start << "[ms]" << std::endl;
}

//  regacy
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
	int count = 0;
	std::vector<int> collapsed_edge;
	for (int i = 0; i < max_iter; i++) {

		igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);

		collapsed_edge.push_back(Q[i].index);	// legacy
		count++;
	}
	std::cout << count << "�̕ӂ��ȗ����܂����D\n";

	// �ʂ��č\��:�d���ʂ̍폜
	if (is_remesh) {
		remesh(V, F);
	}
}

void temp(std::vector< edge_info > &Q, Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXi &E, Eigen::VectorXi &EMAP, Eigen::MatrixXi &EF, Eigen::MatrixXi &EI) {
	// - - - - - �ӂ̒����ƒ��_�𒲂ׂ� - - - - - - -
	Q = {};
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);

		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}
	std::sort(Q.begin(), Q.end(), cmp);		// ��r�֐�cmp���g�p����sort
}

// �͈͂��w�肵�ă��_�N�V����
// prim_mask(0or1)��1�̖ʂ���C���Ԃɍ팸���Ă���
// �ł��Z���ӂ��璸�_������
// �������ʒu�𓯂��ɂ��邾���Ȃ̂ŁA�ʂ̍č\�����K�v
// ��̕ӂ���������C�������ʂ��X�V���ĂȂ������x�ɂ�����������Ɣ����Ȋ����ɂȂ�D
void limited_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask,  // �ʔԍ��������̃}�b�v
	const double ratio,				   // prim_mask ��1�̕����̂����������������񂷂邩
	bool is_remesh
) {
	std::cout << "�͈͂��w�肵�ă��_�N�V����" << std::endl;

	// �Ӎs����쐬
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	igl::edge_flaps(F, E, EMAP, EF, EI);

	
	// - - - - - �ӂ̒����ƒ��_�𒲂ׂ� - - - - - - -
	std::vector< edge_info > Q;
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		
		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}
	std::sort(Q.begin(), Q.end(), cmp);		// ��r�֐�cmp���g�p����sort

	
	// - - - - - - prim_mask���璸�_��mask���쐬 - - - - - - - - - - 
	Eigen::VectorXi vert_mask = Eigen::VectorXi::Zero(V.rows());
	for (int i = 0; i < prim_mask.rows(); i++) {
		if (prim_mask(i) == 0) {
			vert_mask(F(i, 0)) = 1;
			vert_mask(F(i, 1)) = 1;
			vert_mask(F(i, 2)) = 1;

		}
	}
	//: " << vert_mask << std::endl;

	/*
	for (int i = 0; i < Q.size(); i++) {
		std::cout << Q[i].index << " " << Q[i].cost << std::endl;
	}
	*/

	// max_iter : �팸����ӂ̐�
	// prim_mask ��1�̕����̂����������������񂷂邩
	//double max_iter = int( E.rows() * ratio );
	double maskcount = 0;
	for (int i = 0; i < E.rows(); i++) {
		if (vert_mask(E(Q[i].index, 0)) != 1 && vert_mask(E(Q[i].index, 1)) != 1 ) {
			maskcount++;
		}
	}
	double max_iter = int( maskcount * ratio );
	std::cout << "maskcount " << maskcount << " maxiter " << max_iter;

	//std::cout << "max_iter : " << max_iter  << " / E.rows() : " << E.rows() << std::endl;

	std::vector<int> collapsed_edge;
	int i = 1;		
	int count = 0;  // �ȗ����ꂽ�ӂ̐�
	
	   
	while (count < max_iter) {
		//std::cout << "cllps " << i << std::endl;
		if (vert_mask(E(Q[i].index, 0)) != 1 && vert_mask(E(Q[i].index, 1)) != 1) { //�ӂ̗��[���}�X�N���̒��_���ǂ���

			//Eigen::MatrixXi uE;
			//igl::edge_flaps(F, uE, EMAP, EF, EI);
			//std::cout << "[" << Q[i].index  << "]" << EF(Q[i].index, 0) << "/" << EF(Q[i].index, 1) << std::endl;

			igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);

			collapsed_edge.push_back(Q[i].index);
			count++;

			// �ēx�\�[�g
			//temp(Q, V, F, E, EMAP, EF, EI);
			//std::cout << count << std::endl ;
		}
		i++;
		if (i >= E.rows()) {
			std::cout << "ERROR" << std::endl;
			break;
		}
	}
	std::cout << count << "�̕ӂ��ȗ����܂���" << std::endl;

	// �ʂ��č\��:�d���ʂ̍폜
	if (is_remesh) {
		remesh(V, F);
	}
}

void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask, // �i�K�t�������}�b�v
	const double ratio,
	bool is_remesh
) {
	int max = prim_mask.maxCoeff();

	
	Eigen::VectorXi new_mask = Eigen::VectorXi::Ones(prim_mask.rows());
	for (int k = 1; k < 5; k++) {
		for (int i = 0; i < prim_mask.rows(); i++) {
			if (prim_mask(i) == k) {
				new_mask(i) = 0;
				// std::cout << k;
			}
		}
		//std::cout << new_mask;
		limited_area_poly_reduction(V, F, new_mask, 0.02 * k, false);
	}

	for (int i = 0; i < prim_mask.rows(); i++) {
		if (prim_mask(i) == -1) {
			new_mask(i) = 0;
			// std::cout << -1;
		}
	}
	//std::cout << new_mask;
	limited_area_poly_reduction(V, F, new_mask, 0.5, false);
}

void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::MatrixXi& InfRefMAP, // �i�K�t�������}�b�v
	std::vector<RowVecter2i_Float> & filter,
	bool is_remesh) 
{
	// Filter�Ŏw�肵�������ō팸���s��(filter[i].after�͍팸����ʂ̊���)
	Eigen::VectorXi mask = Eigen::VectorXi::Zero(F.rows());
	for (RowVecter2i_Float fi : filter)
	{

		for (int i = 0; i < InfRefMAP.rows(); i++)
		{
			//std::cout << InfRefMAP.row(i) << std::endl;
			//std::cout << fi.befor << std::endl;

			if (InfRefMAP.row(i) == fi.befor) {
				mask(i) = 1;
			}
		}
		limited_area_poly_reduction(V, F, mask, fi.after, is_remesh);
	}
}

/* ���C�u�������g�p�������_�N�V����
   �����炭����\�[�g���Ă���H */
void temp2(Eigen::MatrixXd &V, Eigen::MatrixXi &F, float ratio) {
	using namespace std;
	using namespace Eigen;
	using namespace igl;

	std::cout << "���_���팸���܂�" << std::endl;
  
  
	// Prepare array-based edge data structures and priority queue
	VectorXi EMAP;
	MatrixXi E,EF,EI;
	typedef std::set<std::pair<double,int> > PriorityQueue;	// std::set �����Ń\�[�g���Ă����^
	PriorityQueue Q;
	std::vector<PriorityQueue::iterator > Qit;
	
	MatrixXd C;												// �ӂ̒��S���W
	int num_collapsed;										// �팸�������_�̐�

	// Function to reset original mesh and data structures
    std::cout << "Reset\n";
	edge_flaps(F,E,EMAP,EF,EI);
	Qit.resize(E.rows());

	C.resize(E.rows(),V.cols());
	VectorXd costs(E.rows());
	Q.clear();
	for (int e = 0; e < E.rows(); e++)
	{
		double cost = e;
		RowVectorXd p(1, 3);
		shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		C.row(e) = p;
		Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;	
		// �}�������l�ւ̃C�e���[�^�C�܂�Qit[e]��Q�̕ӂ��ɂ��Ẵf�[�^�ւ̃C�e���[�^
	}
	num_collapsed = 0;

	std::cout << "V : " << V.rows() << std::endl;
	std::cout << "F : " << F.rows() << std::endl;
	std::cout << "E : " << E.rows() << std::endl;
	std::cout << "VV : " << V.row(100000) << std::endl;
	std::cout << "FF : " << F.row(100) << std::endl;
	

    // If animating then collapse 10% of edges
	bool something_collapsed = false;
	// collapse edge
	//const int max_iter = std::ceil(ratio * Q.size());
	const int max_iter = 200000;
	std::cout << "max_iter : " << max_iter << std::endl;
	int count = 0;
	for (int j = 0; j < max_iter; j++)
	{
		if (!collapse_edge(
			shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, Qit, C)
			// �ȗ������ӂ͂p����폜�����D(Q�͂��̂����Ŏ����Ń\�[�g)
			// �ȗ��̂��сCshortest_edge_and_midpoint��Q���X�V�����D�������ɏd�݂������...?
			// F���␳����C�ȗ����ꂽ�ʂ�[0,0,0]�ɂȂ�D
			// �ȗ����ꂽV�͌Ǘ����_�ɂȂ�.
			// �ڂ����̓����Q��
			)
		{
			//std::cout << count  << "--break\n";
			if (count > max_iter)break;
			//break;
		}
		else {
			//std::cout << "-";
			count++;
		}
		something_collapsed = true;
		num_collapsed++;
	}
	std::cout << "connt : " << count << std::endl;

	std::cout << "V : " << V.rows() << std::endl;
	std::cout << "F : " << F.rows() << std::endl;
	std::cout << "E : " << E.rows() << std::endl;
	std::cout << "VV : " << V.row(100000) << std::endl;
	std::cout << "FF : " << F.row(100) << std::endl;
	
	int counts = 0;
	for (int i = 0; i < F.rows(); i++) {
		if (!(F(i, 0) == 0 && F(i,1) == 0)) {
			counts++;
		}
	}
	std::cout << "count : " << counts << std::endl;
}