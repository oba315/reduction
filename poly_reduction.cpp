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
	int index;					 // Eの何行目にあたるか
	double cost;				 // 辺の長さ
	Eigen::RowVectorXd midpoint; // 辺の中点
};

// ソート用比較関数
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

// 重複頂点の削除(要修正)
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {



	// 重複頂点の削除

	//http://sssiii.seesaa.net/article/434541169.html
	Eigen::MatrixXd SV;
	Eigen::MatrixXi SVI, SVJ;
	igl::remove_duplicate_vertices(V, 0.0000001, SV, SVI, SVJ);     // SVに削除後の頂点
	V = SV;
	// SVJに元の頂点番号→削除後の頂点番号が格納されているので、
	// インデックスリストを更新
	// ループを回さず一括処理する方法があるはず？
	for (int i = 0; i < F.rows(); i++)
	{
		for (int j = 0; j < F.cols(); j++)
		{
			int temp = F(i, j);
			F(i, j) = SVJ(temp);
		}
	}

	//　構成頂点に同一の頂点を含む面の削除

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

	// 不正頂点のチェック
	std::vector<int> flag;
	for (int i = 0; i < V.rows(); i++) {
		if ((F.array() != i).all()) {
			flag.push_back(i);
		}
	}
	if(flag.size() == 0)  std::cout << "不正頂点はありません." << std::endl;
	else {
		std::cout << "不正頂点が" << flag.size() << "個あります。修正します。" << std::endl;

		// 頂点を削除
		Eigen::MatrixXi SVJ;
		remove_row(V, SVJ, flag);

		// 面の対応を修正
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
	// 削減する頂点の数
	const int max_iter = std::ceil(V.rows() * ratio);
	std::cout << "max_iter : " << max_iter << std::endl;


	// 辺行列を作成
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	igl::edge_flaps(F, E, EMAP, EF, EI);


	// 辺の長さと中点を調べる
	std::vector< edge_info > Q;
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}

	std::sort(Q.begin(), Q.end(), cmp);		//比較関数cmpを使用してsort


	// for (auto itr : Q){ std::cout << "index:" << itr.index << " cost:" << itr.cost << std::endl; }

	// 最も短い辺から頂点を結合
	// ただし位置を同じにするだけなので、面の再構成が必要
	std::vector<int> collapsed_edge;
	for (int i = 0; i < max_iter; i++) {

		igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);

		collapsed_edge.push_back(Q[i].index);	// legacy
	}


	// 面を再構成
	if (is_remesh) {

		remesh(V, F);

	}
}

// 範囲を指定してリダクション
void limited_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask,  // 面番号→可視性のマップ
	const double ratio,
	bool is_remesh
) {
	std::cout << "範囲を指定してリダクション" << std::endl;

	// 辺行列を作成
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	igl::edge_flaps(F, E, EMAP, EF, EI);

	
	// 辺の長さと中点を調べる
	std::vector< edge_info > Q;
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}
	std::sort(Q.begin(), Q.end(), cmp);		//比較関数cmpを使用してsort

	
											// prim_maskから頂点のmaskを作成
	Eigen::VectorXi vert_mask = Eigen::VectorXi::Zero(V.rows());
	for (int i = 0; i < prim_mask.rows(); i++) {
		if (prim_mask(i) == 1) {
			vert_mask(F(i, 0)) = 1;
			vert_mask(F(i, 1)) = 1;
			vert_mask(F(i, 2)) = 1;
		}
	}
	//std::cout << "vert_mask (" << vert_mask.rows() << ") : " << vert_mask << std::endl;

	// 最も短い辺から頂点を結合
	// ただし位置を同じにするだけなので、面の再構成が必要
	
	// 省略の度合いは見えていない部分の中の割合で指定
	//for (int i = 0; i < );
	double max_iter = int( E.rows() * ratio );
	std::cout << "max_iter : " << max_iter  << " / E.rows() : " << E.rows() << std::endl;

	std::vector<int> collapsed_edge;
	int i = 1;		
	int count = 0;  // 省略された辺の数
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
	std::cout << count << "個の辺を省略しました" << std::endl;

	// 面を再構成
	if (is_remesh) {
		remesh(V, F);
	}
}