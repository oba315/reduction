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
// 全体を一定の割合で削減
void poly_reduction(Eigen::MatrixXd& V, Eigen::MatrixXi& F, const double ratio, bool is_remesh);


//------------------------------------------------------------------------------------------------//




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

// 重複頂点の削除
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
	clock_t start = clock();

	//- - - - - - 重複頂点の削除 - - - - - - - - - -
	int vn = V.rows();
	//http://sssiii.seesaa.net/article/434541169.html
	Eigen::MatrixXd SV;
	Eigen::MatrixXi SVI, SVJ;
	igl::remove_duplicate_vertices(V, 0.0000001, SV, SVI, SVJ);     //重複頂点の削除 SVに削除後の頂点
	
	if (V.rows() == SV.rows()) {
		std::cout << "不正頂点はありません." << std::endl;
		return;
	}
	else  std::cout << "不正頂点が" << V.rows() - SV.rows() << "個あります。修正します。" << std::endl;
	
	V = SV;
	
	//- - - - - 面を構成する頂点のインデックスを更新 - - - - - 
	// SVJに元の頂点番号→削除後の頂点番号が格納されているので、
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
	std::cout << "	finish (" << vn << "->" << V.rows() << ")\n	   time:" << clock() - start << "[ms]" << std::endl;
	
	/*
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
		//Eigen::MatrixXi SVJ;
		//remove_row(V, SVJ, flag);
		std::cout << "V.rows()" << V.rows();
		// 面の対応を修正(旧頂点番号を新頂点番号に変更)
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
// 重複頂点の削除
void remesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C) {
	clock_t start = clock();

	//- - - - - - 重複頂点の削除 - - - - - - - - - -
	int vn = V.rows();
	//http://sssiii.seesaa.net/article/434541169.html
	Eigen::MatrixXd SV;
	Eigen::MatrixXi SVI, SVJ;
	igl::remove_duplicate_vertices(V, 0.0000001, SV, SVI, SVJ);     //重複頂点の削除 SVに削除後の頂点

	if (V.rows() == SV.rows()) {
		std::cout << "不正頂点はありません." << std::endl;
		return;
	}
	else  std::cout << "不正頂点が" << V.rows() - SV.rows() << "個あります。修正します。" << std::endl;

	V = SV;

	//- - - - - 面を構成する頂点のインデックスを更新 - - - - - 
	// SVJに元の頂点番号→削除後の頂点番号が格納されているので、
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
	int count = 0;
	std::vector<int> collapsed_edge;
	for (int i = 0; i < max_iter; i++) {

		igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);

		collapsed_edge.push_back(Q[i].index);	// legacy
		count++;
	}
	std::cout << count << "個の辺を省略しました．\n";

	// 面を再構成:重複面の削除
	if (is_remesh) {
		remesh(V, F);
	}
}

void temp(std::vector< edge_info > &Q, Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXi &E, Eigen::VectorXi &EMAP, Eigen::MatrixXi &EF, Eigen::MatrixXi &EI) {
	// - - - - - 辺の長さと中点を調べる - - - - - - -
	Q = {};
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);

		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}
	std::sort(Q.begin(), Q.end(), cmp);		// 比較関数cmpを使用してsort
}

// 範囲を指定してリダクション
// prim_mask(0or1)が1の面から，順番に削減していく
// 最も短い辺から頂点を結合
// ただし位置を同じにするだけなので、面の再構成が必要
// 一つの辺を消した後，長さ順位を更新してないから一度にたくさん消すと微妙な感じになる．
void limited_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask,  // 面番号→可視性のマップ
	const double ratio,				   // prim_mask が1の部分のうち何割をさくげんするか
	bool is_remesh
) {
	std::cout << "範囲を指定してリダクション" << std::endl;

	// 辺行列を作成
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	igl::edge_flaps(F, E, EMAP, EF, EI);

	
	// - - - - - 辺の長さと中点を調べる - - - - - - -
	std::vector< edge_info > Q;
	for (int e = 0; e < E.rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		igl::shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		
		edge_info Qelement = { e, cost, p };
		Q.push_back(Qelement);
	}
	std::sort(Q.begin(), Q.end(), cmp);		// 比較関数cmpを使用してsort

	
	// - - - - - - prim_maskから頂点のmaskを作成 - - - - - - - - - - 
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

	// max_iter : 削減する辺の数
	// prim_mask が1の部分のうち何割をさくげんするか
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
	int count = 0;  // 省略された辺の数
	
	   
	while (count < max_iter) {
		//std::cout << "cllps " << i << std::endl;
		if (vert_mask(E(Q[i].index, 0)) != 1 && vert_mask(E(Q[i].index, 1)) != 1) { //辺の両端がマスク内の頂点かどうか

			//Eigen::MatrixXi uE;
			//igl::edge_flaps(F, uE, EMAP, EF, EI);
			//std::cout << "[" << Q[i].index  << "]" << EF(Q[i].index, 0) << "/" << EF(Q[i].index, 1) << std::endl;

			igl::collapse_edge(Q[i].index, Q[i].midpoint, V, F, E, EMAP, EF, EI);

			collapsed_edge.push_back(Q[i].index);
			count++;

			// 再度ソート
			//temp(Q, V, F, E, EMAP, EF, EI);
			//std::cout << count << std::endl ;
		}
		i++;
		if (i >= E.rows()) {
			std::cout << "ERROR" << std::endl;
			break;
		}
	}
	std::cout << count << "個の辺を省略しました" << std::endl;

	// 面を再構成:重複面の削除
	if (is_remesh) {
		remesh(V, F);
	}
}

void multiple_area_poly_reduction(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	const Eigen::VectorXi& prim_mask, // 段階付き可視性マップ
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
	const Eigen::MatrixXi& InfRefMAP, // 段階付き可視性マップ
	std::vector<RowVecter2i_Float> & filter,
	bool is_remesh) 
{
	// Filterで指定した割合で削減を行う(filter[i].afterは削減する面の割合)
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

/* ライブラリを使用したリダクション
   おそらく毎回ソートしている？ */
void temp2(Eigen::MatrixXd &V, Eigen::MatrixXi &F, float ratio) {
	using namespace std;
	using namespace Eigen;
	using namespace igl;

	std::cout << "頂点を削減します" << std::endl;
  
  
	// Prepare array-based edge data structures and priority queue
	VectorXi EMAP;
	MatrixXi E,EF,EI;
	typedef std::set<std::pair<double,int> > PriorityQueue;	// std::set 自動でソートしてくれる型
	PriorityQueue Q;
	std::vector<PriorityQueue::iterator > Qit;
	
	MatrixXd C;												// 辺の中心座標
	int num_collapsed;										// 削減した頂点の数

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
		// 挿入した値へのイテレータ，つまりQit[e]はQの辺ｅについてのデータへのイテレータ
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
			// 省略した辺はＱから削除される．(Qはそのうえで自動でソート)
			// 省略のたび，shortest_edge_and_midpointでQが更新される．→ここに重みをつければ...?
			// Fも補正され，省略された面は[0,0,0]になる．
			// 省略されたVは孤立頂点になる.
			// 詳しくはメモ参照
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