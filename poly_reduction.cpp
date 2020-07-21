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

#include "my_collapse_edge.h"



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
	else  std::cout << "不正頂点が" << V.rows() - SV.rows() << "個あります。修正しました。" << std::endl;
	
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
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	// - - - - - 構成頂点に同一の頂点を含む面の削除 - - - - - - -
	std::set<int> used_vid;
	std::vector<double> tempvec;
	for (int i = 0; i < F.rows(); i++) {
		if (F(i, 0) != F(i, 1) && F(i, 1) != F(i, 2) && F(i, 2) != F(i, 0)) {
			tempvec.push_back(F(i, 0));
			tempvec.push_back(F(i, 1));
			tempvec.push_back(F(i, 2));
		}
		used_vid.insert(F(i, 0));
		used_vid.insert(F(i, 1));
		used_vid.insert(F(i, 2));
	}
	Eigen::MatrixXi temp = VecToMat(tempvec);
	F = temp;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	
	// - - - - - 孤立頂点の削除 - - - - - - - - - - - - - - - 
	int count = 0;
	Eigen::VectorXi mask = Eigen::VectorXi::Zero(V.rows());
	for (int i = 0; i < V.rows(); i++) {
		for (auto iter = used_vid.begin(); iter != used_vid.end(); ++iter) {
			if (i == *iter) {
				break;
			}
			if (i < *iter) {
				mask(i) = 1;
				count++;
				break;
			}
		}
	}
	std::cout << "孤立頂点が" << count << "個あります．修正します．" << std::endl;
	

	std::cout << "	finish (" << vn << "->" << V.rows() << ")\n	   time:" << clock() - start << "[ms]" << std::endl;
	
	
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

	std::set<int> used_vid;
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
		used_vid.insert(F(i, 0));
		used_vid.insert(F(i, 1));
		used_vid.insert(F(i, 2));
	}
	Eigen::MatrixXi temp = VecToMat(tempvec);
	Eigen::MatrixXd tempC = VecToMatXd(tempvecC);
	F = temp;
	C = tempC;

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

	// - - - - - 孤立頂点の削除 - - - - - - - - - - - - - - - 
	int count = 0;
	std::vector<int> mask;
	for (int i = 0; i < V.rows(); i++) {
		for (auto iter = used_vid.begin(); iter != used_vid.end(); ++iter) {
			if (i == *iter) {
				break;
			}
			if (i < *iter) {
				mask.push_back(i);
				count++;
				break;
			}
		}
	}
	std::cout << "孤立頂点が" << count << "個あります．修正します．" << std::endl;
	remove_row(V, SVJ, mask);
	//- - - - - 面を構成する頂点のインデックスを更新 - - - - - 
	// SVJに元の頂点番号→削除後の頂点番号が格納されているので、
	// ループを回さず一括処理する方法があるはず？
	
	for (int i = 0; i < F.rows(); i++)
	{
		for (int j = 0; j < F.cols(); j++)
		{
			int temp = F(i, j);
			F(i, j) = SVJ(temp,0);
		}
	}
	std::cout << F.rows() << " " << F.cols() << " " << F(100, 2);
	std::cout << "	finish (" << vn << "->" << V.rows() << ")\n	   time:" << clock() - start << "[ms]" << std::endl;
}




// 範囲を指定してリダクション
// prim_mask(0or1)が1の面から，順番に削減していく
// 最も短い辺から頂点を結合
// ただし位置を同じにするだけなので、面の再構成が必要
// 一つの辺を消した後，長さ順位を更新してないから一度にたくさん消すと微妙な感じになる．
// 毎回ループしていないのでRegacy
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
   毎回ソートしている 範囲無し*/
void reduction(Eigen::MatrixXd &V, Eigen::MatrixXi &F, float ratio, Eigen::VectorXd face_weight) {
	using namespace std;
	using namespace Eigen;
	using namespace igl;

	// - - - - - - 宣言 - - - - - - - - - - - - - - - - - 
	VectorXi  EMAP;
	MatrixXi  E,EF,EI;
	MatrixXd  C;												// 辺の中心座標
	typedef std::set<std::pair<double,int> > PriorityQueue;	    // std::set 自動でソートしてくれる型
	PriorityQueue Q;
	std::vector<PriorityQueue::iterator > Qit;					// 挿入した値へのイテレータ，つまりQit[e]はQの辺ｅについてのデータへのイテレータ
	int  num_collapsed       = 0;   							// 削減した頂点の数
	bool something_collapsed = false;
	// - - - - - - - - - - - - - - - - - - - - - - - - - -
	const int max_iter =  std::ceil(ratio * float(V.rows()));
	std::cout << "頂点を削減します   max_iter : " << max_iter << std::endl;

	// - - - - - - 初期化 - - - - - - - - - - - - - - - - - - - - - -
    edge_flaps(F,E,EMAP,EF,EI);
	int count_half_edge = 0;
	if (EF.minCoeff() == -1) {
		std::cout << "ERROR : 片翼の辺があります．" << std::endl;
		count_half_edge += 1;
	}
	Qit.resize(E.rows() - count_half_edge);
	C.resize(E.rows() ,V.cols());
	VectorXd costs(E.rows());
	Q.clear();
	for (int e = 0; e < E.rows(); e++)
	{
		double cost = e;
		RowVectorXd p(1, 3);
		shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		C.row(e) = p;
		if (EF.row(e).minCoeff() == -1) continue;
		Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;	
	}
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	std::cout << "--- ";

	// collapse edge
	int count = 0;
	while (num_collapsed < max_iter )
	{
		if (mycollapse::collapse_edge(
			shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, Qit, C, face_weight)
			// 省略した辺はＱから削除される．(Qはそのうえで自動でソート)
			// 省略のたび，shortest_edge_and_midpointでQが更新される．→ここに重みをつければ...?
			// Fも補正され，省略された面は[0,0,0]になる．
			// 省略されたVは孤立頂点になる.
			// 詳しくはメモ参照
			)
		{
			num_collapsed++;
		}
	}
	if (num_collapsed > 0) something_collapsed = true;
	
	
	// 削減後の辺の数．
	int counts = 0;
	for (int i = 0; i < F.rows(); i++) {
		if (!(F(i, 0) == 0 && F(i, 1) == 0)) { counts++; }
	}
	std::cout << num_collapsed << "個の頂点を省略しました．(" << V.rows() <<  "->" << V.rows()-num_collapsed << ")" <<  std::endl;
	
}

