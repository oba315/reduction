#pragma once

#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#include "my_structs.h"
#include "image.h"
#include <iostream>
#include "myScene.h"

#ifndef IGL_NO_EIGEN
#  include <Eigen/Core>
#endif

// Colorub型と互換
#define COL_RED    {255,0,0}
#define COL_BLACK  {0,0,0}
#define COL_WHITE  {255,255,255}

class embree_process
{
private:
	RTCDevice device;
	RTCScene scene;
	Camera cam;
	Image image;

	
public:
	embree_process(Camera cam, Image image);
	virtual ~embree_process();

	// メッシュの登録
	void mesh_registration(Eigen::MatrixXd V, Eigen::MatrixXi F);

	// レイとの交差判定(反射、屈折無し)
	void intersection_decision();

	// 面番号→可視性のマップを作成
	void intersection_decision(Eigen::VectorXi &VisiblityMAP);

	// レイとの交差判定(一つのみテスト用)
	void intersection_decision_test();

	// 画像を保存
	void save_image();

	Vector3 calcViewingRay(int ix, int iy, int width, int height);

};


