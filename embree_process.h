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

// Colorub^Ζέ·
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

	// bVΜo^
	void mesh_registration(Eigen::MatrixXd V, Eigen::MatrixXi F);

	// CΖΜπ·»θ(½ΛAόά³΅)
	void intersection_decision();

	// ΚΤ¨Β«Μ}bvπμ¬
	void intersection_decision(Eigen::VectorXi &VisiblityMAP);

	// CΖΜπ·»θ(κΒΜέeXgp)
	void intersection_decision_test();

	// ζπΫΆ
	void save_image();

	Vector3 calcViewingRay(int ix, int iy, int width, int height);

};


