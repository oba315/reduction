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

// Colorub�^�ƌ݊�
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

	// ���b�V���̓o�^
	void mesh_registration(Eigen::MatrixXd V, Eigen::MatrixXi F);

	// ���C�Ƃ̌�������(���ˁA���ܖ���)
	void intersection_decision();

	// �ʔԍ��������̃}�b�v���쐬
	void intersection_decision(Eigen::VectorXi &VisiblityMAP);

	// ���C�Ƃ̌�������(��̂݃e�X�g�p)
	void intersection_decision_test();

	// �摜��ۑ�
	void save_image();

	Vector3 calcViewingRay(int ix, int iy, int width, int height);

};


