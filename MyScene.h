#pragma once
#include <igl/readOBJ.h>
#include <iostream>
#include <vector>
#include <string>
#include "Vector3.h"
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#include "image.h"

// Colorub�^�ƌ݊�
#define COL_RED    {255,0,0}
#define COL_BLACK  {0,0,0}
#define COL_WHITE  {255,255,255}
#define COL_ERROR  {236,113,240}
#define COL_BACKGOUND {107,124,143}


#ifndef STRUCT_CAMERA	// �t���O�������Ă��Ȃ���΁i if HEADER_H is not defined �j {
#define STRUCT_CAMERA	//     �t���O�𗧂Ă�i define HEADER_H �j
typedef struct {
	Vector3	view;
	Vector3	refe;
	Vector3	up;
	double	fovy;
} Camera;		//     }
#endif	




// - - - ���b�V�����ƃ}�e���A�������܂Ƃ߂��N���X - - - - - - - - - - - - - 
class MyObject {
private:
	
	typedef struct {
		double transparency = 0;	// ���ߓx:0�Ŕ񓧉�
		double IOR = 1;				// ���ܗ�
	}MyMaterial;

	int index = -1;
	int geomID = -1; // embree�ł̎���ID
	
public:
	Eigen::MatrixXd V;			// ���_�̈ʒu���
	Eigen::MatrixXi F;			// �ʂ��\�����钸�_�̃C���f�b�N�X
	Eigen::MatrixXd TC, N;
	Eigen::MatrixXi FTC, FN;

	Eigen::VectorXi InflectionMAP;  // ���ܗ��̃}�b�s���O�@�Փ˖����F-1, ���ܖ���:0, ���܉�:int
	Eigen::VectorXd VisibilityMAP;	// �ʁ����o�I�d�݁@�̃}�b�s���O

	RTCGeometry geom;
	MyMaterial mat;

	bool has_texture = false;
	Colorub color = COL_RED;
	bool smooth_shading = true;

	//  obj�t�@�C���̓ǂݍ��� (texture����mtl�t�@�C������ǂ񂾕����X�}�[�g)
	//  ���V�[���֓o�^���Ă�������.
	MyObject(std::string path, bool has_texture = false);

	void print_info();

	//  embree�p����ID�̑���
	void set_geomID(int id);
	int get_geomID();

	void add_to_embree_scene(RTCDevice &device, RTCScene &scene);
	
};
// ---------------------------------------------------------------------------


// - - - �I�u�W�F�N�g���܂Ƃ߂��N���X - - - - - - - - - - - - - - - - - - - -
/*�@�e�N�X�`����MyScene�ȉ��ɔz�u����̂ŁA�Q�Ƃ������ꍇ�͂�������B
    �����I�ɂ̓I�u�W�F�N�g��MyScene�̃��[�J���ϐ��ɂ��������ǃI�u�W�F�N�g���ŌĂׂȂ��Ȃ�̂��ɂ���*/
class MyScene
{
private:
	bool camera_exist = false;
public:
	std::vector<MyObject> object;
	//std::vector<std::string> object_name;
	std::vector<Image> texture;

	Camera camera;

	void add_object_to_scene(MyObject &obj);

	void scene_info();

	/*�V�[�����ɂ���I�u�W�F�N�g��embree�I�u�W�F�N�g�ɕϊ����Aembree�̃V�[���ɓo�^*/
	void add_to_embree_scene(RTCDevice &device, RTCScene &scene);
	
	// geomID����MyObject�̌���
	// �A�h���X��Ԃ����ق��������̂ł́H
	int geomID_to_objectID(int id);

	int add_texture(std::string path);
	void make_camera();
	void add_camera(Camera &cam);
};
// ---------------------------------------------------------------------------
