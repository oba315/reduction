#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <stdio.h>
#include "testClass.h"
#include "poly_reduction.h"
#include "vector3.h"
#include "my_structs.h"
#include "image.h"
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#include "embree_process.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;

#define mesh_dir_path std::string("../../../mesh/")



int main(int argc, char* argv[])
{
	// -------- �I�u�W�F�N�g�̓ǂݍ��� ----------------

	std::string mesh_data_path = mesh_dir_path + "water_test_lit.obj";
	
	igl::readOBJ(mesh_data_path, V, F);
	std::cout << "V.rows() : " << V.rows() << std::endl;
	std::cout << "F.rows() : " << F.rows() << std::endl;
	//std::cout << "V :" << V << std::endl;
	//std::cout << "F :" << F << std::endl;

	// -------------------------------------------------

	// --------- �J�����̐ݒ� --------------------------
	Camera cam;
	cam.view   = Vector3(0.0, 1.5, 5);	    // ���_
	cam.refe   = Vector3(0, 0.0, 0.0);      // �����_
	cam.up     = Vector3(0, 1.0, 0.0);      // �A�b�v�x�N�g���H			
	cam.fovy   = 45.0 / 180.0 * M_PI;
	Image image( 512, 512 );				// �摜�T�C�Y
	// -------------------------------------------------


	embree_process embtest(cam, image );
	embtest.mesh_registration(V, F);
	Eigen::VectorXi VisiblitiMAP = Eigen::VectorXi::Zero(F.rows());
	embtest.intersection_decision(VisiblitiMAP);
	//embtest.save_image();
	//std::cout << "Visiblity map" << VisiblitiMAP << std::endl;
	//delete embtest;

	
	// ------- �F�̕ύX -------------
	// ���ɏ�����
	Eigen::MatrixXd C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
	for (int i = 0; i < VisiblitiMAP.rows(); i++) {
		if (VisiblitiMAP(i) == 1) {
			//std::cout << " " << i ;
			C.row(i) << 1, 0, 0;
		}
	}
	// ------------------------------
	
	
	
	//poly_reduction(V, F, 0.1);
	double ratio = 0.1;
	// �L�[����
	const auto& key_down =
		[&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
	{
		switch (key)
		{
		case ' ':
			std::cout << "spacekey pressed : �|���S�����팸���܂�" << std::endl;
			//poly_reduction(V, F, ratio);
			limited_area_poly_reduction(V, F, VisiblitiMAP, ratio, false );
			ratio + 0.1;
			std::cout << "V.rows() : " << V.rows() << std::endl;
			std::cout << "F.rows() : " << F.rows() << std::endl;
			//std::cout << "F :" << F << std::endl;
			//std::cout << "V :" << V << std::endl;


			viewer.data().clear();
			viewer.data().set_mesh(V, F);
			viewer.data().set_colors(C);
			viewer.data().set_face_based(true);
			break;
		case 'R':
		case 'r':
			std::cout << "Rkey pressed : ���b�V�����ēǂݍ��݂��܂�" << std::endl;
			igl::readOBJ(mesh_data_path, V, F);
			ratio = 0.1;

			viewer.data().clear();
			std::cout << "V.rows() : " << V.rows() << std::endl;
			std::cout << "F.rows() : " << F.rows() << std::endl;
			viewer.data().set_mesh(V, F);
			viewer.data().set_colors(C);
			viewer.data().set_face_based(true);
			break;
		default:
			return false;
		}
		return true;
	};


	// Plot the mesh
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
	viewer.callback_key_down = key_down;

	viewer.data().set_colors(C);

	
	// -------------------------------------------------
	// �J�����I�u�W�F�N�g�̍쐬
	viewer.append_mesh();		      // id : 1

	Eigen::MatrixXd camV;
	Eigen::MatrixXi camF;
	igl::readOBJ(mesh_dir_path + "camera_wir.obj", camV, camF);

	Eigen::Vector3d m = { cam.view.x,cam.view.y,cam.view.z };
	move_mash(camV, m);

	viewer.data().clear();
	viewer.data().set_mesh(camV, camF);


	viewer.data().add_edges(
		Eigen::RowVector3d(cam.view.x, cam.view.y, cam.view.z),
		Eigen::RowVector3d(cam.refe.x, cam.refe.y, cam.refe.z),
		Eigen::RowVector3d(1, 0, 0)
	);

	viewer.selected_data_index = 0;		// id��߂�
	// --------------------------------------------------
	
	viewer.launch();

	std::cout << "a";
	
}
