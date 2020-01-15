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
#include "myScene.h"
#include "crossing_judgment_with_mapping.h"
#include <random>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

#define mesh_dir_path std::string("../../../mesh/")

void inflectionmap_to_color(Eigen::VectorXi &InflectionMAP, Eigen::MatrixXd &C) {
	
	

	int max = InflectionMAP.maxCoeff();
	Eigen::VectorXd tempMAP(InflectionMAP.rows()) ;
	for (int i = 0; i < InflectionMAP.rows(); i++) {
		if (InflectionMAP(i) == -1) {
			tempMAP(i) = 0.;
		}
		else {
			tempMAP(i) = double ( (max + 1) - InflectionMAP(i));
			//std::cout << (double((max + 1) - InflectionMAP(i))) << " ";
		}
	}
	//std::cout << tempMAP << std::endl;

	//tempMAP = tempMAP / double(max + 1);
	//std::cout << tempMAP << std::endl;
	igl::jet(tempMAP, 0, max + 1, C);
	
}

int main(int argc, char* argv[])
{
	// ----- シーンの作成 --------
	MyScene myscene;
	
	// -------- オブジェクトの読み込み ----------------
	
	MyObject water(mesh_dir_path + "water_test_lit.obj", false);
	water.mat.IOR = 1.0;
	water.mat.transparency = 0.8;
	myscene.add_object_to_scene(water);
	
	myscene.scene_info();

	F = water.F;
	V = water.V;

	// -------------------------------------------------

	// --------- カメラの設定 --------------------------
	myscene.make_camera();
	myscene.camera.view   = Vector3(0.0, 1.5, 5);	    // 視点
	myscene.camera.refe   = Vector3(0, 0.0, 0.0);      // 注視点
	myscene.camera.up     = Vector3(0, 1.0, 0.0);      // アップベクトル？			
	myscene.camera.fovy   = 45.0 / 180.0 * M_PI;
	Image image( 512, 512 );				// 画像出力用
	// -------------------------------------------------


	embree_process embtest(myscene.camera, image );
	embtest.mesh_registration(water.V, water.F);

	Eigen::VectorXi VisiblitiMAP = Eigen::VectorXi::Zero(water.F.rows());
	embtest.intersection_decision(VisiblitiMAP);
	//embtest.save_image();
	//std::cout << "Visiblity map" << VisiblitiMAP << std::endl;
	//delete embtest;

	// 最終的にVisiblityMapがつくれればいい？
	
	// デバイスを作成 //
	RTCDevice device = rtcNewDevice(NULL);
	// シーンを作成 //
	RTCScene scene = rtcNewScene(device);

	myscene.add_to_embree_scene(device, scene);

	crossing_judgement(image, scene, myscene);	// 交差判定

	//std::cout << myscene.object[0].VisibilityMAP << std::endl;
	
	/*
	for (int i = 0; i < myscene.object[0].VisibilityMAP.rows(); i++) {
		std::cout << myscene.object[0].VisibilityMAP(i) << " ";
	}*/

	// ------- 色の変更 -------------
	// 白に初期化
	Eigen::MatrixXd C = Eigen::MatrixXd::Constant(water.F.rows(), 3, 1);
	for (int i = 0; i < water.VisibilityMAP.rows(); i++) {
		if (myscene.object[0].VisibilityMAP(i) > 1) {
			//std::cout << ":" ;
			C.row(i) << 1, 0, 0;
		}
		else
		{
			//std::cout << "[" << myscene.object[0].VisibilityMAP(i) << "] ";
			C.row(i) << myscene.object[0].VisibilityMAP(i), 0, 0;
		}
	}
	// ------------------------------
	Eigen::VectorXd J;
	J.resize(F.rows());
	for (int i = 0; i < J.rows(); i++) {
		J(i) = double (std::rand());
	}
	igl::jet(J, true, C);
	igl::jet(myscene.object[0].VisibilityMAP, 0, 1, C);
	// std::cout << myscene.object[0].InflectionMAP << std::endl;
	inflectionmap_to_color(myscene.object[0].InflectionMAP, C);

	//poly_reduction(V, F, 0.1);
	double ratio = 0.1;
	// キー操作
	const auto& key_down =
		[&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
	{
		switch (key)
		{
		case ' ':
			std::cout << "spacekey pressed : ポリゴンを削減します" << std::endl;
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
			std::cout << "Rkey pressed : メッシュを再読み込みします" << std::endl;
			igl::readOBJ(mesh_dir_path, V, F);
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
	// カメラオブジェクトの作成
	viewer.append_mesh();		      // id : 1

	Eigen::MatrixXd camV;
	Eigen::MatrixXi camF;
	igl::readOBJ(mesh_dir_path + "camera_wir.obj", camV, camF);

	Eigen::Vector3d m = { myscene.camera.view.x,myscene.camera.view.y,myscene.camera.view.z };
	move_mash(camV, m);

	viewer.data().clear();
	viewer.data().set_mesh(camV, camF);


	viewer.data().add_edges(
		Eigen::RowVector3d(myscene.camera.view.x, myscene.camera.view.y, myscene.camera.view.z),
		Eigen::RowVector3d(myscene.camera.refe.x, myscene.camera.refe.y, myscene.camera.refe.z),
		Eigen::RowVector3d(1, 0, 0)
	);

	viewer.selected_data_index = 0;		// idを戻す
	// --------------------------------------------------
	
	viewer.launch();

	std::cout << "a";
	
}
