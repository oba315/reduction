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
#include "crossing_judgment.h"
#include <time.h>

//Eigen::MatrixXd V;
//Eigen::MatrixXi F;

#define mesh_dir_path std::string("C:/Users/piedp/Documents/Resources/mesh/primitive/")


void rendering(Image& image, MyScene myscene) {
	clock_t start = clock();

	ren::crossing_judgement(image, *myscene.rtcscene_ptr, myscene);

	char picturename[] = "out.bmp";
	image.save(picturename);

	clock_t end = clock();
	const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
	std::cout << "time " << time << "[ms]" << std::endl;
}

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


// -----�O���b�h�̒ǉ�-------------------------------
void show_grid(
			igl::opengl::glfw::Viewer& viewer,
			int data_id,
			const int grid_size = 50,			// �O���b�h�̃T�C�Y) 
			const int grid_unit = 1,			// �O���b�h�̍ŏ��P��
			const int grid_emph = 10			// �����������P��
){
	viewer.selected_data_index = data_id;
	for (int i = 0; i < grid_size; i += grid_unit) {
		Eigen::RowVector3d grid_color;
		if (i % grid_emph == 0) grid_color << 1, 0, 0;
		else grid_color << 0, 0, 0;

		viewer.data().add_edges(
			Eigen::RowVector3d(-1 * grid_size, 0, i),
			Eigen::RowVector3d(grid_size, 0, i),
			grid_color
		);
		viewer.data().add_edges(
			Eigen::RowVector3d(-1 * grid_size, 0, -1 * i),
			Eigen::RowVector3d(grid_size, 0, -1 * i),
			grid_color
		);
		viewer.data().add_edges(
			Eigen::RowVector3d(i, 0, -1 * grid_size),
			Eigen::RowVector3d(i, 0, grid_size),
			grid_color
		);
		viewer.data().add_edges(
			Eigen::RowVector3d(-1 * i, 0, -1 * grid_size),
			Eigen::RowVector3d(-1 * i, 0, grid_size),
			grid_color
		);
	}

	viewer.selected_data_index = 0;		// id��߂�
	
}



int main(int argc, char* argv[])
{
	// ----- �V�[���̍쐬 --------
	MyScene myscene;
	
	// -------- �I�u�W�F�N�g�̓ǂݍ��� ----------------
	
	//MyObject water(mesh_dir_path + "water_test.obj", false);
	MyObject water("C:/Users/piedp/Documents/Resources/mesh/river/river_cut.obj", false);
	water.mat.IOR = 1.4;
	water.mat.transparency = 0.9	;
	water.mat.reflectivity = 0;
	water.color = COL_WATER;
	water.smooth_shading = true;
	myscene.add_object_to_scene(water);
	MyObject* mesh1 = &water;
	
	water.mat.IOR = 1.9;
	
	std::cout << &water << " " << &(myscene.object[0]) << std::endl;

	

	std::cout << "ERROR : F.cols = " << water.F.cols() << std::endl;

	// -------------------------------------------------

	// ----- ���}�b�v --------------
	std::string image_path = "C:/Users/piedp/Documents/Resources/HDR/Arches_E_PineTree/Arches_E_PineTree/8k.bmp";
	myscene.add_texture(image_path);
	std::cout << "LL" << (*myscene.texture[myscene.texture.size() - 1]).getWidth() << std::endl;
	(*(myscene.texture[myscene.texture.size() - 1])).setType("ENV");
	myscene.use_environment_map = true;
	myscene.envmap = myscene.texture[myscene.texture.size() - 1];


	// --------- �J�����̐ݒ� --------------------------
	myscene.make_camera();
	myscene.camera.view   = Vector3(20, 0, 0);	    // ���_
	myscene.camera.refe   = Vector3(0, 0.0, 0.0);      // �����_
	myscene.camera.up     = Vector3(0, 1.0, 0.0);      // �A�b�v�x�N�g���H			
	myscene.camera.fovy   = 45.0 / 180.0 * M_PI;
	Image image( 1024, 1024 );				// �摜�o�͗p
	// -------------------------------------------------


	embree_process embtest(myscene.camera, image );
	embtest.mesh_registration(water.V, water.F);

	
	
	// �f�o�C�X���쐬 //
	RTCDevice device = rtcNewDevice(NULL);
	// �V�[�����쐬 //
	RTCScene scene = rtcNewScene(device);

	myscene.add_to_embree_scene(device, scene);

	crossing_judgement(image, scene, myscene);	// ��������

	//std::cout << myscene.object[0].ReflectionMAP << std::endl;
	
	/*
	for (int i = 0; i < myscene.object[0].VisibilityMAP.rows(); i++) {
		std::cout << myscene.object[0].VisibilityMAP(i) << " ";
	}*/

	// ------- �F�̕ύX -------------
	// ���ɏ�����
	water.C = Eigen::MatrixXd::Constant(water.F.rows(), 3, 1);
	
	// ��
	Eigen::VectorXi VisblityMap = water.InflectionMAP;

	inflectionmap_to_color(water.InflectionMAP, water.C);
	// ------------------------------


	//poly_reduction(V, F, 0.1);
	double ratio = 0.1;
	clock_t render_start;
	clock_t render_end;
	double render_time;
	bool   exist_grid;
	int    grid_id;
	// �L�[����
	const auto& key_down =
		[&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
	{
		switch (key)
		{
		case ' ':
			std::cout << "spacekey pressed : �|���S�����팸���܂�" << std::endl;
			//poly_reduction(V, F, ratio);
			//limited_area_poly_reduction(V, F, VisiblitiMAP, ratio, false );
			multiple_area_poly_reduction(water.V, water.F, VisblityMap);
			ratio + 0.1;
			std::cout << "V.rows() : " << water.V.rows() << std::endl;
			std::cout << "F.rows() : " << water.F.rows() << std::endl;
			//std::cout << "F :" << F << std::endl;
			//std::cout << "V :" << V << std::endl;


			viewer.data().clear();
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			break;

		case 'D':
		case 'd':
			std::cout << "Dkey pressed : �s�����_���폜���܂�" << std::endl;
			remesh(water.V, water.F);
			viewer.data().clear();
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			break;

		case 'R':
		case 'r':
			std::cout << "Rkey pressed : ���b�V�����ēǂݍ��݂��܂�" << std::endl;
			igl::readOBJ(mesh_dir_path, water.V, water.F);
			ratio = 0.1;

			viewer.data().clear();
			std::cout << "V.rows() : " << water.V.rows() << std::endl;
			std::cout << "F.rows() : " << water.F.rows() << std::endl;
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			break;

		case 'P':
		case 'p':
			std::cout << "Pkey pressed : �����_�����O���܂�" << std::endl;
			rendering(image, myscene);
			break;

		case 'C':
		case 'c':
			std::cout << "C : ���f���̏�Ԍ��؂��s���܂�" << std::endl;
			for (MyObject* o : myscene.object) {
				(*o).model_checker();
			}
			break;

		case 'G':
		case 'g':
			std::cout << "�O���b�h�̕\����؂�ւ��܂�" << std::endl;
			if (exist_grid) {
				viewer.selected_data_index = grid_id;
				viewer.data().clear();
				viewer.selected_data_index = 0;
				exist_grid = false;
			}
			else {
				show_grid(viewer, grid_id);
				exist_grid = true;
			}


		case 'S':
		case 's':
			std::cout << "���b�V����ۑ����܂�" << std::endl;
			igl::writeOBJ("./mesh/out.obj", water.V, water.F);
			std::cout << "���b�V����ۑ����܂��� : . / mesh / out.obj" << std::endl;
			
		default:
			return false;
		}
		return true;
	};


	// Plot the mesh
	// data().id = 0
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(water.V, water.F);
	viewer.data().set_colors(water.C);

	
	// -------------------------------------------------
	// �J�����I�u�W�F�N�g�̍쐬
	viewer.append_mesh();		      // id : 1

	Eigen::MatrixXd camV;
	Eigen::MatrixXi camF;
	igl::readOBJ(mesh_dir_path + "camera_wir.obj", camV, camF);

	Eigen::Vector3d m = { myscene.camera.view.x,myscene.camera.view.y,myscene.camera.view.z };
	move_mash(camV, m);

	viewer.data().clear();
	viewer.data().set_mesh(camV, camF);

	// �J����������\��
	viewer.data().add_edges(
		Eigen::RowVector3d(myscene.camera.view.x, myscene.camera.view.y, myscene.camera.view.z),
		Eigen::RowVector3d(myscene.camera.refe.x, myscene.camera.refe.y, myscene.camera.refe.z),
		Eigen::RowVector3d(1, 0, 0)
	);
	
	
	std::vector<Eigen::RowVector3d> points;
	points.push_back(Eigen::RowVector3d(myscene.camera.view.x, myscene.camera.view.y, myscene.camera.view.z));
	crossing_judgement_test(image, scene, myscene, 217, 174, points);	// ��������test
	for (int i = 0; i < points.size(); i++) {
		std::cout << points[i] << std::endl;
	}
	for (int i = 0; i < points.size()-1; i++) {
		viewer.data().add_edges(
			points[i],
			points[i+1],
			Eigen::RowVector3d(0, 0, 0)
		);
	}
	viewer.selected_data_index = 0;		// id��߂�
	// --------------------------------------------------
	
	// -------------------------------------------------
	// �O���b�h�̍쐬
	viewer.append_mesh();				// id : 2
	grid_id = viewer.data().id;		// ���݂�data().id
	show_grid(viewer, grid_id);	
	exist_grid = true;
	viewer.selected_data_index = 0;		// id��߂�



	viewer.callback_key_down = key_down;
	myscene.scene_info();
	viewer.launch();

	//rendering(image, myscene);

	std::cout << "a";
	
}
