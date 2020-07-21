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
#include "matrix_tools.h"
#include "poly_delete.h"
#include "get_single_lay.h"
//#include <igl/png/readPNG.h>

//Eigen::MatrixXd V;
//Eigen::MatrixXi F;

#define mesh_dir_path std::string("C:/Users/piedp/Documents/Resources/mesh/primitive/")


void rendering(Image& image, MyScene myscene) {
	clock_t start = clock();

	ren::crossing_judgement(image, *myscene.rtcscene_ptr, myscene);

	char picturename[] = "out.png";
	image.save_png(picturename);

	clock_t end = clock();
	const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
	std::cout << "time " << time << "[ms]" << std::endl;

	
}

// -1を０，その他を小さいほうから赤→青にマッピング
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


// -----グリッドの追加-------------------------------
void show_grid(
			igl::opengl::glfw::Viewer& viewer,
			int data_id,
			const int grid_size = 50,			// グリッドのサイズ) 
			const int grid_unit = 1,			// グリッドの最小単位
			const int grid_emph = 10			// 強調したい単位
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
	viewer.selected_data_index = 0;		// idを戻す
}


// --------------- 単一レイの表示 -------------------------------------
int single_lay(igl::opengl::glfw::Viewer& viewer,
					MyScene& myscene,
					Image& image,
					Eigen:: RowVector3d col,
					unsigned int x, unsigned int y) {
	
	std::vector<Vector3>single_ray = get_single_lay(x, y, image, *myscene.rtcscene_ptr, myscene);
	viewer.append_mesh();				
	int id = viewer.data().id;
	for (int i = 0; i < single_ray.size(); i += 2) {
		Eigen::RowVector3d sp;
		sp << single_ray[i].x, single_ray[i].y, single_ray[i].z;
		Eigen::RowVector3d ep;
		ep << single_ray[i + 1].x, single_ray[i + 1].y, single_ray[i + 1].z;
		viewer.data().add_edges(sp, ep, col);
	}
	viewer.selected_data_index = 0;		// idを戻す
	return id;
}
// --------------------------------------------------------------------
// --------------- レイとの交点における法線の表示 -------------------------------------
int ray2nom( int inc, // 使用する交点の解像度(何個おきに計算するか) 
	igl::opengl::glfw::Viewer& viewer,
	MyScene& myscene,
	Image& image,
	Eigen::RowVector3d col
	) {

	std::vector<Vector3>single_ray = show_normal(inc, image, *myscene.rtcscene_ptr, myscene);
	viewer.append_mesh();
	int id = viewer.data().id;
	
	for (int i = 0; i < single_ray.size(); i += 2) {
		Eigen::RowVector3d sp;
		sp << single_ray[i].x, single_ray[i].y, single_ray[i].z;
		Eigen::RowVector3d ep;
		ep << single_ray[i + 1].x, single_ray[i + 1].y, single_ray[i + 1].z;
		viewer.data().add_edges(sp, ep, col);
	}
	viewer.selected_data_index = 0;		// idを戻す
	return id;
}
// --------------------------------------------------------------------

int main(int argc, char* argv[])
{

	// ----- シーンの作成 --------
	MyScene myscene;
	
	// -------- オブジェクトの読み込み ----------------
	std::string mesh_path = "C:/Users/piedp/Documents/Resources/mesh/river/river_cut.obj";
	//std::string mesh_path = "C:/Users/piedp/Documents/Resources/mesh/primitive/sphere_big.obj";
	//MyObject water(mesh_dir_path + "water_test.obj", false);
	MyObject water(mesh_path, false);
	//MyObject water("C:/Users/piedp/Documents/Resources/mesh/river/reduct.3.3.obj", false);
	//MyObject water("C:/Users/piedp/Documents/Resources/mesh/primitive/cube.obj", false);
	water.mat.IOR = 1.4;
	water.mat.transparency = 1.;
	water.mat.reflectivity = 0;
	water.color = COL_WATER;
	water.smooth_shading = true;
	myscene.add_object_to_scene(water);
	MyObject* mesh1 = &water;
	
	water.mat.IOR = 1.9;
	
	std::cout << &water << " " << &(myscene.object[0]) << std::endl;

	

	std::cout << "ERROR : F.cols = " << water.F.cols() << std::endl;

	// -------------------------------------------------

	// ----- 環境マップ --------------
	std::string image_path = "C:/Users/piedp/Documents/Resources/HDR/Arches_E_PineTree/Arches_E_PineTree/8k.bmp";
	myscene.add_texture(image_path);
	std::cout << "LL" << (*myscene.texture[myscene.texture.size() - 1]).getWidth() << std::endl;
	(*(myscene.texture[myscene.texture.size() - 1])).setType("ENV");
	myscene.use_environment_map = true;
	myscene.envmap = myscene.texture[myscene.texture.size() - 1];


	// --------- カメラの設定 --------------------------
	myscene.make_camera();
	myscene.camera.view   = Vector3(20, 0, 0);	    // 視点
	myscene.camera.refe   = Vector3(0, 0.0, 0.0);      // 注視点
	myscene.camera.up     = Vector3(0, 1.0, 0.0);      // アップベクトル？			
	myscene.camera.fovy   = 45.0 / 180.0 * M_PI;
	Image image( 1024, 1024 );				// 画像出力用
	// -------------------------------------------------


	embree_process embtest(myscene.camera, image );
	embtest.mesh_registration(water.V, water.F);

	
	
	// デバイスを作成 //
	RTCDevice device = rtcNewDevice(NULL);
	// シーンを作成 //
	RTCScene scene = rtcNewScene(device);

	myscene.add_to_embree_scene(device, scene);

	crossing_judgement(image, scene, myscene);	// 交差判定
	
	// レンダリングを行う
	rendering(image, myscene);
	


	// ------- 色の変更 -------------
	// BLKに初期化(1:white)
	water.C = Eigen::MatrixXd::Constant(water.F.rows(), 3, 0);
	
	// 仮
	Eigen::VectorXi VisblityMap = water.InfRefMAP.col(0);
	Eigen::VectorXi VisblityMap2 = water.InfRefMAP.col(1);

	replace_exception_row(water.RayStrengthMAP, 0, water.RayStrengthMAP.minCoeff());
	replace_exception_row(water.RayLengthMAP, 0, water.RayLengthMAP.maxCoeff());
	//inflectionmap_to_color(VisblityMap, water.C);
	//water.normalize_RayLengthMAP(); 
	//igl::jet(water.RayLengthMAP, 0, 1, water.C);
	//water.C.col(0) = water.RayStrengthMAP;

	igl::jet(water.RayLengthMAP, water.RayLengthMAP.minCoeff(), water.RayLengthMAP.maxCoeff(), water.C);
	//igl::jet(water.RayStrengthMAP, water.RayStrengthMAP.minCoeff(), water.RayStrengthMAP.maxCoeff(), water.C);
	
	/*
	Eigen::VectorXd positionMAP(water.F.rows());
	for (int i = 0; i < water.F.rows(); i++) {
		positionMAP(i) = water.V(water.F(i, 0), 0) + water.V(water.F(i, 1), 0) + water.V(water.F(i, 2), 0) / 3;
	}
	positionMAP = (positionMAP.array() - positionMAP.minCoeff()).array() / (positionMAP.maxCoeff() - positionMAP.minCoeff());
	*/
	//igl::jet(positionMAP, positionMAP.minCoeff(), positionMAP.maxCoeff(), water.C);
	// ------------------------------

	//std::cout << water.RayLengthMAP.topRows(100) << std::endl;
	//std::cout << water.RayStrengthMAP.topRows(100) << std::endl;


	//test
	/*
	Eigen::VectorXi mask = Eigen::VectorXi::Zero(water.F.rows());
	for (int i = 0; i < water.F.rows(); i++) {
		//if (i > 0&&i<200000)
		if (water.InfRefMAP(i, 0) != 0)
			mask[i] = 1;
	}
	std::cout << "befor " << water.F.rows() << std::endl;
	poldel::poly_delete(water.V, water.F, water.C, mask);
	std::cout << "after " << water.F.rows() << std::endl;
	//RTCDevice device = rtcNewDevice(NULL);
	//RTCScene scene = rtcNewScene(device);
	myscene.add_to_embree_scene(device, scene);
	*/


	//poly_reduction(V, F, 0.1);
	// switchの中では宣言できない？
	double ratio = 0.1;
	clock_t render_start;
	clock_t render_end;
	double render_time;
	bool   exist_grid;
	int    grid_id;
	Eigen::VectorXd mask;
	Eigen::VectorXi deletemask = Eigen::VectorXi::Zero(water.F.rows());
	std::vector<RowVecter2i_Float> filter;

	// キー操作
	const auto& key_down =
		[&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
	{
		switch (key)
		{
		case ' ':
			std::cout << "spacekey pressed : ポリゴンを削減します" << std::endl;
			
			for (int i = -1; i < 4; i++) filter.push_back({ {-1,i}, 0.8 });
			for (int i = -1; i < 4; i++) filter.push_back({ {3,i}, 0.6 });
			for (int i = -1; i < 4; i++) filter.push_back({ {2,i}, 0.4 });
			for (int i = -1; i < 4; i++) filter.push_back({ {1,i}, 0.2 });
			

			multiple_area_poly_reduction(water.V, water.F, water.InfRefMAP, filter, false);
			ratio + 0.1;
			std::cout << "V.rows() : " << water.V.rows() << std::endl;
			std::cout << "F.rows() : " << water.F.rows() << std::endl;
			//std::cout << "F :" << F << std::endl;
			//std::cout << "V :" << V << std::endl;


			viewer.data().clear();
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			water.update_embree_scene(device, scene);
			break;

		case '9':
			// 全体を削減
			std::cout << "9 pressed : ポリゴンを削減します" << std::endl;
			//limited_area_poly_reduction(water.V, water.F, mask, 0.5, false);
			//poly_reduction(water.V, water.F, 0.5, false);
			mask = Eigen::VectorXd::Zero(water.InflectionMAP.rows());
			for (int i = 0; i < water.InfRefMAP.rows(); i++) {
				if (water.InfRefMAP(i, 0) == -1) mask(i) = 1.;
			}
			mask = Eigen::VectorXd::Ones(water.F.rows());
			reduction(water.V, water.F, 0.8, mask);
			//reduction(water.V, water.F, 0.8, positionMAP);
			ratio + 0.1;
			std::cout << "V.rows() : " << water.V.rows() << std::endl;
			std::cout << "F.rows() : " << water.F.rows() << std::endl;
			//std::cout << "F :" << F << std::endl;
			//std::cout << "V :" << V << std::endl;


			viewer.data().clear();
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			std::cout << "add\n";
			water.update_embree_scene(device, scene);
			break;

		

		case 'j':
		case 'J':
			std::cout << "到達しなかった面を削除します" << std::endl;
			for (int i = 0; i < water.F.rows(); i++) {
				if (water.InfRefMAP(i,0) == -1) {
					deletemask(i) = 1;
				}
			}
			poldel::poly_delete(water.V, water.F, water.C, deletemask);
			// libiglシーンの更新
			viewer.selected_data_index = 0;
			viewer.data().clear();
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			// embreeの更新
			water.update_embree_scene(device, scene);
			break;


		case 'D':
		case 'd':
			std::cout << "Dkey pressed : 不正頂点を削除します" << std::endl;
			remesh(water.V, water.F, water.C);
			viewer.data().clear();
			viewer.data().set_mesh(water.V, water.F);
			viewer.data().set_colors(water.C);
			viewer.data().set_face_based(true);
			break;

		case 'R':
		case 'r':
			std::cout << "Rkey pressed : メッシュを再読み込みします" << std::endl;
			igl::readOBJ(mesh_path, water.V, water.F);
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
			std::cout << "Pkey pressed : レンダリングします" << std::endl;
			rendering(image, myscene);
			break;

		case 'C':
		case 'c':
			std::cout << "C : モデルの状態検証を行います" << std::endl;
			for (MyObject* o : myscene.object) {
				(*o).model_checker();
			}
			std::cout << "Clear" << std::endl;
			myscene.scene_info();
			break;

		case 'G':
		case 'g':
			std::cout << "グリッドの表示を切り替えます" << std::endl;
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
			std::cout << "メッシュを保存します" << std::endl;
			igl::writeOBJ("./mesh/out.obj", water.V, water.F);
			std::cout << "メッシュを保存しました : . / mesh / out.obj" << std::endl;
			
		default:
			return false;
		}
		return true;
	};
	

	// ==== libiglシーンへのメッシュ登録 ====

	// ----water---------------------------------------
	// data().id = 0
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(water.V, water.F);
	viewer.data().set_colors(water.C);
	// ------------------------------------------------
	
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

	// カメラ視線を表示
	viewer.data().add_edges(
		Eigen::RowVector3d(myscene.camera.view.x, myscene.camera.view.y, myscene.camera.view.z),
		Eigen::RowVector3d(myscene.camera.refe.x, myscene.camera.refe.y, myscene.camera.refe.z),
		Eigen::RowVector3d(1, 0, 0)
	);
	
	viewer.selected_data_index = 0;		// idを戻す
	// --------------------------------------------------
	
	// -------------------------------------------------
	// グリッドの作成
	viewer.append_mesh();				// id : 2
	grid_id = viewer.data().id;		// 現在のdata().id
	show_grid(viewer, grid_id);	
	exist_grid = true;
	viewer.selected_data_index = 0;		// idを戻す
	// -------------------------------------------------

	// --------------------------------------------------
	// 単一レイの表示					
	Eigen::RowVector3d col;
	col << 1, 1, 1;
	single_lay(viewer, myscene, image, col, 511, 550);  // id : 3
	col << 1, 0, 0;
	single_lay(viewer, myscene, image, col, 513, 550);  // id : 4
	// --------------------------------------------------
    // 法線の表示
	col << 0, 1, 0;
	//ray2nom(20, viewer, myscene, image, col);

	viewer.callback_key_down = key_down;
	myscene.scene_info();
	viewer.launch();

	//rendering(image, myscene);

	std::cout << "a";
	
}
