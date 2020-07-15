#include "MyScene.h"
#include "vector3.h"
#include <time.h>

#include <igl/edge_flaps.h>


#define NAME(var) std::cout << #var << std::endl;


int geomIDtemp = 0;  // シーンのオブジェクト数がわかればいらなくなる



MyObject::MyObject(std::string path, bool has_texture) {
	
	this->has_texture = has_texture;

	if (has_texture == false) {
		igl::readOBJ(path, V, F);
	}
	else {
		igl::readOBJ(path, V, TC, N, F, FTC, FN);
	}

	// 各マップの初期化
	int len = this->F.rows();
	this->InflectionMAP  = Eigen::VectorXi::Zero(len) - Eigen::VectorXi::Ones(len); // -1
	this->ReflectionMAP  = Eigen::VectorXi::Zero(len) - Eigen::VectorXi::Ones(len); // -1
	this->InfRefMAP      = Eigen::MatrixXi::Zero(len, 2) - Eigen::MatrixXi::Ones(len, 2);
	this->VisibilityMAP  = Eigen::VectorXd::Zero(len); // 0
	this->RayLengthMAP   = Eigen::VectorXd::Zero(len); // 0
	this->RayStrengthMAP = Eigen::VectorXd::Zero(len);

	
}


// -------------------------------------------------------------------
void MyObject::print_info() {
	std::cout << "\n- - - Object Info - - -" << std::endl;
	std::cout << "name : " ;
	NAME(this);
	std::cout << "V.rows()        : " << V.rows() << std::endl;
	std::cout << "F.rows()        : " << F.rows() << std::endl;
	std::cout << "has_texture     : " << has_texture << std::endl;
	std::cout << "index           : " << index << std::endl;
	std::cout << "geomID          : " << geomID << std::endl;
	std::cout << "smooth shading  : " << smooth_shading << std::endl;
	if (geomID != -1) std::cout << "    -> embreeのシーンに登録済みです" << std::endl;
	std::cout << "material        : "<< std::endl;
	std::cout << " - transparency : " << mat.transparency << std::endl;
	std::cout << " - IOR          : " << mat.IOR << std::endl;
	std::cout << " - reflectivity : " << mat.reflectivity << std::endl;
	std::cout << "- - - - - - - - - - -" << std::endl;
}//-------------------------------------------------------------------


//--------------------------------------------------------------------
void MyObject::model_checker()
{
	/* 四角以上の面が存在していないか */
	if (F.cols() != 3) {
		std::cout << "エラー：三角面以外で構成されています．" << std::endl;
	}

	/* 辺の両端に面が存在しているか */
	Eigen::MatrixXi uE;
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi EF;
	Eigen::MatrixXi EI;
	igl::edge_flaps(F, uE, EMAP, EF, EI);
	for(int ii = 0; ii < EF.rows(); ii++ ){
		for( int jj = 0; jj < EF.cols(); jj++ ){
			if( EF(ii, jj) == -1 ) {
				
				std::cout << "辺" << ii << "が閉じていません F:" << EF(ii,0) << "/" << EF(ii,1) << std::endl;
			}
		}
	}
}//-------------------------------------------------------------------


// --------------------------------------------------------------------
void MyObject::set_geomID(int id) { geomID = id; }
int MyObject::get_geomID() { return(geomID); }
// ---------------------------------------------------------------------


// -- オブジェクトをembreeのシーンに登録 ---------------------------------
void MyObject::add_to_embree_scene(RTCDevice &device, RTCScene &scene) {
	int ntri = F.rows();      // シーン中の三角形の数
	int nvert = V.rows();     // シーン中の頂点の総数

	// ３角形ジオメトリを作成 
	RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
	this->geom_ptr = &geom;
	// 頂点座標をセット 
	int j = 0;
	float* vertices = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), nvert);
	for (int i = 0; i < nvert; i++) {
		vertices[j] = float(V(i, 0));
		vertices[j + 1] = float(V(i, 1));
		vertices[j + 2] = float(V(i, 2));
		j += 3;
	}

	// 面を構成する頂点番号をセット
	j = 0;
	unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), ntri);
	for (int i = 0; i < ntri; i++) {
		indices[j] = F(i, 0);
		indices[j + 1] = F(i, 1);
		indices[j + 2] = F(i, 2);
		// std::cout << "F?" << i<< " "<< sizeof(indices) <<" "<<F(i, 0) << F(i, 1) << F(i, 2) << " "<< indices[i]<<" "<< indices[i3 + 1] << std::endl;
		j += 3;
	}

	/* シーンへ登録 */
	rtcCommitGeometry(geom);
	this->geomID = rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);
	rtcCommitScene(scene);

	// geomを登録...
	this->geom = geom;
	// geomIDを設定
	//geomID = geomIDtemp;
	//geomIDtemp++;
}// --------------------------------------------------------------------

// --- embreeのシーンを更新 --------------------------------------------
void MyObject::update_embree_scene(RTCDevice& device, RTCScene& scene) {
	int ntri = F.rows();      // シーン中の三角形の数
	int nvert = V.rows();     // シーン中の頂点の総数

	// ３角形ジオメトリを作成 
	RTCGeometry geom = this->geom;

	// 頂点座標をセット 
	int j = 0;
	float* vertices = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), nvert);
	for (int i = 0; i < nvert; i++) {
		vertices[j] = float(V(i, 0));
		vertices[j + 1] = float(V(i, 1));
		vertices[j + 2] = float(V(i, 2));
		j += 3;
	}

	// 面を構成する頂点番号をセット
	j = 0;
	unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), ntri);
	for (int i = 0; i < ntri; i++) {
		indices[j] = F(i, 0);
		indices[j + 1] = F(i, 1);
		indices[j + 2] = F(i, 2);
		// std::cout << "F?" << i<< " "<< sizeof(indices) <<" "<<F(i, 0) << F(i, 1) << F(i, 2) << " "<< indices[i]<<" "<< indices[i3 + 1] << std::endl;
		j += 3;
	}

	/* シーンへ登録 */
	rtcCommitGeometry(geom);
	this->geomID = rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);
	rtcCommitScene(scene);

	// geomを登録...
	this->geom = geom;
	// geomIDを設定
	//geomID = geomIDtemp;
	//geomIDtemp++;
}// --------------------------------------------------------------------


void MyObject::normalize_RayLengthMAP() {
	double maxx = 0;
	double minn = DBL_MAX;
	for (int i = 0; i < RayLengthMAP.size(); i++) {
		if (RayLengthMAP(i) == 0) continue;
		if (RayLengthMAP(i) > maxx) maxx = RayLengthMAP(i);
		if (RayLengthMAP(i) < minn) minn = RayLengthMAP(i);
	}
	for (int i = 0; i < RayLengthMAP.size(); i++) {
		if (RayLengthMAP(i) == 0) RayLengthMAP(i) = maxx;
	}
	std::cout << "normalize ratlength" << maxx << " " << minn << std::endl;
	double range = maxx - minn;
	RayLengthMAP.array() = (RayLengthMAP.array() - minn) / range;
}

// シーンにオブジェクトを追加し、引数オブジェクトをシーン内のオブジェクトの参照に
void MyScene::add_object_to_scene(MyObject &obj) {
	object.push_back(&obj);
	//return (this->object.back());
}


// -------- シーン情報を出力 --------------
void MyScene::scene_info() {
	std::cout << "\n- - - SCENE INFOMATION - - -" << std::endl;
	if (camera_exist) {
		std::cout << " << camera info >>" <<  std::endl;
		std::cout << " - - fovi   : " << camera.fovy << std::endl;
		std::cout << " - - refe   : "; camera.refe.print(); std::cout << std::endl;
		std::cout << " - - up     : "; camera.up.print();   std::cout << std::endl;
		std::cout << " - - view   : "; camera.view.print(); std::cout << std::endl;
	}
	else std::cout << "camera is not exist" << std::endl;
	std::cout << " << object info >>" << std::endl;
	std::cout << "number of object   : " << object.size() << std::endl;
	for (int i = 0; i < object.size(); i++) {
		(*object[i]).print_info();
	}
	std::cout << "number of texture  : " << texture.size() << std::endl;
	if (texture.size() != 0) {
		for (int i = 0; i < texture.size(); i++) {
			std::cout << "  - texture" << i << std::endl;
			std::cout << "  - - address   : " << texture[i] << std::endl;
			std::cout << "  - - size[" << (*texture[i]).getWidth() << " " << (*texture[i]).getHeight() << "]" << std::endl;
			std::cout << "  - - type      : " << (*texture[i]).getType() << std::endl;
		}
	}
	std::cout <<"- - - - - - - - - - - - - - " <<  std::endl;
}

void MyScene::add_to_embree_scene(RTCDevice &device, RTCScene &scene)
{
	this->rtcdevice_ptr = &device;
	this->rtcscene_ptr = &scene;
	for (int i = 0; i < this->object.size(); i++) {
		(*this->object[i]).add_to_embree_scene(device, scene);
	}
}


int MyScene::geomID_to_objectID(int id) {
	for (int i = 0; i < object.size(); i++) {
		if ((*object[i]).get_geomID() == id) {
			return (i);
		}
	}
	std::cout << "geomID : " << id << "を持つオブジェクトが存在しません\n id : " << std::endl;
	for (int i = 0; i < object.size(); i++) {
		std::cout << (*object[i]).get_geomID();
	}
	std::cout << std::endl;
	
}


// - - - テクスチャ読み込み - - - 
int MyScene::add_texture(std::string path) {
	
	char* temppath = new char[path.size() + 1];
	std::char_traits<char>::copy(temppath, path.c_str(), path.size() + 1);
	Image *tex;
	tex = new Image();
	if (tex->load(temppath) == false) {
		fprintf(stderr, "Cant read texture\n");
		return(-1);
	}
	texture.push_back(tex);

	// 仮出力
	char testname[] = "test.bmp";
	std::cout << "KK" << texture[0]->getWidth() << std::endl;
	std::cout << texture[0] << std::endl;
	(*texture[0]).save(testname);
	return 1;
}


// - - - カメラ登録 - - - - - -
void MyScene::make_camera() {
	Camera cam;
	this->camera = cam;
	this->camera_exist = true;
}
void MyScene::add_camera(Camera &cam) {
	this->camera = cam;
	this->camera_exist = true;
}




// - - - - - - 諸関数 - - - - - - - -

