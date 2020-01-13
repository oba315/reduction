#include "embree_process.h"



// コンストラクタ
embree_process::embree_process(Camera cam, Image image) {
	this->cam = cam;
	device = rtcNewDevice(NULL);  // デバイスを作成
	scene = rtcNewScene(device);  // シーンを作成　
	this->image = image;          // ポインタでもらったほうがいいよね、、、
}

// デストラクタ
embree_process :: ~embree_process() {
	rtcReleaseScene(scene);       // シーンを削除
	rtcReleaseDevice(device);	  // デバイスを削除
}

// メッシュの登録
void embree_process::mesh_registration(Eigen::MatrixXd V, Eigen::MatrixXi F) {

	
	int ntri = F.rows();      // シーン中の三角形の数
	int nvert = V.rows();     // シーン中の頂点の総数

	// ３角形ジオメトリを作成 
	RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

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
	rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);
	rtcCommitScene(scene);
}

// 引数無し：画像出力
void embree_process::intersection_decision() {

	// ---------- レイを生成する -------------------------------
	Vector3 ray;

	struct RTCRayHit rayhit;
	/* レイの始点 */
	rayhit.ray.org_x = cam.view.x;  // x
	rayhit.ray.org_y = cam.view.y;  // y
	rayhit.ray.org_z = cam.view.z;  // z
	
	/* 交差判定する範囲を指定 */
	rayhit.ray.tnear = 0.0f;     // 範囲の始点
	rayhit.ray.tfar = INFINITY;  // 範囲の終点．交差判定後には交差点までの距離が格納される．

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	// ---------------------------------------------------------

	for (int j = 0; j < image.getHeight(); j++) {
		for (int i = 0; i < image.getWidth(); i++) {

			// ------- レイを初期化 ------------
			ray = calcViewingRay(i, j, image.getWidth(), image.getHeight());
			rayhit.ray.dir_x     = ray.x;  // x
			rayhit.ray.dir_y     = ray.y;  // y
			rayhit.ray.dir_z     = ray.z;  // z

			rayhit.ray.flags     = false;	   // 何かと衝突したか
			rayhit.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
			rayhit.ray.tnear     = 0.0f;       // 範囲の始点
			rayhit.ray.tfar      = INFINITY;   // 終点[出力]
			
			rtcIntersect1(scene, &context, &rayhit);	// 衝突判定
			
														// 画像に色を登録
			if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
				
				image.setPixel(i, j, COL_RED);
				//std::cout << "Intersect" << std::endl;
				//std::cout << rayhit.hit.primID << std::endl;
				//Colorub col = int_to_color(rayhit.hit.primID, 32 * 32);
				//image.setPixel(i, j, col);
			}
			else {
				image.setPixel(i, j, COL_BLACK);
			}
		}
	}
}


// 面番号→可視性のマップを作成
// VisiblityMAP は Eigen::VectorXi::Zero(F.rows()) を渡してください
void embree_process::intersection_decision(Eigen::VectorXi& VisiblityMAP) {
	
	// ---------- レイを生成する -------------------------------
	Vector3 ray;

	struct RTCRayHit rayhit;
	/* レイの始点 */
	rayhit.ray.org_x = cam.view.x;  // x
	rayhit.ray.org_y = cam.view.y;  // y
	rayhit.ray.org_z = cam.view.z;  // z

	/* 交差判定する範囲を指定 */
	rayhit.ray.tnear = 0.0f;     // 範囲の始点
	rayhit.ray.tfar = INFINITY;  // 範囲の終点．交差判定後には交差点までの距離が格納される．


	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	// std::cout << "Visiblity map" << VisiblityMAP << std::endl;
	// ---------------------------------------------------------

	for (int j = 0; j < image.getHeight(); j++) {
		for (int i = 0; i < image.getWidth(); i++) {

			// ------- レイを初期化 ------------
			ray = calcViewingRay(i, j, image.getWidth(), image.getHeight());
			rayhit.ray.dir_x = ray.x;  // x
			rayhit.ray.dir_y = ray.y;  // y
			rayhit.ray.dir_z = ray.z;  // z

			rayhit.ray.flags = false;	   // 何かと衝突したか
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
			rayhit.ray.tnear = 0.0f;       // 範囲の始点
			rayhit.ray.tfar = INFINITY;   // 終点[出力]
			rayhit.hit.geomID = -1;
			


			rtcIntersect1(scene, &context, &rayhit);	// 衝突判定

			
			// -------- 可視性マップに記録 ------------

			if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID && rayhit.hit.primID != -1) {
				VisiblityMAP(rayhit.hit.primID) = 1;
			}

			// ----------------------------------------


			// 画像に色を登録
			if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

				image.setPixel(i, j, COL_RED);

				//std::cout << "Intersect" << std::endl;
				//std::cout << rayhit.hit.primID << std::endl;

				//Colorub col = int_to_color(rayhit.hit.primID, 32 * 32);
				//image.setPixel(i, j, col);
			}
			else {
				image.setPixel(i, j, COL_BLACK);
			}

		}
		//std::cout << std::endl;
	}
	//std::cout << "Visiblity map" << VisiblityMAP << std::endl;

}


// 画像を保存
void embree_process::save_image() {

	char picturename[] = "out.bmp";
	std::cout << picturename << " を保存します" << std::endl;
	image.save(picturename);
}


Vector3 embree_process::calcViewingRay(int ix, int iy, int width, int height)
{

	Vector3 view_vec = cam.refe - cam.view;
	Vector3 side_vec = view_vec.cross(cam.up);
	Vector3 up_vec = side_vec.cross(view_vec);
	side_vec.normalize();
	up_vec.normalize();
	double vr = view_vec.length();
	double dp = (2.0 * vr * tan(cam.fovy / 2.0)) / (double)height;
	double xs = ((ix + 0.5f) - width / 2.0) * dp;
	double ys = ((iy + 0.5f) - height / 2.0) * dp;
	Vector3 ray = xs * side_vec + ys * up_vec + view_vec;
	ray.normalize();
	return(ray);
}


// レイとの交差判定(一つのみテスト用)
void embree_process::intersection_decision_test() {

	Vector3 ray = calcViewingRay(256, 128, image.getWidth(), image.getHeight());

	std::cout << "ray : " << ray.x << " " << ray.y << " " << ray.z << endl;


	/* レイを生成する */
	struct RTCRayHit rayhit;

	/* レイの始点 */
	rayhit.ray.org_x = cam.view.x;  // x
	rayhit.ray.org_y = cam.view.y;  // y
	rayhit.ray.org_z = cam.view.z;  // z
	/* レイの方向 */
	rayhit.ray.dir_x = ray.x;  // x
	rayhit.ray.dir_y = ray.y;  // y
	rayhit.ray.dir_z = ray.z;  // z
	/* 交差判定する範囲を指定 */
	rayhit.ray.tnear = 0.0f;     // 範囲の始点
	rayhit.ray.tfar = INFINITY;  // 範囲の終点．交差判定後には交差点までの距離が格納される．

	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

	/* 交差判定 */
	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	rtcIntersect1(scene, &context, &rayhit);

	if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
	{
		/* 交差点が見つからなかった場合 */
		std::cout << "Reject." << std::endl;
	}
	else
	{
		/* 交差点が見つかった場合 */
		std::cout << "Intersect" << std::endl;

		std::cout << "tfar : " << rayhit.ray.tfar << std::endl;

		//calc_col_point(rayhit, true);

	}

	/* .... */

	std::cout << "rtcray.geomID : " << rayhit.hit.geomID << std::endl;
	std::cout << "RTC_INVALID_GEOMETRY_ID : " << RTC_INVALID_GEOMETRY_ID << std::endl;
	std::cout << "type " << typeid(rayhit.hit.geomID).name() << std::endl;

}