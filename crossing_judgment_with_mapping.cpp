#include "crossing_judgment_with_mapping.h"
#include <math.h>

void inflection_crossing_judgement(
	Colorub& pixel_color,			//return
	RTCRayHit& prev_ray,
	RTCScene& scene, MyScene& myscene,
	int prev_obj_id,
	int iteration);
Colorub calc_color_with_texture(RTCRayHit& rayhit, MyScene& myscene, int obj_id);
Vector3 get_normal(RTCRayHit& rayhit, MyScene &myscene, int obj_id);
void reflection_crossing_judgement_test(
	Colorub& pixel_color,			//return
	RTCRayHit& prev_ray,
	RTCScene& scene, MyScene& myscene,
	int prev_obj_id,
	int count, int i, int j, std::vector<Eigen::RowVector3d>& points);
bool crossing_judgement_test(Image& image, RTCScene& scene, MyScene& myscene, int i, int j,
	std::vector<Eigen::RowVector3d>& points);



Vector3 get_normal(RTCRayHit& rayhit, MyScene &myscene, int obj_id) {
	Vector3 nom;
	if (!((*myscene.object[obj_id]).smooth_shading)) {
		nom = { rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z };
	}
	else {
		float P[4] = { 0,0,0,0 };
		RTCInterpolateArguments arg{ (*myscene.object[obj_id]).geom, rayhit.hit.primID, rayhit.hit.u, rayhit.hit.v, RTC_BUFFER_TYPE_VERTEX, 0,
			P, nullptr, nullptr, nullptr, nullptr, nullptr, 4 };
		rtcInterpolate(&arg);
		nom = { P[0], P[1], P[2] };
		//std::cout << P[0] << std::endl;
	}
	nom.normalize();
	return (nom);
}

 // 視線を計算
Vector3 calcViewingRay(Camera& cam, int ix, int iy, int width, int height)
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

// 色をミックス
void mix(Colorub &col1, Colorub &col2, double fac) {
	col1.r = (col1.r * (1 - fac)) + (col2.r * fac);
	col1.g = (col1.g * (1 - fac)) + (col2.g * fac);
	col1.b = (col1.b * (1 - fac)) + (col2.b * fac);
}



// 屈折時の交点計算
// 屈折後の2回目以降の交点計算はここで行う。
void inflection_crossing_judgement(
			Colorub& pixel_color,			//return
			RTCRayHit& prev_ray, 
			RTCScene& scene, MyScene& myscene, 
			int prev_obj_id,
			int iteration)
{
	double transparency = (*myscene.object[prev_obj_id]).mat.transparency;
	double IOR = (*myscene.object[prev_obj_id]).mat.IOR;
	// 新しいRAYを定義
	struct RTCRayHit rayhit;

	// レイの始点 
	rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
	rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
	rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);

	//std::cout << rayhit.ray.org_x << " " << rayhit.ray.org_x << " " << rayhit.ray.org_z << std::endl;

	// 法線の取得
	Vector3 nom;
	nom = get_normal(prev_ray, myscene, prev_obj_id);

	// ------- 屈折した視線の計算 -------------------------------------------------------
	Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);	
	view.normalize();

	// 裏面に衝突した場合
	if (nom.dot(view) > 0) {
		nom = nom * (-1.0);
		IOR = 1.0 / IOR;
	}

	view = (1 / fabs(view.dot(nom))) * view;	// 計算上Vの長さを指定する必要がある。
	
	double temp1 = IOR * IOR * view.lengthSquared();
	double temp2 = (nom + view).lengthSquared();
	double kf = 1.0 / sqrt(fabs(temp1 - temp2));

	// std::cout << IOR << "/" << kf << " ";
	
	Vector3 T; // 屈折した視線
	T = (nom + view)*kf - nom;
	// -----------------------------------------------------------------------------------
	
	rayhit.ray.dir_x = T.x;
	rayhit.ray.dir_y = T.y;
	rayhit.ray.dir_z = T.z;

	//std::cout << rayhit.ray.dir_x << " " << rayhit.ray.dir_x << " " << rayhit.ray.dir_z << std::endl;

	// 交差判定する範囲を指定 
	rayhit.ray.tnear = 0.011f;     // 範囲の始点 //屈折率１の時0.0だとエラー

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.ray.tfar = INFINITY;

	// 交差判定
	rtcIntersect1(scene, &context, &rayhit);
	//std::cout <<"h" <<rayhit.hit.geomID;

	if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

		int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);

		// 到達した際の屈折回数の中で最も小さいものを格納
		int * infmap = &((*myscene.object[obj_id]).InflectionMAP(rayhit.hit.primID) );
		if (*infmap == -1 || *infmap > iteration + 1) *infmap = iteration + 1;

		// 屈折の考慮
		if ((*myscene.object[obj_id]).mat.transparency != 0) {
			inflection_crossing_judgement(
				pixel_color,			//return
				rayhit,
				scene, myscene,
				obj_id, iteration + 1
			);
		}
	}
	
	// std::cout << "R" << std::endl;
}



// 反射時の交点計算
// 反射回数はmyscene.reflecting_limitで制限される
// 反射後の2回目以降の交点計算はここで行う。
// 現在屈折とは(正確には)両立できない。
void reflection_crossing_judgement(
	Colorub& pixel_color,			//return
	RTCRayHit& prev_ray,
	RTCScene& scene, MyScene& myscene,
	int prev_obj_id,
	int count)
{
	double reflectivity = (*myscene.object[prev_obj_id]).mat.reflectivity;

	// 新しいRAYを定義
	struct RTCRayHit rayhit;

	// レイの始点 
	rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
	rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
	rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);

	// 法線の取得
	Vector3 nom = get_normal(prev_ray, myscene, prev_obj_id);

	// ------- 反射した視線の計算 -------------------------------------------------------
	Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
	view.normalize();

	Vector3 R; // 反射した視線
	R = view - (2. * (view.dot(nom)) * nom);
	// -----------------------------------------------------------------------------------
	
	//if ((view + R).dot(nom) != 0) { std::cout << " / " << (view + R).dot(nom);}

	
	if (view.dot(nom) > 0) {
		std::cout << "ERROR : " << count << " " << prev_ray.hit.primID << " ";
		return;
	}
	

	rayhit.ray.dir_x = R.x;
	rayhit.ray.dir_y = R.y;
	rayhit.ray.dir_z = R.z;

	// 交差判定する範囲を指定 
	rayhit.ray.tnear = 0.001f;     // 範囲の始点 //屈折率１の時0.0だとエラー

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.ray.tfar = INFINITY;

	// 交差判定
	rtcIntersect1(scene, &context, &rayhit);
	//std::cout <<"h" <<rayhit.hit.geomID;

	if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

		int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
		MyObject* obj = myscene.object[obj_id];

		// 到達した際の反射回数の中で最も小さいものを格納
		int* infmap = &((*myscene.object[obj_id]).ReflectionMAP(rayhit.hit.primID));
		if (*infmap == -1 || *infmap > count) {
			
			*infmap = count;
		}
		
		
		// 再度反射の考慮
		if ((*obj).mat.reflectivity != 0 && myscene.reflecting_limit > count) {
			reflection_crossing_judgement(
				pixel_color,			//return
				rayhit,
				scene, myscene,
				obj_id,
				count + 1
			);
		}

	}
	
	//std::cout << "R" << std::endl;
}

// テクスチャがある場合の輝度計算
Colorub calc_color_with_texture(RTCRayHit &rayhit, MyScene &myscene, int obj_id) {
	// uv座標を計算
	Eigen::RowVectorXi ftc = (*myscene.object[obj_id]).FTC.row(rayhit.hit.primID);

	Eigen::RowVector2d a = (*myscene.object[obj_id]).TC.row(ftc(0));
	Eigen::RowVector2d b = (*myscene.object[obj_id]).TC.row(ftc(1));
	Eigen::RowVector2d c = (*myscene.object[obj_id]).TC.row(ftc(2));

	Eigen::RowVector2d uv = a + (rayhit.hit.u * (b - a)) + (rayhit.hit.v * (c - a));
	// UVを出力
	//Vector3 uv3(uv(0), uv(1), 0);
	//image.setPixel(i, j, convert_to_color(uv3));

	// 面UVを出力
	//Vector3 uvtemp(rayhit.hit.u, rayhit.hit.v, 0);
	//image.setPixel(i, j, convert_to_color(uvtemp));

	if (0 > uv(0) || uv(0) > 1) {
		std::cout << "u_ERROR " << uv(0) << std::endl;
	}
	if (0 > uv(1) || uv(1) > 1) {
		std::cout << "v_ERROR " << uv(1) << std::endl;
	}
	// テクスチャを出力
	Colorub texcol = (*myscene.texture[0]).getPixel((int)((double)(*myscene.texture[0]).getWidth() * uv(0)), (int)((double)(*myscene.texture[0]).getHeight() * uv(1)));
	//Colorub texcol = texture.getPixel(i, j);
	return(texcol);
}



// - - - レイとの交差判定 - - - 
bool crossing_judgement(Image& image, RTCScene& scene, MyScene &myscene) {
	
	// レイを生成する 
	struct RTCRayHit rayhit;

	// レイの始点 
	rayhit.ray.org_x = myscene.camera.view.x;  // x
	rayhit.ray.org_y = myscene.camera.view.y;  // y
	rayhit.ray.org_z = myscene.camera.view.z;  // z

	// 交差判定する範囲を指定 
	rayhit.ray.tnear = 0.0f;     // 範囲の始点
	
	// 交差判定 
	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	for (int j = 0; j < image.getHeight(); j++) {
		for (int i = 0; i < image.getWidth(); i++) {

			// そのピクセルの色
			Colorub pixel_color = COL_ERROR;

			// - - - -  RAYの初期化 - - - - - - -
			Vector3 ray = calcViewingRay(myscene.camera, i, j, image.getWidth(), image.getHeight());
			rayhit.ray.dir_x = ray.x;  // x
			rayhit.ray.dir_y = ray.y;  // y
			rayhit.ray.dir_z = ray.z;  // z

			rayhit.ray.flags = false;
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
			rayhit.ray.tfar = INFINITY;
			// - - - - - - - - - - - - - - - - - -

			// - - - - 交差判定 - - - - - - - - -
			rtcIntersect1(scene, &context, &rayhit);

			
			// - - - - レンダリングとマスク作成 - - - - - -
			if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
				
				int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);

				//屈折無しで見えている：重み１
				(*myscene.object[obj_id]).InflectionMAP(rayhit.hit.primID) = 0;
				(*myscene.object[obj_id]).ReflectionMAP(rayhit.hit.primID) = 0;
				//std::cout << ":" << rayhit.hit.primID <<" " <<  myscene.object[obj_id].VisibilityMAP(rayhit.hit.primID) << " ";
				// 輝度計算
				/*
				if (!(myscene.object[obj_id].has_texture)) {
					pixel_color = myscene.object[obj_id].color;
				}
				else {
					pixel_color = calc_color_with_texture(rayhit, myscene, obj_id);
				}*/
								
				// 屈折の考慮
				if ((*myscene.object[obj_id]).mat.transparency != 0) {
					inflection_crossing_judgement(
						pixel_color,			
						rayhit,
						scene, myscene,
						obj_id, 0.0
					);
				}	
				// 反射の考慮
				if ((*myscene.object[obj_id]).mat.reflectivity != 0) {
					reflection_crossing_judgement(
						pixel_color,
						rayhit,
						scene, myscene,
						obj_id, 1
					);
					
				}
			}
			else {
				pixel_color = COL_BACKGOUND;
			}
			image.setPixel(i, j, pixel_color);
			
		}
	}
	return(true);
}

// テスト用
void reflection_crossing_judgement_test(
	Colorub& pixel_color,			//return
	RTCRayHit& prev_ray,
	RTCScene& scene, MyScene& myscene,
	int prev_obj_id,
	int count, int i, int j,
	std::vector<Eigen::RowVector3d>& points)
{
	double reflectivity = (*myscene.object[prev_obj_id]).mat.reflectivity;

	// 新しいRAYを定義
	struct RTCRayHit rayhit;

	// レイの始点 
	rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
	rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
	rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);

	Eigen::RowVector3d point;
	point << rayhit.ray.org_x, rayhit.ray.org_y, rayhit.ray.org_z;
	points.push_back(point);

	// 法線の取得
	Vector3 nom = get_normal(prev_ray, myscene, prev_obj_id);

	// ------- 反射した視線の計算 -------------------------------------------------------
	Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
	view.normalize();

	Vector3 R; // 反射した視線
	R = view - (2. * (view.dot(nom)) * nom);
	// -----------------------------------------------------------------------------------

	if (view.dot(nom) > 0) {
		std::cout << "ERROR : " << count << " " << prev_ray.hit.primID << " ";
		return;
	}

	rayhit.ray.dir_x = R.x;
	rayhit.ray.dir_y = R.y;
	rayhit.ray.dir_z = R.z;

	// 交差判定する範囲を指定 
	rayhit.ray.tnear = 0.001f;     // 範囲の始点 //屈折率１の時0.0だとエラー

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.ray.tfar = INFINITY;

	// 交差判定
	rtcIntersect1(scene, &context, &rayhit);
	//std::cout <<"h" <<rayhit.hit.geomID;

	if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

		int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
		MyObject* obj = myscene.object[obj_id];

		std::cout << "crossing :" << rayhit.hit.primID << " / " << rayhit.hit.geomID << std::endl;

		std::cout << "[" << i << " " << j << "]";

		// 再度反射の考慮
		if ((*obj).mat.reflectivity != 0 && myscene.reflecting_limit > count) {
			reflection_crossing_judgement_test(
				pixel_color,			//return
				rayhit,
				scene, myscene,
				obj_id,
				count + 1, i,j, points
			);
		}
	}
}

//テスト用単レイ衝突判定
bool crossing_judgement_test(Image& image, RTCScene& scene, MyScene& myscene, int i, int j, 
							 std::vector<Eigen::RowVector3d>& points) 
{

	// レイを生成する 
	struct RTCRayHit rayhit;
	rayhit.ray.org_x = myscene.camera.view.x;  // x
	rayhit.ray.org_y = myscene.camera.view.y;  // y
	rayhit.ray.org_z = myscene.camera.view.z;  // z
	rayhit.ray.tnear = 0.0f;     // 範囲の始点
	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	Colorub pixel_color = COL_ERROR;

	// - - - -  RAYの初期化 - - - - - - -
	Vector3 ray = calcViewingRay(myscene.camera, i, j, image.getWidth(), image.getHeight());
	rayhit.ray.dir_x = ray.x;  // x
	rayhit.ray.dir_y = ray.y;  // y
	rayhit.ray.dir_z = ray.z;  // z
	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.ray.tfar = INFINITY;
	
	// - - - - 交差判定 - - - - - - - - -
	rtcIntersect1(scene, &context, &rayhit);


	// - - - - レンダリングとマスク作成 - - - - - -
	if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

		int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);

		std::cout << "crossing :" << rayhit.hit.primID  << " / " << rayhit.hit.geomID << std::endl;
		
		// 屈折の考慮
		if ((*myscene.object[obj_id]).mat.transparency != 0) {
			inflection_crossing_judgement(
				pixel_color,
				rayhit,
				scene, myscene,
				obj_id, 0.0
			);
		}
		// 反射の考慮
		if ((*myscene.object[obj_id]).mat.reflectivity != 0) {
			reflection_crossing_judgement_test(
				pixel_color,
				rayhit,
				scene, myscene,
				obj_id, 1, i,j, points
			);
		}
	}
	return(true);
}

