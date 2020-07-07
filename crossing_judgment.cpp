#include "crossing_judgment.h"
#include <math.h>
#include <cmath>
#include <Eigen/Core>


namespace ren
{

	void inflection_crossing_judgement(
		Colorub& pixel_color,			//return
		RTCRayHit& prev_ray,
		RTCScene& scene, MyScene& myscene,
		int prev_obj_id);
	Colorub calc_color_with_texture(RTCRayHit& rayhit, MyScene& myscene, int obj_id);
	Vector3 get_normal(RTCRayHit& rayhit, MyScene& myscene, int obj_id);


	// スム―ジュシェーディングの可否を考慮
	Vector3 get_normal(RTCRayHit& rayhit, MyScene& myscene, int obj_id) {
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

	// スム―ジュシェーディングの可否を考慮
	Vector3 get_normal2(RTCRayHit& rayhit, MyScene& myscene, RTCScene& scene, int obj_id) {
		Vector3 nom;
		if (!((*myscene.object[obj_id]).smooth_shading)) {
			nom = { rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z };
		}
		else {	//スムーズシェーディング未実装
			float P[3] = { 0,0,0 };
			float dPdu[3] = { 0,0,0 };
			float dPdv[3] = { 0,0,0 };
			rtcInterpolate1(rtcGetGeometry(scene, rayhit.hit.geomID), rayhit.hit.primID,
				rayhit.hit.u, rayhit.hit.v, RTC_BUFFER_TYPE_VERTEX, 0, P, &dPdu[0], &dPdv[0], 3);
			Vector3 d1(dPdu[0], dPdu[1], dPdu[2]);
			Vector3 d2(dPdv[0], dPdv[1], dPdv[2]);
			Vector3 PP(P[0], P[1], P[2]);

			nom = d1.cross(d2);
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

	// 色をミックス : col1*(1-fac) + col2*fac
	void mix(Colorub& col1, Colorub& col2, double fac) {
		col1.r = (int)(((float)col1.r * (1. - fac)) + ((float)col2.r * fac));
		col1.g = (int)(((float)col1.g * (1. - fac)) + ((float)col2.g * fac));
		col1.b = (int)(((float)col1.b * (1. - fac)) + ((float)col2.b * fac));
	}
	Colorub mix2(Colorub& col1, Colorub& col2, double fac) {
		Colorub ret;
		ret.r = (int)(((float)col1.r * (1. - fac)) + ((float)col2.r * fac));
		ret.g = (int)(((float)col1.g * (1. - fac)) + ((float)col2.g * fac));
		ret.b = (int)(((float)col1.b * (1. - fac)) + ((float)col2.b * fac));
		return ret;
	}



	// ベクトルと環境マップの変換
	Colorub vec_to_envmap(RTCRayHit& rayhit, Image& envmap) {

		float x = rayhit.ray.dir_x;
		float y = rayhit.ray.dir_y;
		float z = rayhit.ray.dir_z;
		//std::cout << x << "/" << y << "/" << rayhit.ray.dir_z << "\n";

		
		float u = (1. / (2. * M_PI)) * atan(z / x) + 0.5;
		if (x < 0) {
			if (z <= 0) u -= 0.5;
			else u += 0.5;
		}
		float v = (1 / M_PI) * asin(y) + 0.5;

		return (envmap.getPixel(u, v));
	}

	// - - 衝突しなかったレイの処理
	void ray_to_backgrownd(RTCRayHit& rayhit, MyScene& myscene, Colorub& col) {
		if (myscene.use_environment_map) {
			col = vec_to_envmap(rayhit, *myscene.envmap);
		}
		else col = COL_BACKGOUND;
	}


	// 屈折時の交点計算
	// 屈折後の2回目以降の交点計算はここで行う。
	void inflection_crossing_judgement(
		Colorub& pixel_color,			//return
		RTCRayHit& prev_ray,
		RTCScene& scene, MyScene& myscene,
		int prev_obj_id)
	{
		std::cout << "regacy!";
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
		T = (nom + view) * kf - nom;
		// -----------------------------------------------------------------------------------

		T.normalize();

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

			Colorub col;
			int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
			MyObject* obj = myscene.object[obj_id];


			if (!((*myscene.object[obj_id]).has_texture)) {
				col = (*myscene.object[obj_id]).color;
			}
			else {
				col = calc_color_with_texture(rayhit, myscene, obj_id);
			}

			//　色をミックス
			mix(pixel_color, col, transparency);

			// 屈折の考慮
			if ((*obj).mat.transparency != 0) {
				inflection_crossing_judgement(
					pixel_color,			//return
					rayhit,
					scene, myscene,
					obj_id
				);
			}

		}
		else {
			ray_to_backgrownd(rayhit, myscene, pixel_color);
		}
		//std::cout << "R" << std::endl;
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
		std::cout << "regacy!";

		double reflectivity = (*myscene.object[prev_obj_id]).mat.reflectivity;

		// 新しいRAYを定義
		struct RTCRayHit rayhit;

		// レイの始点 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);


		// 法線の取得
		Vector3 nom = get_normal(prev_ray, myscene, prev_obj_id);
		// スムーズシェーディングを使うと、ありえない衝突が起こってしまう。
		nom.normalize();


		// ------- 反射した視線の計算 -------------------------------------------------------
		Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
		view.normalize();

		Vector3 R; // 反射した視線
		R = view - (2. * (view.dot(nom)) * nom);
		// -----------------------------------------------------------------------------------

		// 稜線付近ではうまく反射を計算できない点がある。
		if (view.dot(nom) > 0) {
			std::cout << "ERROR : " << count << " " << prev_ray.hit.primID << " " << prev_ray.hit.geomID << " ";
			pixel_color = COL_ERROR;
			return;
		}

		R.normalize();

		rayhit.ray.dir_x = R.x;
		rayhit.ray.dir_y = R.y;
		rayhit.ray.dir_z = R.z;

		// 交差判定する範囲を指定 
		rayhit.ray.tnear = 0.09f;     // 範囲の始点 //屈折率１の時0.0だとエラー

		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		// 交差判定
		rtcIntersect1(scene, &context, &rayhit);
		//std::cout <<"h" <<rayhit.hit.geomID;

		if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

			Colorub col;
			int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
			MyObject* obj = myscene.object[obj_id];


			if (!((*obj).has_texture)) {
				col = (*myscene.object[obj_id]).color;
			}
			else {
				col = calc_color_with_texture(rayhit, myscene, obj_id);
			}

			//　色をミックス
			mix(pixel_color, col, reflectivity);

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
		else {
			Colorub bg = COL_BACKGOUND;
			mix(pixel_color, bg, reflectivity);
		}
		//std::cout << "R" << std::endl;
	}

	// テクスチャがある場合の輝度計算
	Colorub calc_color_with_texture(RTCRayHit& rayhit, MyScene& myscene, int obj_id) {
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



	// - - - ランバートシェーダの計算 - - -
	Colorub rambert(RTCRayHit& rayhit, MyScene &myscene) {
		int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
		MyObject* obj = myscene.object[obj_id];

		/* 法線スムージング未処理 */
		Vector3 n(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
		n.normalize();

		double diffuse = obj->mat.diffuse *  n.dot(myscene.sun); //ディフューズ係数
		
		diffuse = max(0., -1*diffuse);

		double ambient = obj->mat.ambient;				 //環境光係数

		Colorub cs;	//物体の表面色

		if (!((*myscene.object[obj_id]).has_texture)) {
			cs = myscene.object[obj_id]->color;
		}
		else {
			cs = calc_color_with_texture(rayhit, myscene, obj_id);
		}

		double coe;
		if ((diffuse + ambient) > 1) coe = 1;
		else coe = (diffuse + ambient);
		Colorub C = cs * coe;

		//cout << diffuse <<"+" << ambient <<"+" << coe << "/" << int(C.r) << "//";

		return C;
	}

	// 屈折レイの計算
	RTCRayHit calc_inflection_ray(RTCRayHit& prev_ray, RTCScene& scene, MyScene& myscene,int prev_obj_id, float IOR){
		
		struct RTCRayHit rayhit;
		
		// レイの始点 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);

		// 法線の取得
		Vector3 nom;
		nom = get_normal2(prev_ray, myscene,scene,  prev_obj_id);

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
		T = (nom + view) * kf - nom;
		// -----------------------------------------------------------------------------------

		T.normalize();
		rayhit.ray.dir_x = T.x;
		rayhit.ray.dir_y = T.y;
		rayhit.ray.dir_z = T.z;

		//std::cout << rayhit.ray.dir_x << " " << rayhit.ray.dir_x << " " << rayhit.ray.dir_z << std::endl;

		// 交差判定する範囲を指定 
		rayhit.ray.tnear = 0.011f;     // 範囲の始点 //屈折率１の時0.0だとエラー
		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		return rayhit;
	}
	// 反射レイの計算
	RTCRayHit calc_reflection_ray(RTCRayHit& prev_ray, RTCScene& scene, MyScene& myscene, int prev_obj_id, float IOR) {

		double reflectivity = (*myscene.object[prev_obj_id]).mat.reflectivity;

		// 新しいRAYを定義
		struct RTCRayHit rayhit;

		// レイの始点 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);


		// 法線の取得
		Vector3 nom = get_normal2(prev_ray, myscene, scene, prev_obj_id);
		// スムーズシェーディングを使うと、ありえない衝突が起こってしまう。
		// nom = { prev_ray.hit.Ng_x, prev_ray.hit.Ng_y, prev_ray.hit.Ng_z }; 
		// nom.normalize();

		//std::cout << nom.x << " " << nom.y << " " << nom.z << std::endl;

		// ------- 反射した視線の計算 -------------------------------------------------------
		Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
		view.normalize();

		// 裏面に衝突した場合
		if (nom.dot(view) > 0) {
			nom = nom * (-1.0);
			IOR = 1.0 / IOR;
			//std::cout << "a";
		}

		
		Vector3 R; // 反射した視線
		R = view - (2. * (view.dot(nom)) * nom);
		//R = 2. * nom + view;
		// -----------------------------------------------------------------------------------

		// 稜線付近ではうまく反射を計算できない点がある。
		if (view.dot(nom) > 0) {
			//std::cout << "ERROR : " << """count <<""" " " << prev_ray.hit.primID << " " << prev_ray.hit.geomID << " " << nom.x <<"/" << nom.y<<"/"<<nom.z <<" ";
			//pixel_color = COL_ERROR;
			return (rayhit);
		}

		R.normalize();

		rayhit.ray.dir_x = R.x;
		rayhit.ray.dir_y = R.y;
		rayhit.ray.dir_z = R.z;

		// 交差判定する範囲を指定 
		rayhit.ray.tnear = 0.09f;     // 範囲の始点 //屈折率１の時0.0だとエラー


		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		return rayhit;
	}
		
		

	// 求めたい方向で初期化されている/交差判定がまだ行われていないレイを渡す．
	Colorub ray_calc(   RTCRayHit& rayhit, 
						RTCScene& scene, 
						MyScene &myscene, 
						Colorub &pixel_color, 
						int inflection_count, 
						int reflection_count   ) {
		
		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		// - - - - 交差判定 - - - - - - - - -
		rtcIntersect1(scene, &context, &rayhit);

		// - - - - レンダリングとマスク作成 - - - - - -
		if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

			if (inflection_count < myscene.inflecting_limit && reflection_count < myscene.reflecting_limit) {
				int       obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
				MyObject* obj = myscene.object[obj_id];
				Vector3 dir(rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z);
				Vector3 nom(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
				nom.normalize();

				// ベースカラーの計算
				Colorub basecol = rambert(rayhit, myscene);
				pixel_color = basecol;


				// 透過率が1未満の場合，反射，屈折の考慮 
				if (obj->mat.transparency != 0) {


					// ----------------  reduction用可視性重みづけ --------------------------------- 
					int fid = rayhit.hit.primID;
					Eigen::VectorXi ff = obj->InfRefMAP.row(fid);
					if ((ff(0) == -1) || inflection_count < ff(0)) {
						obj->InfRefMAP(fid, 0) = inflection_count;
					}
					if ((ff(1) == -1) || reflection_count < ff(1)) {
						obj->InfRefMAP(fid, 1) = reflection_count;
					}// -----------------------------------------------------------------------------

					// ---------- フレネルの計算：何割が反射するか(残りは屈折) ------------------------
					float flesnel;
					float IOR = (*myscene.object[obj_id]).mat.IOR;
					// 裏面に衝突した場合
					if (nom.dot(dir) > 0) {
						nom = nom * (-1.0);
					}
					float f0 = pow(((1 - IOR) / (1 + IOR)), 2);
					float cosine = -1 * (dir.dot(nom));						//内積からcosの計算
					flesnel = f0 + (1 - f0) * pow((1 - cosine), 5);

					//flesnel = 1;
					if (flesnel < 0 || flesnel > 1) std::cout << "flesnel error : " << flesnel;
					// --------------------------------------------------------------------------------


					/*透過マテリアルは無条件でとりあえず屈折も反射も計算しているからムダかも*/

					// 屈折レイを計算
					Colorub col_inf;
					if (flesnel != 1) {
						struct RTCRayHit inf_ray = calc_inflection_ray(rayhit, scene, myscene, obj_id, IOR);
						col_inf = ray_calc(inf_ray, scene, myscene, pixel_color,
							inflection_count + 1, reflection_count);
					}
					else { col_inf = COL_BLACK; }

					// 反射レイを計算
					Colorub col_ref;
					if (flesnel != 0) {
						struct RTCRayHit ref_ray = calc_reflection_ray(rayhit, scene, myscene, obj_id, IOR);
						col_ref = ray_calc(ref_ray, scene, myscene, pixel_color,
							inflection_count, reflection_count + 1);
					}
					else { col_ref = COL_BLACK; }

					// 屈折，反射，ベースカラーをフレネルと反射率に基づいて合成
					Colorub rr = mix2(col_inf, col_ref, flesnel);
					pixel_color = mix2(basecol, rr, obj->mat.transparency);
				}
			}
			else { // 衝突回数制限
				ray_to_backgrownd(rayhit, myscene, pixel_color);
				pixel_color = COL_BLACK;
			}
		}
		else { // 非衝突
			ray_to_backgrownd(rayhit, myscene, pixel_color);
		}

		return( pixel_color);
	}


	// - - - レイとの交差判定 - - - 
	bool crossing_judgement(Image& image, RTCScene& scene, MyScene myscene) {

		// 環境マップ
		Image temp;
		Image* envmap = &temp;
		if (myscene.use_environment_map) {
			for (Image* ptr : myscene.texture) {
				if ((*ptr).getType() == "ENV") {
					envmap = ptr;
				}
			}
		}

		// レイを生成する 
		struct RTCRayHit rayhit;

		// レイの始点 
		rayhit.ray.org_x = myscene.camera.view.x;  // x
		rayhit.ray.org_y = myscene.camera.view.y;  // y
		rayhit.ray.org_z = myscene.camera.view.z;  // z

		// 交差判定する範囲を指定 
		rayhit.ray.tnear = 0.09f;     // 範囲の始点

		// 交差判定 
		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		for (int j = 0; j < image.getHeight(); j++) {
			for (int i = 0; i < image.getWidth(); i++) {

				//std::cout << "[" << i << "," << j << "]";

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

				int inflection_count = 0;
				int reflection_count = 0;

				pixel_color = ray_calc(rayhit, scene, myscene, pixel_color, inflection_count, reflection_count);
				if (0 <= pixel_color.r && pixel_color.r <= 255 && 0 <= pixel_color.r && pixel_color.g <= 255 && 0 <= pixel_color.b && pixel_color.b <= 255){
					image.setPixel(i, j, pixel_color);
				}
				else std::cout << "ERROR: color < 0 or color > 255" << std::endl;

			}
		}
		return(true);
	}

}