#pragma once
#include "crossing_judgment.h"
#include "get_single_lay.h"
#include <math.h>
#include <cmath>
#include <Eigen/Core>
#include <tutorials/common/math/math.h>
#include <tutorials/common/math/vec.h>

// スム―ジュシェーディングの可否を考慮
Vector3 get_normal(RTCRayHit& rayhit, MyScene& myscene, RTCScene& scene, int obj_id) {

	struct vertex_t
	{
	    float position[4];  //位置
		float normal[3];    //法線
		float uv[2];        //テクスチャ座標
	} ;

	Vector3 nom;
	if (!((*myscene.object[obj_id]).smooth_shading)) {
		nom = { rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z };
		std::cout << "not smooth";
	}
	else {
		
		vertex_t v;
		size_t numFloats = sizeof(v) / sizeof(float);
		RTCInterpolateArguments arg{ (*myscene.object[obj_id]).geom, rayhit.hit.primID, rayhit.hit.u, rayhit.hit.v, RTC_BUFFER_TYPE_VERTEX, 0,
			 (float*)&v, nullptr, nullptr, nullptr, nullptr, nullptr, numFloats };
		rtcInterpolate(&arg);
		nom = {v.normal[0], v.normal[1], v.normal[2]};
		std::cout << "///" << rayhit.hit.primID;
		//nom.print();
		for (int i = 0; i < 15; i++) {
			//std::cout << std::fixed << std::setprecision(8) << P[i] << ",";
			//std::cout << v.normal[0]<<  v.normal[1] << v.normal[2] << std::endl;
		}
		std::cout << std::endl;
		

		/*
		float P[3] = { 0,0,0 };
		//Vec3fa diffuse = Vec3fa(1.0f, 0.0f, 0.0f);
		embree::Vec3fa Ng;
		embree::Vec3fa dPdu, dPdv;
		unsigned int geomID = rayhit.hit.geomID; {
			rtcInterpolate1(rtcGetGeometry(scene, geomID), rayhit.hit.primID,
				rayhit.hit.u, rayhit.hit.v, RTC_BUFFER_TYPE_VERTEX, 0, nullptr, &dPdu.x, &dPdv.x, 3);
			Ng = embree::cross(dPdu, dPdv);
			std::cout << "///" << rayhit.hit.primID << "/"<< dPdu << dPdv  << rayhit.hit.u << ":" << rayhit.hit.v << std::endl;
		}
		Ng = embree::normalize(Ng);
		//Ng = dPdu;
		nom.x = Ng.x; nom.y = Ng.y; nom.z = Ng.z;
		//std::cout << "nom::";
		//nom.print();
		*/
		/* calculate smooth shading normal */
		
		/*
		float temp;
		rtcCommitScene(scene);
		rtcCommitGeometry(rtcGetGeometry(scene,rayhit.hit.geomID));
		embree::Vec3fa Ng(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
		if (1) {
			embree::Vec3fa dPdu, dPdv;
			unsigned int geomID = rayhit.hit.geomID; {
				rtcInterpolate1(rtcGetGeometry(scene, geomID), rayhit.hit.primID, rayhit.hit.u, rayhit.hit.v, RTC_BUFFER_TYPE_VERTEX, 0, nullptr, &dPdu.x, &dPdv.x, 3);
			}
			rtcCommitScene(scene);

			RTCGeometry geom = rtcGetGeometry(scene, rayhit.hit.geomID);
			std::cout << geom;
			int pp = rtcGetGeometryFirstHalfEdge(geom, 0);
			std::cout <<"#" << pp;
			
				
				//return dPdu;
			Ng = cross(dPdu, dPdv);
			//std::cout << "/"<<  Ng << std::endl;
			std::cout << "\n";
			std::cout << "[" << rayhit.hit.geomID << "," << rayhit.hit.primID << "]" << rayhit.hit.u << " " << rayhit.hit.v << " " << dPdu << " " << dPdv << std::endl;
			temp = rayhit.hit.u;
		}
		Ng = normalize(Ng);
		
		nom.x = Ng.x; nom.y = Ng.y; nom.z = Ng.z;
		//nom.x = temp; nom.y = 0; nom.z = 0;
		*/
	}
	//nom.normalize();
	return (nom);
}


// 求めたい方向で初期化されている/交差判定がまだ行われていないレイを渡す．
void ray_calc_single(std::vector<Vector3>& ray_points,
	RTCRayHit& rayhit,
	RTCScene& scene,
	MyScene& myscene,
	Colorub& pixel_color,
	int inflection_count,
	int reflection_count) {

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	// - - - - 交差判定 - - - - - - - - -
	rtcIntersect1(scene, &context, &rayhit);

	// 軌跡
	Vector3 start(rayhit.ray.org_x,rayhit.ray.org_y, rayhit.ray.org_z);
	Vector3 dir(rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z);
	double farf = rayhit.ray.tfar;
	if (isinf(farf)) {
		std::cout << "far = inf" << farf << "\n "; farf = 500.;
	}
	Vector3 end = dir * farf + start;
	ray_points.push_back(start);
	ray_points.push_back(end);

	// - - - - レンダリングとマスク作成 - - - - - -
	if ((rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) && (inflection_count < myscene.inflecting_limit && reflection_count < myscene.reflecting_limit)) {

		int       obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
		MyObject* obj = myscene.object[obj_id];
		Vector3 nomx(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
		Vector3 nom = get_normal(rayhit, myscene, scene, obj_id);
		//nom.normalize();
		std::cout << "衝突";
		nomx.print();
		nom.print();
		end.print();
		std::cout << std::endl;

		// 軌跡
		double nomlen = 2.5;
		ray_points.push_back(end);
		ray_points.push_back(nom*nomlen + end);
		ray_points.push_back(end);
		ray_points.push_back(nomx*nomlen + end);



		
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
				struct RTCRayHit inf_ray = ren::calc_inflection_ray(rayhit, scene, myscene, obj_id, IOR);
				ray_calc_single(ray_points, inf_ray, scene, myscene, pixel_color,
					inflection_count + 1, reflection_count);
			}
			else { col_inf = COL_BLACK; }

			// 反射レイを計算
			Colorub col_ref;
			if (flesnel != 0) {
				struct RTCRayHit ref_ray = ren::calc_reflection_ray(rayhit, scene, myscene, obj_id, IOR);
				ray_calc_single(ray_points, ref_ray, scene, myscene, pixel_color,
					inflection_count, reflection_count + 1);
			}
			else { col_ref = COL_BLACK; }
		}
	}
	
}

//　表示用のレイの追跡をする．Vector3のvectorで始点終点始点で返す．
std::vector<Vector3> get_single_lay(int i, int j, Image& image, RTCScene& scene, MyScene myscene) {
	// レイを生成する 
	struct RTCRayHit rayhit;

	// レイの始点 
	rayhit.ray.org_x = myscene.camera.view.x;  // x
	rayhit.ray.org_y = myscene.camera.view.y;  // y
	rayhit.ray.org_z = myscene.camera.view.z;  // z
	// - - - -  RAYの初期化 - - - - - - -
	Vector3 ray = ren::calcViewingRay(myscene.camera, i, j, image.getWidth(), image.getHeight());
	rayhit.ray.dir_x = ray.x;  // x
	rayhit.ray.dir_y = ray.y;  // y
	rayhit.ray.dir_z = ray.z;  // z
	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.ray.tfar = INFINITY;
	// - - - - - - - - - - - - - - - - - -
	int inflection_count = 0;
	int reflection_count = 0;

	std::vector<Vector3> ray_points;
	Colorub pixel_color = COL_BACKGOUND;
	ray_calc_single(ray_points, rayhit, scene, myscene, pixel_color, inflection_count, reflection_count);

	std::cout << ":single_ray\n";
	for (Vector3 v : ray_points) {
		std::cout << v.x << " " << v.y << " " << v.z << std::endl;
	}
	
	return ray_points;
}


std::vector<Vector3> show_normal(int inc, Image& image, RTCScene& scene, MyScene myscene) {

	std::vector<Vector3> ray_points;

	// レイを生成する 
	struct RTCRayHit rayhit;
	// レイの始点 
	rayhit.ray.org_x = myscene.camera.view.x;  // x
	rayhit.ray.org_y = myscene.camera.view.y;  // y
	rayhit.ray.org_z = myscene.camera.view.z;  // z
	for (int i=0; i < image.getWidth(); i += inc) {
		for (int j=0; j < image.getHeight(); j += inc) {
			// - - - -  RAYの初期化 - - - - - - -
			Vector3 ray = ren::calcViewingRay(myscene.camera, i, j, image.getWidth(), image.getHeight());
			rayhit.ray.dir_x = ray.x;  // x
			rayhit.ray.dir_y = ray.y;  // y
			rayhit.ray.dir_z = ray.z;  // z
			rayhit.ray.flags = false;
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
			rayhit.ray.tfar = INFINITY;
			// - - - - - - - - - - - - - - - - - -
			int inflection_count = 0;
			int reflection_count = 0;

			// - - - - 交差判定 - - - - - - - - -
			struct RTCIntersectContext context;
			rtcInitIntersectContext(&context);
			rtcIntersect1(scene, &context, &rayhit);

			// - - - - レンダリングとマスク作成 - - - - - -
			if ((rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) && (inflection_count < myscene.inflecting_limit && reflection_count < myscene.reflecting_limit)) {

				int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
				// 軌跡
				Vector3 org(rayhit.ray.org_x, rayhit.ray.org_y, rayhit.ray.org_z);
				Vector3 dir(rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z);
				double farf = rayhit.ray.tfar;
				Vector3 hitp = org + dir * farf;
				//Vector3 nomx(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
				Vector3 nom = get_normal(rayhit, myscene, scene, obj_id);
				//nom.normalize();

				// 軌跡
				double nomlen = 0.5;
				ray_points.push_back(hitp);
				ray_points.push_back(nom * nomlen + hitp);

			}
		}
	}

	return ray_points;
}

