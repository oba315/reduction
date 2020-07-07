#pragma once

#include "image.h"
#include "vector3.h"
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#include <iostream>
#include <vector>
#include <igl/readOBJ.h>
#include "MyScene.h"

namespace ren
{

	bool crossing_judgement(Image& image, RTCScene& scene, MyScene myscene);
	Colorub mix2(Colorub& col1, Colorub& col2, double fac);
	RTCRayHit calc_inflection_ray(RTCRayHit& prev_ray, RTCScene& scene, MyScene& myscene, int prev_obj_id, float IOR);
	RTCRayHit calc_reflection_ray(RTCRayHit& prev_ray, RTCScene& scene, MyScene& myscene, int prev_obj_id, float IOR);
	Vector3 calcViewingRay(Camera& cam, int ix, int iy, int width, int height);
	Vector3 get_normal(RTCRayHit& rayhit, MyScene& myscene, int obj_id);
}

