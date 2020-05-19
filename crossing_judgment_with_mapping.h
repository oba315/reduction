#pragma once

#include "image.h"
#include "vector3.h"
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#include <iostream>
#include <vector>
#include <igl/readOBJ.h>
#include "MyScene.h"
#include <Eigen/Core>

bool crossing_judgement(Image& image, RTCScene& scene, MyScene& myscene);

bool crossing_judgement_test(Image& image, RTCScene& scene, MyScene& myscene, int i, int j,	std::vector<Eigen::RowVector3d>& points);
