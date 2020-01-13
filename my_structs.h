#pragma once
#include "vector3.h"


//- - - - カメラ構造体 - - - - - - - - -
// いずれEigenで書き直したほうが楽？
typedef struct {
	Vector3	view;
	Vector3	refe;
	Vector3	up;
	double	fovy;
} Camera;
// --------------------------------------

