#include "embree_process.h"



// �R���X�g���N�^
embree_process::embree_process(Camera cam, Image image) {
	this->cam = cam;
	device = rtcNewDevice(NULL);  // �f�o�C�X���쐬
	scene = rtcNewScene(device);  // �V�[�����쐬�@
	this->image = image;          // �|�C���^�ł�������ق���������ˁA�A�A
}

// �f�X�g���N�^
embree_process :: ~embree_process() {
	rtcReleaseScene(scene);       // �V�[�����폜
	rtcReleaseDevice(device);	  // �f�o�C�X���폜
}

// ���b�V���̓o�^
void embree_process::mesh_registration(Eigen::MatrixXd V, Eigen::MatrixXi F) {

	
	int ntri = F.rows();      // �V�[�����̎O�p�`�̐�
	int nvert = V.rows();     // �V�[�����̒��_�̑���

	// �R�p�`�W�I���g�����쐬 
	RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

	// ���_���W���Z�b�g 
	int j = 0;
	float* vertices = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), nvert);
	for (int i = 0; i < nvert; i++) {
		vertices[j] = float(V(i, 0));
		vertices[j + 1] = float(V(i, 1));
		vertices[j + 2] = float(V(i, 2));
		j += 3;
	}

	// �ʂ��\�����钸�_�ԍ����Z�b�g
	j = 0;
	unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), ntri);
	for (int i = 0; i < ntri; i++) {
		indices[j] = F(i, 0);
		indices[j + 1] = F(i, 1);
		indices[j + 2] = F(i, 2);
		// std::cout << "F?" << i<< " "<< sizeof(indices) <<" "<<F(i, 0) << F(i, 1) << F(i, 2) << " "<< indices[i]<<" "<< indices[i3 + 1] << std::endl;
		j += 3;
	}

	/* �V�[���֓o�^ */
	rtcCommitGeometry(geom);
	rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);
	rtcCommitScene(scene);
}

// ���������F�摜�o��
void embree_process::intersection_decision() {

	// ---------- ���C�𐶐����� -------------------------------
	Vector3 ray;

	struct RTCRayHit rayhit;
	/* ���C�̎n�_ */
	rayhit.ray.org_x = cam.view.x;  // x
	rayhit.ray.org_y = cam.view.y;  // y
	rayhit.ray.org_z = cam.view.z;  // z
	
	/* �������肷��͈͂��w�� */
	rayhit.ray.tnear = 0.0f;     // �͈͂̎n�_
	rayhit.ray.tfar = INFINITY;  // �͈͂̏I�_�D���������ɂ͌����_�܂ł̋������i�[�����D

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	// ---------------------------------------------------------

	for (int j = 0; j < image.getHeight(); j++) {
		for (int i = 0; i < image.getWidth(); i++) {

			// ------- ���C�������� ------------
			ray = calcViewingRay(i, j, image.getWidth(), image.getHeight());
			rayhit.ray.dir_x     = ray.x;  // x
			rayhit.ray.dir_y     = ray.y;  // y
			rayhit.ray.dir_z     = ray.z;  // z

			rayhit.ray.flags     = false;	   // �����ƏՓ˂�����
			rayhit.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
			rayhit.ray.tnear     = 0.0f;       // �͈͂̎n�_
			rayhit.ray.tfar      = INFINITY;   // �I�_[�o��]
			
			rtcIntersect1(scene, &context, &rayhit);	// �Փ˔���
			
														// �摜�ɐF��o�^
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


// �ʔԍ��������̃}�b�v���쐬
// VisiblityMAP �� Eigen::VectorXi::Zero(F.rows()) ��n���Ă�������
void embree_process::intersection_decision(Eigen::VectorXi& VisiblityMAP) {
	
	// ---------- ���C�𐶐����� -------------------------------
	Vector3 ray;

	struct RTCRayHit rayhit;
	/* ���C�̎n�_ */
	rayhit.ray.org_x = cam.view.x;  // x
	rayhit.ray.org_y = cam.view.y;  // y
	rayhit.ray.org_z = cam.view.z;  // z

	/* �������肷��͈͂��w�� */
	rayhit.ray.tnear = 0.0f;     // �͈͂̎n�_
	rayhit.ray.tfar = INFINITY;  // �͈͂̏I�_�D���������ɂ͌����_�܂ł̋������i�[�����D


	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	// std::cout << "Visiblity map" << VisiblityMAP << std::endl;
	// ---------------------------------------------------------

	for (int j = 0; j < image.getHeight(); j++) {
		for (int i = 0; i < image.getWidth(); i++) {

			// ------- ���C�������� ------------
			ray = calcViewingRay(i, j, image.getWidth(), image.getHeight());
			rayhit.ray.dir_x = ray.x;  // x
			rayhit.ray.dir_y = ray.y;  // y
			rayhit.ray.dir_z = ray.z;  // z

			rayhit.ray.flags = false;	   // �����ƏՓ˂�����
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
			rayhit.ray.tnear = 0.0f;       // �͈͂̎n�_
			rayhit.ray.tfar = INFINITY;   // �I�_[�o��]
			rayhit.hit.geomID = -1;
			


			rtcIntersect1(scene, &context, &rayhit);	// �Փ˔���

			
			// -------- �����}�b�v�ɋL�^ ------------

			if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID && rayhit.hit.primID != -1) {
				VisiblityMAP(rayhit.hit.primID) = 1;
			}

			// ----------------------------------------


			// �摜�ɐF��o�^
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


// �摜��ۑ�
void embree_process::save_image() {

	char picturename[] = "out.bmp";
	std::cout << picturename << " ��ۑ����܂�" << std::endl;
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


// ���C�Ƃ̌�������(��̂݃e�X�g�p)
void embree_process::intersection_decision_test() {

	Vector3 ray = calcViewingRay(256, 128, image.getWidth(), image.getHeight());

	std::cout << "ray : " << ray.x << " " << ray.y << " " << ray.z << endl;


	/* ���C�𐶐����� */
	struct RTCRayHit rayhit;

	/* ���C�̎n�_ */
	rayhit.ray.org_x = cam.view.x;  // x
	rayhit.ray.org_y = cam.view.y;  // y
	rayhit.ray.org_z = cam.view.z;  // z
	/* ���C�̕��� */
	rayhit.ray.dir_x = ray.x;  // x
	rayhit.ray.dir_y = ray.y;  // y
	rayhit.ray.dir_z = ray.z;  // z
	/* �������肷��͈͂��w�� */
	rayhit.ray.tnear = 0.0f;     // �͈͂̎n�_
	rayhit.ray.tfar = INFINITY;  // �͈͂̏I�_�D���������ɂ͌����_�܂ł̋������i�[�����D

	rayhit.ray.flags = false;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

	/* �������� */
	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	rtcIntersect1(scene, &context, &rayhit);

	if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
	{
		/* �����_��������Ȃ������ꍇ */
		std::cout << "Reject." << std::endl;
	}
	else
	{
		/* �����_�����������ꍇ */
		std::cout << "Intersect" << std::endl;

		std::cout << "tfar : " << rayhit.ray.tfar << std::endl;

		//calc_col_point(rayhit, true);

	}

	/* .... */

	std::cout << "rtcray.geomID : " << rayhit.hit.geomID << std::endl;
	std::cout << "RTC_INVALID_GEOMETRY_ID : " << RTC_INVALID_GEOMETRY_ID << std::endl;
	std::cout << "type " << typeid(rayhit.hit.geomID).name() << std::endl;

}