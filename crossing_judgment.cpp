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


	// �X���\�W���V�F�[�f�B���O�̉ۂ��l��
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

	// �X���\�W���V�F�[�f�B���O�̉ۂ��l��
	Vector3 get_normal2(RTCRayHit& rayhit, MyScene& myscene, RTCScene& scene, int obj_id) {
		Vector3 nom;
		if (!((*myscene.object[obj_id]).smooth_shading)) {
			nom = { rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z };
		}
		else {	//�X���[�Y�V�F�[�f�B���O������
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

	// �������v�Z
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

	// �F���~�b�N�X : col1*(1-fac) + col2*fac
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



	// �x�N�g���Ɗ��}�b�v�̕ϊ�
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

	// - - �Փ˂��Ȃ��������C�̏���
	void ray_to_backgrownd(RTCRayHit& rayhit, MyScene& myscene, Colorub& col) {
		if (myscene.use_environment_map) {
			col = vec_to_envmap(rayhit, *myscene.envmap);
		}
		else col = COL_BACKGOUND;
	}


	// ���܎��̌�_�v�Z
	// ���܌��2��ڈȍ~�̌�_�v�Z�͂����ōs���B
	void inflection_crossing_judgement(
		Colorub& pixel_color,			//return
		RTCRayHit& prev_ray,
		RTCScene& scene, MyScene& myscene,
		int prev_obj_id)
	{
		std::cout << "regacy!";
		double transparency = (*myscene.object[prev_obj_id]).mat.transparency;
		double IOR = (*myscene.object[prev_obj_id]).mat.IOR;
		// �V����RAY���`
		struct RTCRayHit rayhit;

		// ���C�̎n�_ 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);

		//std::cout << rayhit.ray.org_x << " " << rayhit.ray.org_x << " " << rayhit.ray.org_z << std::endl;

		// �@���̎擾
		Vector3 nom;
		nom = get_normal(prev_ray, myscene, prev_obj_id);


		// ------- ���܂��������̌v�Z -------------------------------------------------------
		Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
		view.normalize();

		// ���ʂɏՓ˂����ꍇ
		if (nom.dot(view) > 0) {
			nom = nom * (-1.0);
			IOR = 1.0 / IOR;
		}

		view = (1 / fabs(view.dot(nom))) * view;	// �v�Z��V�̒������w�肷��K�v������B

		double temp1 = IOR * IOR * view.lengthSquared();
		double temp2 = (nom + view).lengthSquared();
		double kf = 1.0 / sqrt(fabs(temp1 - temp2));

		// std::cout << IOR << "/" << kf << " ";

		Vector3 T; // ���܂�������
		T = (nom + view) * kf - nom;
		// -----------------------------------------------------------------------------------

		T.normalize();

		rayhit.ray.dir_x = T.x;
		rayhit.ray.dir_y = T.y;
		rayhit.ray.dir_z = T.z;

		//std::cout << rayhit.ray.dir_x << " " << rayhit.ray.dir_x << " " << rayhit.ray.dir_z << std::endl;

		// �������肷��͈͂��w�� 
		rayhit.ray.tnear = 0.011f;     // �͈͂̎n�_ //���ܗ��P�̎�0.0���ƃG���[

		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		// ��������
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

			//�@�F���~�b�N�X
			mix(pixel_color, col, transparency);

			// ���܂̍l��
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



	// ���ˎ��̌�_�v�Z
	// ���ˉ񐔂�myscene.reflecting_limit�Ő��������
	// ���ˌ��2��ڈȍ~�̌�_�v�Z�͂����ōs���B
	// ���݋��܂Ƃ�(���m�ɂ�)�����ł��Ȃ��B
	void reflection_crossing_judgement(
		Colorub& pixel_color,			//return
		RTCRayHit& prev_ray,
		RTCScene& scene, MyScene& myscene,
		int prev_obj_id,
		int count)
	{
		std::cout << "regacy!";

		double reflectivity = (*myscene.object[prev_obj_id]).mat.reflectivity;

		// �V����RAY���`
		struct RTCRayHit rayhit;

		// ���C�̎n�_ 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);


		// �@���̎擾
		Vector3 nom = get_normal(prev_ray, myscene, prev_obj_id);
		// �X���[�Y�V�F�[�f�B���O���g���ƁA���肦�Ȃ��Փ˂��N�����Ă��܂��B
		nom.normalize();


		// ------- ���˂��������̌v�Z -------------------------------------------------------
		Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
		view.normalize();

		Vector3 R; // ���˂�������
		R = view - (2. * (view.dot(nom)) * nom);
		// -----------------------------------------------------------------------------------

		// �Ő��t�߂ł͂��܂����˂��v�Z�ł��Ȃ��_������B
		if (view.dot(nom) > 0) {
			std::cout << "ERROR : " << count << " " << prev_ray.hit.primID << " " << prev_ray.hit.geomID << " ";
			pixel_color = COL_ERROR;
			return;
		}

		R.normalize();

		rayhit.ray.dir_x = R.x;
		rayhit.ray.dir_y = R.y;
		rayhit.ray.dir_z = R.z;

		// �������肷��͈͂��w�� 
		rayhit.ray.tnear = 0.09f;     // �͈͂̎n�_ //���ܗ��P�̎�0.0���ƃG���[

		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		// ��������
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

			//�@�F���~�b�N�X
			mix(pixel_color, col, reflectivity);

			// �ēx���˂̍l��
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

	// �e�N�X�`��������ꍇ�̋P�x�v�Z
	Colorub calc_color_with_texture(RTCRayHit& rayhit, MyScene& myscene, int obj_id) {
		// uv���W���v�Z
		Eigen::RowVectorXi ftc = (*myscene.object[obj_id]).FTC.row(rayhit.hit.primID);

		Eigen::RowVector2d a = (*myscene.object[obj_id]).TC.row(ftc(0));
		Eigen::RowVector2d b = (*myscene.object[obj_id]).TC.row(ftc(1));
		Eigen::RowVector2d c = (*myscene.object[obj_id]).TC.row(ftc(2));

		Eigen::RowVector2d uv = a + (rayhit.hit.u * (b - a)) + (rayhit.hit.v * (c - a));
		// UV���o��
		//Vector3 uv3(uv(0), uv(1), 0);
		//image.setPixel(i, j, convert_to_color(uv3));

		// ��UV���o��
		//Vector3 uvtemp(rayhit.hit.u, rayhit.hit.v, 0);
		//image.setPixel(i, j, convert_to_color(uvtemp));

		if (0 > uv(0) || uv(0) > 1) {
			std::cout << "u_ERROR " << uv(0) << std::endl;
		}
		if (0 > uv(1) || uv(1) > 1) {
			std::cout << "v_ERROR " << uv(1) << std::endl;
		}
		// �e�N�X�`�����o��
		Colorub texcol = (*myscene.texture[0]).getPixel((int)((double)(*myscene.texture[0]).getWidth() * uv(0)), (int)((double)(*myscene.texture[0]).getHeight() * uv(1)));
		//Colorub texcol = texture.getPixel(i, j);
		return(texcol);
	}



	// - - - �����o�[�g�V�F�[�_�̌v�Z - - -
	Colorub rambert(RTCRayHit& rayhit, MyScene &myscene) {
		int obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
		MyObject* obj = myscene.object[obj_id];

		/* �@���X���[�W���O������ */
		Vector3 n(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
		n.normalize();

		double diffuse = obj->mat.diffuse *  n.dot(myscene.sun); //�f�B�t���[�Y�W��
		
		diffuse = max(0., -1*diffuse);

		double ambient = obj->mat.ambient;				 //�����W��

		Colorub cs;	//���̂̕\�ʐF

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

	// ���܃��C�̌v�Z
	RTCRayHit calc_inflection_ray(RTCRayHit& prev_ray, RTCScene& scene, MyScene& myscene,int prev_obj_id, float IOR){
		
		struct RTCRayHit rayhit;
		
		// ���C�̎n�_ 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);

		// �@���̎擾
		Vector3 nom;
		nom = get_normal2(prev_ray, myscene,scene,  prev_obj_id);

		// ------- ���܂��������̌v�Z -------------------------------------------------------
		Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
		view.normalize();

		// ���ʂɏՓ˂����ꍇ
		if (nom.dot(view) > 0) {
			nom = nom * (-1.0);
			IOR = 1.0 / IOR;
		}

		view = (1 / fabs(view.dot(nom))) * view;	// �v�Z��V�̒������w�肷��K�v������B
		double temp1 = IOR * IOR * view.lengthSquared();
		double temp2 = (nom + view).lengthSquared();
		double kf = 1.0 / sqrt(fabs(temp1 - temp2));

		// std::cout << IOR << "/" << kf << " ";
		Vector3 T; // ���܂�������
		T = (nom + view) * kf - nom;
		// -----------------------------------------------------------------------------------

		T.normalize();
		rayhit.ray.dir_x = T.x;
		rayhit.ray.dir_y = T.y;
		rayhit.ray.dir_z = T.z;

		//std::cout << rayhit.ray.dir_x << " " << rayhit.ray.dir_x << " " << rayhit.ray.dir_z << std::endl;

		// �������肷��͈͂��w�� 
		rayhit.ray.tnear = 0.011f;     // �͈͂̎n�_ //���ܗ��P�̎�0.0���ƃG���[
		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		return rayhit;
	}
	// ���˃��C�̌v�Z
	RTCRayHit calc_reflection_ray(RTCRayHit& prev_ray, RTCScene& scene, MyScene& myscene, int prev_obj_id, float IOR) {

		double reflectivity = (*myscene.object[prev_obj_id]).mat.reflectivity;

		// �V����RAY���`
		struct RTCRayHit rayhit;

		// ���C�̎n�_ 
		rayhit.ray.org_x = prev_ray.ray.org_x + (prev_ray.ray.tfar * prev_ray.ray.dir_x);
		rayhit.ray.org_y = prev_ray.ray.org_y + (prev_ray.ray.tfar * prev_ray.ray.dir_y);
		rayhit.ray.org_z = prev_ray.ray.org_z + (prev_ray.ray.tfar * prev_ray.ray.dir_z);


		// �@���̎擾
		Vector3 nom = get_normal2(prev_ray, myscene, scene, prev_obj_id);
		// �X���[�Y�V�F�[�f�B���O���g���ƁA���肦�Ȃ��Փ˂��N�����Ă��܂��B
		// nom = { prev_ray.hit.Ng_x, prev_ray.hit.Ng_y, prev_ray.hit.Ng_z }; 
		// nom.normalize();

		//std::cout << nom.x << " " << nom.y << " " << nom.z << std::endl;

		// ------- ���˂��������̌v�Z -------------------------------------------------------
		Vector3 view(prev_ray.ray.dir_x, prev_ray.ray.dir_y, prev_ray.ray.dir_z);
		view.normalize();

		// ���ʂɏՓ˂����ꍇ
		if (nom.dot(view) > 0) {
			nom = nom * (-1.0);
			IOR = 1.0 / IOR;
			//std::cout << "a";
		}

		
		Vector3 R; // ���˂�������
		R = view - (2. * (view.dot(nom)) * nom);
		//R = 2. * nom + view;
		// -----------------------------------------------------------------------------------

		// �Ő��t�߂ł͂��܂����˂��v�Z�ł��Ȃ��_������B
		if (view.dot(nom) > 0) {
			//std::cout << "ERROR : " << """count <<""" " " << prev_ray.hit.primID << " " << prev_ray.hit.geomID << " " << nom.x <<"/" << nom.y<<"/"<<nom.z <<" ";
			//pixel_color = COL_ERROR;
			return (rayhit);
		}

		R.normalize();

		rayhit.ray.dir_x = R.x;
		rayhit.ray.dir_y = R.y;
		rayhit.ray.dir_z = R.z;

		// �������肷��͈͂��w�� 
		rayhit.ray.tnear = 0.09f;     // �͈͂̎n�_ //���ܗ��P�̎�0.0���ƃG���[


		rayhit.ray.flags = false;
		rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
		rayhit.ray.tfar = INFINITY;

		return rayhit;
	}
		
		

	// ���߂��������ŏ���������Ă���/�������肪�܂��s���Ă��Ȃ����C��n���D
	Colorub ray_calc(   RTCRayHit& rayhit, 
						RTCScene& scene, 
						MyScene &myscene, 
						Colorub &pixel_color, 
						int inflection_count, 
						int reflection_count   ) {
		
		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		// - - - - �������� - - - - - - - - -
		rtcIntersect1(scene, &context, &rayhit);

		// - - - - �����_�����O�ƃ}�X�N�쐬 - - - - - -
		if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

			if (inflection_count < myscene.inflecting_limit && reflection_count < myscene.reflecting_limit) {
				int       obj_id = myscene.geomID_to_objectID(rayhit.hit.geomID);
				MyObject* obj = myscene.object[obj_id];
				Vector3 dir(rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z);
				Vector3 nom(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
				nom.normalize();

				// �x�[�X�J���[�̌v�Z
				Colorub basecol = rambert(rayhit, myscene);
				pixel_color = basecol;


				// ���ߗ���1�����̏ꍇ�C���ˁC���܂̍l�� 
				if (obj->mat.transparency != 0) {


					// ----------------  reduction�p�����d�݂Â� --------------------------------- 
					int fid = rayhit.hit.primID;
					Eigen::VectorXi ff = obj->InfRefMAP.row(fid);
					if ((ff(0) == -1) || inflection_count < ff(0)) {
						obj->InfRefMAP(fid, 0) = inflection_count;
					}
					if ((ff(1) == -1) || reflection_count < ff(1)) {
						obj->InfRefMAP(fid, 1) = reflection_count;
					}// -----------------------------------------------------------------------------

					// ---------- �t���l���̌v�Z�F���������˂��邩(�c��͋���) ------------------------
					float flesnel;
					float IOR = (*myscene.object[obj_id]).mat.IOR;
					// ���ʂɏՓ˂����ꍇ
					if (nom.dot(dir) > 0) {
						nom = nom * (-1.0);
					}
					float f0 = pow(((1 - IOR) / (1 + IOR)), 2);
					float cosine = -1 * (dir.dot(nom));						//���ς���cos�̌v�Z
					flesnel = f0 + (1 - f0) * pow((1 - cosine), 5);

					//flesnel = 1;
					if (flesnel < 0 || flesnel > 1) std::cout << "flesnel error : " << flesnel;
					// --------------------------------------------------------------------------------


					/*���߃}�e���A���͖������łƂ肠�������܂����˂��v�Z���Ă��邩�烀�_����*/

					// ���܃��C���v�Z
					Colorub col_inf;
					if (flesnel != 1) {
						struct RTCRayHit inf_ray = calc_inflection_ray(rayhit, scene, myscene, obj_id, IOR);
						col_inf = ray_calc(inf_ray, scene, myscene, pixel_color,
							inflection_count + 1, reflection_count);
					}
					else { col_inf = COL_BLACK; }

					// ���˃��C���v�Z
					Colorub col_ref;
					if (flesnel != 0) {
						struct RTCRayHit ref_ray = calc_reflection_ray(rayhit, scene, myscene, obj_id, IOR);
						col_ref = ray_calc(ref_ray, scene, myscene, pixel_color,
							inflection_count, reflection_count + 1);
					}
					else { col_ref = COL_BLACK; }

					// ���܁C���ˁC�x�[�X�J���[���t���l���Ɣ��˗��Ɋ�Â��č���
					Colorub rr = mix2(col_inf, col_ref, flesnel);
					pixel_color = mix2(basecol, rr, obj->mat.transparency);
				}
			}
			else { // �Փˉ񐔐���
				ray_to_backgrownd(rayhit, myscene, pixel_color);
				pixel_color = COL_BLACK;
			}
		}
		else { // ��Փ�
			ray_to_backgrownd(rayhit, myscene, pixel_color);
		}

		return( pixel_color);
	}


	// - - - ���C�Ƃ̌������� - - - 
	bool crossing_judgement(Image& image, RTCScene& scene, MyScene myscene) {

		// ���}�b�v
		Image temp;
		Image* envmap = &temp;
		if (myscene.use_environment_map) {
			for (Image* ptr : myscene.texture) {
				if ((*ptr).getType() == "ENV") {
					envmap = ptr;
				}
			}
		}

		// ���C�𐶐����� 
		struct RTCRayHit rayhit;

		// ���C�̎n�_ 
		rayhit.ray.org_x = myscene.camera.view.x;  // x
		rayhit.ray.org_y = myscene.camera.view.y;  // y
		rayhit.ray.org_z = myscene.camera.view.z;  // z

		// �������肷��͈͂��w�� 
		rayhit.ray.tnear = 0.09f;     // �͈͂̎n�_

		// �������� 
		struct RTCIntersectContext context;
		rtcInitIntersectContext(&context);

		for (int j = 0; j < image.getHeight(); j++) {
			for (int i = 0; i < image.getWidth(); i++) {

				//std::cout << "[" << i << "," << j << "]";

				// ���̃s�N�Z���̐F
				Colorub pixel_color = COL_ERROR;

				// - - - -  RAY�̏����� - - - - - - -
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