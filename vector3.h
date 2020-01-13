#ifndef VECTOR_3_H
#define VECTOR_3_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>

//----------------------------------------------------------------------------

class Vector3
{
public:
	double	x, y, z;

	// �R���X�g���N�^
	Vector3() { x = y = z = 0;}
	Vector3(double a, double b, double c ) { x=a; y=b; z=c;}
	/// ���
	Vector3& operator = ( const Vector3& b )
	{
		x = b.x;
		y = b.y;
		z = b.z;
		return *this;
	}

	/// �������]
	Vector3 operator - ()
	{
		return Vector3( -x, -y, -z );
	}

	/// �a
	Vector3 operator + ( const Vector3& b )
	{
		return Vector3( x + b.x, y + b.y, z + b.z );
	}
	/// ��
	Vector3 operator - ( const Vector3& b )
	{
		return Vector3( x - b.x, y - b.y, z - b.z );
	}
	/// �x�N�g�� x ���l
	Vector3 operator * ( double t )
	{
		return Vector3( x * t, y * t, z * t );
	}
	///  ���l x �x�N�g��
	friend inline Vector3 operator * ( double t, Vector3& b )
	{
		return b * t;
	}
	/// ��
	Vector3 operator / ( const double t ) 
	{
		return Vector3( x / t, y / t, z / t );
	}

	// ���g�ɑ΂���a���Ϗ�
	/// �a
	Vector3& operator += ( Vector3& b ) 
	{
		x += b.x;
		y += b.y;
		z += b.z;
		return *this;
	}
	/// ��
	Vector3& operator -= ( Vector3& b ) 
	{
		x -= b.x;
		y -= b.y;
		z -= b.z;
		return *this;
	}
	/// ��
	Vector3& operator *= ( double t ) 
	{
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}
	/// ��
	Vector3& operator /= ( double t )
	{
		x /= t;
		y /= t;
		z /= t;
		return *this;
	}

	/// ����
	double dot( const Vector3& b )
	{
		return x * b.x + y * b.y + z * b.z;
	}

	/// �O��
	Vector3 cross( const Vector3& b )
	{
		return Vector3( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x );
	}

	/// ����
	double lengthSquared() 
	{
		return this->dot(*this);
	}
	double length() 
	{
		return (double)sqrt( (double)this->lengthSquared() );
	}

	/// �P�ʃx�N�g����
	Vector3& normalize()
	{
		(*this) /= this->length();
		return *this;
	}

	// ��r
	/// ������
	bool operator == ( Vector3& b )
	{
		return( b.x==x && b.y==y && b.z == z );
	}

	/// �قȂ邩
	bool operator != ( Vector3& b )
	{
		return !( b == *this );
	}
};

#endif