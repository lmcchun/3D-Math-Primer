#include "stdafx.h"
#include "Matrix4x3.h"
#include <assert.h>
#include "EulerAngles.h"
#include "Quaternion.h"
#include "RotationMatrix.h"

// ��Ϊ��λ����
void Matrix4x3::identity()
{
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = 0.0f; ty = 0.0f; tz = 0.0f; // XXX
}

// ������ƽ�Ʋ��ֵĵ�������Ϊ��
void Matrix4x3::zeroTranslation()
{
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// ƽ�Ʋ��ָ�ֵ, ����Ϊ������ʽ
void Matrix4x3::setTranslation(const Vector3 &d)
{
	tx = d.x;
	ty = d.y;
	tz = d.z;
}

// ƽ�Ʋ��ָ�ֵ, ����Ϊ������ʽ
void Matrix4x3::setupTranslation(const Vector3 &d)
{
	// ���Ա任������Ϊ��λ����
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	// ƽ�Ʋ��ָ�ֵ
	tx = d.x; ty = d.y; tz = d.z;
}

// ����ִ�оֲ�->���ռ�任�ľ���, �ֲ��ռ��λ�úͷ�λ�ڸ��ռ�������,
// �÷����������;�ǹ�������-����ı任����, ����任�Ƿǳ���ֱ�ӵ�.
// ���ȴ�����ռ�任�����Կռ�, ���ű任������ռ�
// ��λ������ŷ���ǻ���ת����ָ��
void Matrix4x3::setupLocalToParent(const Vector3 &pos, const EulerAngles &orient)
{
	// ����һ����ת����
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);
	// ���� 4x3 ����
	// ע��, ����ٶ�������Ҫ, ���ǿ���ֱ�Ӽ������, ���� RotationMatrix ��ʱ����,
	// �⽫��ʡһ�κ������ú�һЩ���Ʋ���
	setupLocalToParent(pos, orientMatrix);
}

void Matrix4x3::setupLocalToParent(const Vector3 &pos, const RotationMatrix &orient)
{
	// ���ƾ������ת����
	// ���� RotationMatrix.cpp �е�ע��, ��ת����"һ��"�ǹ���-�������, �Ǹ�-�ֲ���ϵ,
	// ��������Ǿֲ�-����ϵ�ľ���, ���Ҫ��ת��
	m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
	m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
	m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;
	// ��������ƽ�Ʋ���, ƽ���� 3x3 ����"֮��",�������ֻ��򵥵ĸ�����λ�ü���
	tx = pos.x; ty = pos.y; tz = pos.z;
}

// ����ִ�и�-�ֲ��ռ�任����, �ֲ��ռ��λ�úͷ�λ�ڸ��ռ�������
// �÷����������;�ǹ�������-����ı任����,
// ͨ������任���ȴ�����ռ�ת�������Կռ�, ����ת��������ռ�
// 4x3 ���������ɺ�һ��ת��,
// ���������빹���������� T �� R, ������ M=TR
void Matrix4x3::setupParentToLocal(const Vector3 &pos, const EulerAngles &orient)
{
	// ����һ����ת����
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);
	// ���� 4x3 ����
	setupParentToLocal(pos, orientMatrix);
}

void Matrix4x3::setupParentToLocal(const Vector3 &pos, const RotationMatrix &orient)
{
	// ���ƾ������ת����
	// ����ֱ�Ӹ���Ԫ��(����ת��), ���� RotationMatrix.cpp �е�ע�͵����з�ʽ����
	m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
	m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
	m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;
	// ����ƽ�Ʋ���,
	// һ����˵, ������ռ䵽���Կռ�ֻ��ƽ�Ʋ��ֵ���
	// ������ǵ���ת��"��"������, ����Ӧ����תƽ�Ʋ���,
	// ����ȴ���ƽ��-pos �ľ��� T, �ڴ�����ת���� R,
	// �ٰ��������ӳ� TR ��һ����
	tx = -(pos.x * m11 + pos.y * m21 + pos.z * m31);
	ty = -(pos.x * m12 + pos.y * m22 + pos.z * m32);
	tz = -(pos.x * m13 + pos.y * m23 + pos.z * m33);
}

// ��������������ת�ľ���
// ��ת����һ���� 1 ��ʼ������ָ��
// 1 => �� x ����ת
// 2 => �� y ����ת
// 3 => �� z ����ת
// theta ����ת��, �Ի��ȱ�ʾ, �����ַ�����"������"
// ƽ�Ʋ�������
void Matrix4x3::setupRotate(int axis, float theta)
{
	// ȡ����ת�ǵ� sin �� cos ֵ
	float s, c;
	sinCos(&s, &c, theta);
	// �ж���ת��
	switch (axis)
	{
	case 1: // �� x ����ת
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = c; m23 = s;
		m31 = 0.0f; m32 = -s; m33 = c;
		break;
	case 2: // �� y ����ת
		m11 = c; m12 = 0.0f; m13 = -s;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s; m32 = 0.0f; m33 = c;
		break;
	case 3: // �� z ����ת
		m11 = c; m12 = s; m13 = 0.0f;
		m21 = -s; m22 = c; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	default:
		// �Ƿ�����
		assert(false);
	}
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// ���������������ת, ��ת��ͨ��ԭ��
// ��ת��Ϊ��λ����
// theta ����ת����, �Ի��ȱ�ʾ, �����ַ���������"������"
// ƽ�Ʋ�������
void Matrix4x3::setupRotate(const Vector3 &axis, float theta)
{
	// �����ת���Ƿ�Ϊ��λ����
	assert(fabs(axis * axis - 1.0f) < 0.01f);
	// ȡ����ת�ǵ� sin �� cos ֵ
	float s, c;
	sinCos(&s, &c, theta);
	// ���� 1 - cosv(theta) ��һЩ���õ��ӱ��ʽ
	float a = 1.0f - c;
	float ax = a * axis.x;
	float ay = a * axis.y;
	float az = a * axis.z;
	// ����Ԫ�صĸ�ֵ
	// �����Ż��Ļ���, ��Ϊ�������ͬ���ӱ��ʽ, ���ǰ������������������
	m11 = ax * axis.x + c;
	m12 = ax * axis.y + axis.z * s;
	m13 = ax * axis.z - axis.y * s;
	m21 = ay * axis.x - axis.z * s;
	m22 = ay * axis.y + c;
	m23 = ay * axis.z + axis.x * s;
	m31 = az * axis.x + axis.y * s;
	m32 = az * axis.y - axis.x * s;
	m33 = az * axis.z + c;
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// ����Ԫ��ת��������
// ƽ�Ʋ�������
void Matrix4x3::fromQuaternion(const Quaternion &q)
{
	// ����һЩ���õ��ӱ��ʽ
	float ww = 2.0f * q.w;
	float xx = 2.0f * q.x;
	float yy = 2.0f * q.y;
	float zz = 2.0f * q.z;
	// ����Ԫ�صĸ�ֵ
	// ��Ȼ���Ż��Ļ���, ��Ϊ�������ͬ���ӱ��ʽ, ���ǰ������������������
	m11 = 1.0f - yy * q.y - zz * q.z;
	m12 = xx * q.y + ww * q.z;
	m13 = xx * q.z - ww * q.x;
	m21 = xx * q.y - ww * q.z;
	m22 = 1.0f - xx * q.x - zz * q.z;
	m23 = yy * q.z + ww * q.x;
	m31 = xx * q.z + ww * q.y;
	m32 = yy * q.z - ww * q.x;
	m33 = 1.0f - xx * q.x - yy * q.y;
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// �����ظ����������ŵľ���
// ������������ k, ʹ������ Vector3(k, k, k) ��ʾ
void Matrix4x3::setupScale(const Vector3 &s)
{
	// ����Ԫ�ظ�ֵ, �ǳ�ֱ��
	m11 = s.x; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = s.y; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = s.z;
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// ���������������ŵľ���
// ��ת��Ϊ��λ����
// ƽ�Ʋ�������
void Matrix4x3::setupScaleAlongAxis(const Vector3 &axis, float k)
{
	// �����ת���Ƿ�λ����
	assert(fabs(axis * axis - 1.0f) < 0.01f);
	// ���� k - 1 �ͳ��õ��ӱ��ʽ
	float a = k - 1.0f;
	float ax = a * axis.x;
	float ay = a * axis.y;
	float az = a * axis.z;
	// ����Ԫ�ظ�ֵ, ���������Լ�����Ż�����, ��Ϊ��Խ�Ԫ�����
	m11 = ax * axis.x + 1.0f;
	m22 = ay * axis.y + 1.0f;
	m33 = az * axis.z + 1.0f;
	m12 = m21 = ax * axis.y;
	m23 = m32 = ay * axis.z;
	m31 = m13 = az * axis.x;
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// �����б����
// �б������һ������ָ��, �任Ч��������α������ʾ:
// axis == 1 => y += s * x, z += t * x
// axis == 2 => z += t * y, x += s * y
// axis == 3 => x += s * z, y += t * z
// ƽ�Ʋ�������
void Matrix4x3::setupShear(int axis, float s, float t)
{
	// �ж��б�����
	switch (axis)
	{
	case 1: // �� x �б� y �� z
		m11 = 1.0f; m12 = s; m13 = t;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	case 2: // �� y �б� z �� x
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = s; m22 = 1.0f; m23 = t;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	case 3: // �� z �б� x �� y
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s; m32 = t; m33 = 1.0f;
		break;
	default:
		// �Ƿ�����
		assert(false);
		break;
	}
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// ����ͶӰ����, ͶӰƽ���ԭ��, �Ҵ�ֱ�ڵ�λ���� n
void Matrix4x3::setupProject(const Vector3 &n)
{
	// �����ת���Ƿ�λ����
	assert(fabs(n * n - 1.0f) < 0.01f);
	// ����Ԫ�ظ�ֵ, ���������Լ�����Ż�����, ��Ϊ��Խ�Ԫ�����
	m11 = 1.0f - n.x * n.x;
	m22 = 1.0f - n.y * n.y;
	m33 = 1.0f - n.z * n.z;
	m12 = m21 = -n.x * n.y;
	m23 = m31 = -n.y * n.z;
	m31 = m13 = -n.z * n.x;
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// ���췴�����, ����ƽ��ƽ��������ƽ��
// ����ƽ����һ������ָ��
// 1 => �� x=k ƽ�淴��
// 2 => �� y=k ƽ�淴��
// 3 => �� z=k ƽ�淴��
// ƽ�Ʋ�����Ϊ���ʵ�ֵ, ��Ϊ k!=0ʱ ƽ����һ���ᷢ����
void Matrix4x3::setupReflect(int axis, float k)
{
	// �жϷ���ƽ��
	switch (axis)
	{
	case 1: // �� x=k ƽ�淴��
		m11 = -1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		tx = 2.0f * k;
		ty = 0.0f;
		tz = 0.0f;
		break;
	case 2: // �� y=k ƽ�淴��
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = -1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		tx = 0.0f;
		ty = 2.0f * k;
		tz = 0.0f;
		break;
	case 3: // �� z=k ƽ�淴��
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = -1.0f;
		tx = 0.0f;
		ty = 0.0f;
		tz = 2.0f * k;
		break;
	default:
		// �Ƿ�����
		assert(false);
		break;
	}
}

// ���췴�����, ����ƽ��Ϊͨ��ԭ�������ƽ��, �Ҵ�ֱ�ڵ�λ����
// ƽ�Ʋ�������
void Matrix4x3::setupReflect(const Vector3 &n)
{
	// �����ת���Ƿ�λ����
	assert(fabs(n * n - 1.0f) < 0.01f);
	// ���㹫���ӱ��ʽ
	float ax = -2.0f * n.x;
	float ay = -2.0f * n.y;
	float az = -2.0f * n.z;
	// ����Ԫ�ظ�ֵ, ���������Լ�����Ż�����, ��Ϊ��Խ�Ԫ�����
	m11 = 1.0f + ax * n.x;
	m22 = 1.0f + ay * n.y;
	m33 = 1.0f + az * n.z;
	m12 = m21 = ax * n.y;
	m23 = m32 = ay * n.z;
	m31 = m13 = az * n.x;
	// ƽ�Ʋ�������
	tx = 0.0f;
	ty = 0.0f;
	tz = 0.0f;
}

// �任�õ�, ��ʹ��ʹ�������������ֽ�������Դ���һ��ֱ��
// �ṩ *= �����, �Է��� C ���Ե��﷨ϰ��
Vector3 operator*(const Vector3 &p, const Matrix4x3 &m)
{
	// �������Դ�������
	return Vector3(
		p.x * m.m11 + p.y * m.m21 + p.z * m.m31 + m.tx,
		p.x * m.m12 + p.y * m.m22 + p.z * m.m32 + m.ty,
		p.x * m.m13 + p.y * m.m23 + p.z * m.m33 + m.tz);
}

Vector3 &operator*=(Vector3 &p, const Matrix4x3 &m)
{
	p = p * m;
	return p;
}

// ��������, ��ʹ��ʹ�þ����������ֽ�������Դ���һ��ֱ��
// �ṩ *= �����, �Է��� C ���Ե��﷨ϰ��
Matrix4x3 operator*(const Matrix4x3 &a, const Matrix4x3 &b)
{
	Matrix4x3 r;
	// �������ϵ� 3x3 (���Ա任) ����
	r.m11 = a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
	r.m12 = a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
	r.m13 = a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;
	r.m21 = a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
	r.m22 = a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
	r.m23 = a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;
	r.m31 = a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
	r.m32 = a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
	r.m33 = a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;
	// ����ƽ�Ʋ���
	r.tx = a.tx * b.m11 + a.ty * b.m21 + a.tz * b.m31 + b.tx;
	r.ty = a.tx * b.m12 + a.ty * b.m22 + a.tz * b.m32 + b.ty;
	r.tz = a.tx * b.m13 + a.ty * b.m23 + a.tz * b.m33 + b.tz;
	// ����ֵ, ���ַ�����Ҫ���ÿ������캯��
	// ����ٶȷǳ���Ҫ, ���ǿ�����Ҫ�õ����ĺ����������ĵط���������ֵ
	return r;
}

Matrix4x3 &operator*=(Matrix4x3 &a, const Matrix4x3 &b)
{
	a = a * b;
	return a;
}

// ����������� 3x3 ���ֵ�����ʽ
float determinant(const Matrix4x3 &m)
{
	return m.m11 * (m.m22 * m.m33 - m.m23 * m.m32)
		+ m.m12 * (m.m23 * m.m31 - m.m21 * m.m32)
		+ m.m13 * (m.m21 * m.m32 - m.m22 * m.m31);
}

// ��������, ʹ�þ���İ�������������ʽ�ķ���
Matrix4x3 inverse(const Matrix4x3 &m)
{
	// ��������ʽ
	float det = determinant(m);
	// ����������, ������ʽΪ��, ��û�������
	assert(fabs(det) > 0.000001f);
	// ���� 1/����ʽ, �����ĳ���ֻ��Ҫһ��
	float oneOverDet = 1.0f / det;
	// ���� 3x3 ���ֵ���, �ð�������������ʽ
	Matrix4x3 r;
	r.m11 = (m.m22 * m.m33 - m.m23 * m.m32) * oneOverDet;
	r.m12 = (m.m13 * m.m32 - m.m12 * m.m33) * oneOverDet;
	r.m13 = (m.m12 * m.m23 - m.m13 * m.m22) * oneOverDet;
	r.m21 = (m.m23 * m.m31 - m.m21 * m.m33) * oneOverDet;
	r.m22 = (m.m11 * m.m33 - m.m13 * m.m31) * oneOverDet;
	r.m23 = (m.m13 * m.m21 - m.m11 * m.m23) * oneOverDet;
	r.m31 = (m.m21 * m.m32 - m.m22 * m.m31) * oneOverDet;
	r.m32 = (m.m12 * m.m31 - m.m11 * m.m32) * oneOverDet;
	r.m33 = (m.m11 * m.m22 - m.m12 * m.m21) * oneOverDet;
	// ����ƽ�Ʋ��ֵ���
	r.tx = -(m.tx * r.m11 + m.ty * r.m21 + m.tz * r.m31);
	r.ty = -(m.tx * r.m12 + m.ty * r.m22 + m.tz * r.m32);
	r.tz = -(m.tx * r.m13 + m.ty * r.m23 + m.tz * r.m33);
	// ����ֵ, ���ַ�����Ҫ���ÿ������캯��
	// ����ٶȷǳ���Ҫ, ���ǿ�����Ҫ�õ����ĺ����������ĵط���������ֵ
	return r;
}

// ����������ʽ����ƽ�Ʋ���
Vector3 getTranslation(const Matrix4x3 &m)
{
	return Vector3(m.tx, m.ty, m.tz);
}

// �Ӹ�->�ֲ�(����->����)�任��������ȡ�����λ��
// �������������任
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3 &m)
{
	// ����ƽ��ֵ���� 3x3 ���ֵ�ת��
	// ���������������(�÷�������Ӧ���ڷǸ���任)
	return Vector3(
		-(m.tx * m.m11 + m.ty * m.m12 + m.tz * m.m13),
		-(m.tx * m.m21 + m.ty * m.m22 + m.tz * m.m23),
		-(m.tx * m.m31 + m.ty * m.m32 + m.tz * m.m33));
}

// �Ӿֲ�->��(������->����)�任��������ȡ�����λ��
Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3 &m)
{
	//  �����λ�þ���ƽ�Ʋ���
	return Vector3(m.tx, m.ty, m.tz);
}