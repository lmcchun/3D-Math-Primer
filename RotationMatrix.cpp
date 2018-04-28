#include "stdafx.h"
#include "RotationMatrix.h"
#include "EulerAngles.h"
#include "Quaternion.h"

// ��Ϊ��λ����
void RotationMatrix::identity()
{
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
}

// ��ŷ���ǲ����������
void RotationMatrix::setup(const EulerAngles &orientation)
{
	// ����Ƕȵ� sin �� cos ֵ
	float sh, ch, sp, cp, sb, cb;
	sinCos(&sh, &ch, orientation.heading);
	sinCos(&sp, &cp, orientation.pitch);
	sinCos(&sb, &cb, orientation.bank);

	// ������
	m11 = ch * cb + sh * sp * sb;
	m12 = -ch * sb + sh * sp * cb;
	m13 = sh * cp;

	m21 = sb * cp;
	m22 = cb * cp;
	m23 = -sp;

	m31 = -sh * cb + ch * sp * sb;
	m32 = sb * sh + ch * sp * cb;
	m33 = ch * cp;
}

// ���ݹ���-������ת��Ԫ���������
void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion &q)
{
	// ������, �����Ż��Ŀ���, ��Ϊ����ӱ��ʽ����ͬ��, ���ǰ��Ż���������������
	m11 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	m12 = 2.0f * (q.x * q.y + q.w * q.z);
	m13 = 2.0f * (q.x * q.z - q.w * q.y);

	m21 = 2.0f * (q.x * q.y - q.w * q.z);
	m22 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
	m23 = 2.0f * (q.y * q.z + q.w * q.x);

	m31 = 2.0f * (q.x * q.z + q.w * q.y);
	m32 = 2.0f * (q.y * q.z - q.w * q.x);
	m33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
}

// ��������-������ת��Ԫ���������
void RotationMatrix::fromObjectToInertialQuaternion(const Quaternion &q)
{
	// ������, �����Ż��Ŀ���, ��Ϊ����ӱ��ʽ����ͬ��, ���ǰ��Ż���������������
	m11 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
	m12 = 2.0f * (q.x * q.y - q.w * q.z);
	m13 = 2.0f * (q.x * q.z + q.w * q.y);

	m21 = 2.0f * (q.x * q.y + q.w * q.z);
	m22 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
	m23 = 2.0f * (q.y * q.z - q.w * q.x);

	m31 = 2.0f * (q.x * q.z - q.w * q.y);
	m32 = 2.0f * (q.y * q.z + q.w * q.x);
	m33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
}

// ������������-����任
Vector3 RotationMatrix::inertialToObject(const Vector3 &v) const
{
	// ��"��׼"��ʽִ�о���˷�
	return Vector3(
		m11 * v.x + m21 * v.y + m31 * v.z,
		m12 * v.x + m22 * v.y + m32 * v.z,
		m13 * v.x + m23 * v.y + m33 * v.z);
}

// ������������-���Ա任
Vector3 RotationMatrix::objectToInertial(const Vector3 &v) const
{
	// ����ת��
	return Vector3(
		m11 * v.x + m12 * v.y + m13 * v.z,
		m21 * v.x + m22 * v.y + m23 * v.z,
		m31 * v.x + m32 * v.y + m33 * v.z);
}