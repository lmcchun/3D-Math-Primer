#include "stdafx.h"
#include "EulerAngles.h"
#include "Matrix4x3.h"
#include "Quaternion.h"
#include "RotationMatrix.h"

// ȫ��"��λ"ŷ���ǳ���
const EulerAngles kEulerAnglesIdentity{ 0.0f, 0.0f, 0.0f };

// ��ŷ����ת�������Ƽ���
// �ͱ�ʾ 3D ��λ��Ŀ�Ķ���, ������ı�ŷ���ǵ�ֵ
// ������������ʾ��������ٶȵ�, ������Ӱ��

void EulerAngles::canonize()
{
	// ����, �� pitch �任�� -Pi �� Pi ֮��
	pitch = wrapPi(pitch);

	// ����, �� pitch �任�� -Pi/2.0 �� Pi/2.0 ֮��
	if (pitch < -kPiOver2)
	{
		pitch = -kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
	else if (pitch > kPiOver2)
	{
		pitch = kPi - pitch;
		heading += kPi;
		bank += kPi;
	}

	// ���ڼ�������������, �������һ�������
	if (fabs(pitch) > kPiOver2 - 1e-4)
	{
		// ����������, �������ƴ�ֱ�����ת���� heading
		heading += bank;
		bank = 0.0f;
	}
	else
	{
		// ����������, �� bank ת�������Ƽ���
		bank = wrapPi(bank);
	}

	// �� heading ת�������Ƽ���
	heading = wrapPi(heading);
}

// ����-������Ԫ����ŷ����
void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q)
{
	// ���� sin(pitch)
	float sp = -2.0f * (q.y * q.z - q.w * q.x);

	// ���������, ����һ�����
	if (fabs(sp) > 0.9999f)
	{
		// �����Ϸ������·���
		pitch = kPiOver2 * sp;

		// bank ����, ���� heading
		heading = atan2(-q.x * q.z + q.w * q.y, 0.5f - q.y * q.y - q.z * q.z);
		bank = 0.0f;
	}
	else
	{
		// ����Ƕ�, �������ǲ���ʹ��"��ȫ��" asin ����, ��Ϊ֮ǰ�������������ʱ�Ѽ�����Χ������
		pitch = asin(sp);
		heading = atan2(q.x * q.z + q.w * q.y, 0.5f - q.x * q.x - q.y * q.y);
		bank = atan2(q.x * q.y + q.w * q.z, 0.5f - q.x * q.x - q.z * q.z);
	}
}

// ����-������Ԫ����ŷ����
void EulerAngles::fromInertialToObjectQuaternion(const Quaternion &q)
{
	// ���� sin(pitch)
	float sp = -2.0f * (q.y * q.z + q.w * q.x);
	// ���������, ����һ�����
	if (fabs(sp) > 0.9999f)
	{
		// �����Ϸ������·���
		pitch = kPiOver2 * sp;
		// bank ����, ���� heading
		heading = atan2(-q.x * q.z - q.w * q.y, 0.5f - q.y * q.y - q.z * q.z);
		bank = 0.0f;
	}
	else
	{
		// ����Ƕ�
		pitch = asin(sp);
		heading = atan2(q.x * q.z - q.w * q.y, 0.5f - q.x * q.x - q.y * q.y);
		bank = atan2(q.x * q.y - q.w * q.z, 0.5f - q.x * q.x - q.z * q.z);
	}
}

// ������-��������ϵ�任����ŷ����
// ���������������, ����ƽ�Ʋ���
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m)
{
	// ͨ�� m32 ���� sin(pitch)
	float sp = -m.m32;
	// ���������
	if (fabs(sp) > 0.99999f)
	{
		// �����Ϸ������·���
		pitch = kPiOver2 * sp;
		// bank ����, ���� heading
		heading = atan2(-m.m13, m.m11); // XXX
		bank = 0.0f;
	}
	else
	{
		// ����Ƕ�
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}

// ������-��������ϵ�任����ŷ����
// ���������������, ����ƽ�Ʋ���
void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m)
{
	// ���� m23 ���� sin(pitch)
	float sp = -m.m23;

	// ���������
	if (fabs(sp) > 0.99999f)
	{
		// �����Ϸ������·���
		pitch = kPiOver2 * sp;
		// bank ����, ���� heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else
	{
		// ����Ƕ�
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}

// ������ת������ŷ����
void EulerAngles::fromRotationMatrix(const RotationMatrix &m)
{
	// ���� m23 ���� sin(pitch)
	float sp = -m.m23;

	// ���������
	if (fabs(sp) > 0.99999f)
	{
		// �����Ϸ������·���
		pitch = kPiOver2 * sp;
		// bank ����, ���� heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else
	{
		// ����Ƕ�
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}