#include "stdafx.h"
#include "Quaternion.h"
#include <assert.h>
#include "EulerAngles.h"

// ȫ������
const Quaternion kQuaternionIdentity = { 1.0f, 0.0f, 0.0f, 0.0f };

// ������ָ������ת����Ԫ��
void Quaternion::setToRotateAboutX(float theta)
{
	// ������
	float thetaOver2 = theta * 0.5f;
	// ��ֵ
	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta)
{
	// ������
	float thetaOver2 = theta * 0.5f;
	// ��ֵ
	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta)
{
	// ������
	float thetaOver2 = theta * 0.5f;
	// ��ֵ
	w = cos(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta)
{
	// ��ת������׼��
	assert(fabs(vectorMag(axis) - 1.0f) < 0.01f);
	// �����Ǻ� sin ֵ
	float thetaOver2 = theta * 0.5f;
	float sinThetaOver2 = sin(thetaOver2);
	// ��ֵ
	w = cos(thetaOver2);
	x = axis.x * sinThetaOver2;
	y = axis.y * sinThetaOver2;
	z = axis.z * sinThetaOver2;
}

// ����ִ������-������ת����Ԫ��
// ��λ������ŷ������ʽ����
void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation)
{
	// �����ǵ� sin �� cos ֵ
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch * 0.5f);
	sinCos(&sb, &cb, orientation.bank * 0.5f);
	sinCos(&sh, &ch, orientation.heading * 0.5f);
	// ������
	w = ch * cp * cb + sh * sp * sb;
	x = ch * sp * cb + sh * cp * sb;
	y = -ch * sp * sb + sh * cp * cb;
	z = -sh * sp * cb + ch * cp * sb;
}

// ����ִ�й���-������ת����Ԫ��
// ��λ������ŷ������ʽ����
void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation)
{
	// �����ǵ� sin �� cos ֵ
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch * 0.5f);
	sinCos(&sb, &cb, orientation.bank * 0.5f);
	sinCos(&sh, &ch, orientation.heading * 0.5f);
	// ������
	w = ch * cp * cb + sh * sp * sb;
	x = -ch * sp * cb - sh * cp * sb;
	y = ch * sp * sb - sh * cp * cb;
	z = sh * sp * cb - ch * cp * sb;
}

// ��Ԫ���������, �������Ӷ����λ��
// �˵�˳���Ǵ�����
// �����Ԫ����˵�"��׼"�����෴
Quaternion Quaternion::operator*(const Quaternion &a) const
{
	Quaternion result;

	result.w = w * a.w - x * a.x - y * a.y - z * a.z;
	result.x = w * a.x + x * a.w + y * a.z - z * a.y;
	result.y = w * a.y + y * a.w + z * a.x - x * a.z;
	result.z = w * a.z + z * a.w + x * a.y - y * a.x;

	return result;
}

// ��˲���ֵ, ���·��� C++ ϰ��д����
Quaternion &Quaternion::operator*=(const Quaternion &a)
{
	// �˲���ֵ
	*this = *this * a;
	// ������ֵ
	return *this;
}

// ������Ԫ��
// ͨ��, ��Ԫ���������򻯵�
// �ṩ���������Ҫ��Ϊ�˷�ֹ�������, ���������Ԫ���������ܵ����������
void Quaternion::normalize()
{
	// ������Ԫ����ģ
	float mag = (float)sqrt(w * w + x * x + y * y + z * z);
	// ��ⳤ��, ��ֹ�������
	if (mag > 0.0f)
	{
		// ����
		float oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else
	{
		// ���鷳��
		assert(false);
		// �ڷ�������, ���ص�λ��Ԫ��
		identity();
	}
}

// ������ת��
float Quaternion::getRotationAngle() const
{
	// ������, w = cos(theta / 2)
	float thetaOver2 = safeAcos(w);
	// ������ת��
	return thetaOver2 * 2.0f;
}

// ��ȡ��ת��
Vector3 Quaternion::getRotationAxis() const
{
	// ���� sin^2(theta/2), ��ס w = cos(theta/2), sin^2(x) + cos^2(x) = 1
	float sinThetaOver2Sq = 1.0f - w * w;
	// ע�Ᵽ֤��ֵ����
	if (sinThetaOver2Sq <= 0.0f)
	{
		// ��λ��Ԫ��������ȷ����ֵ, ֻҪ������Ч����������
		return Vector3(1.0f, 0.0f, 0.0f);
	}
	// ���� 1/sin(theta/2)
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);
	// ������ת��
	return Vector3(x * oneOverSinThetaOver2, y * oneOverSinThetaOver2, z * oneOverSinThetaOver2);
}

// ��Ԫ�����
// �÷ǳ�Ա����ʵ����Ԫ������Ա����ڱ��ʽ��ʹ��ʱ����"�����﷨"
float dotProduct(const Quaternion &a, const Quaternion &b)
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

// �������Բ�ֵ
Quaternion slerp(const Quaternion &q0, const Quaternion &q1, float t)
{
	// ������Ĳ���, �����鵽, ���ر߽��
	if (t <= 0.0f)
	{
		return q0;
	}
	else if (t >= 1.0f)
	{
		return q1;
	}
	else
	{
		// �õ�˼�����Ԫ���нǵ� cos ֵ
		float cosOmega = dotProduct(q0, q1);
		// ������Ϊ��, ʹ�� -q1
		// ��Ԫ�� q �� -q ������ͬ����ת, �����ܲ�����ͬ�� slerp ����, ����Ҫѡ����ȷ��һ���Ա�����ǽ�����ת
		float q1w;
		float q1x;
		float q1y;
		float q1z;
		if (cosOmega < 0.0f)
		{
			q1w = -q1.w;
			q1x = -q1.x;
			q1y = -q1.y;
			q1z = -q1.z;
			cosOmega = -cosOmega;
		}
		else
		{
			q1w = q1.w;
			q1x = q1.x;
			q1y = q1.y;
			q1z = q1.z;
		}

		// �����õ���������λ��Ԫ��, ���Ե�˽��Ӧ�� <= 1.0
		assert(cosOmega < 1.1f);

		// �����ֵƬ, ע����ǳ��ӽ������
		float k0, k1;
		if (cosOmega > 0.9999f)
		{
			// �ǳ��ӽ�
			k0 = 1.0f - t;
			k1 = t;
		}
		else
		{
			// �����ǹ�ʽ sin^2(omega) + cos^2(omega) = 1 ���� sin ֵ
			float sinOmega = sqrt(1.0f - cosOmega * cosOmega);
			// ���� sin �� cos ֵ����Ƕ�
			float omega = atan2(sinOmega, cosOmega);
			// �����ĸ�ĵ���, ����ֻ��Ҫ��һ��
			float oneOverSinOmega = 1.0f / sinOmega;
			// �����ֵ����
			k0 = sin((1.0f - t) * omega) * oneOverSinOmega;
			k1 = sin(t * omega) * oneOverSinOmega;
		}
		// ��ֵ
		Quaternion result;
		result.w = k0 * q0.w + k1 * q1w;
		result.x = k0 * q0.x + k1 * q1x;
		result.y = k0 * q0.y + k1 * q1y;
		result.z = k0 * q0.z + k1 * q1z;
		// ����
		return result;
	}
}

// ��Ԫ������,������Ԫ����ת�����෴����Ԫ��
Quaternion conjugate(const Quaternion &q)
{
	Quaternion result;
	// ��ת����ͬ
	result.w = q.w;
	// ��ת���෴
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;
	// ����
	return result;
}

// ��Ԫ����
Quaternion pow(const Quaternion &q, float exponent)
{
	// ��鵥λ��Ԫ��, ��ֹ����
	if (fabs(q.w) > 0.9999f)
	{
		return q;
	}
	// ��ȡ��� alpha(alpha = theta/2)
	float alpha = acos(q.w);
	// ������ alpha ֵ
	float newAlpha = alpha * exponent;
	// ������ w ֵ
	Quaternion result;
	result.w = cos(newAlpha);
	// ������ xyz ֵ
	float mult = sin(newAlpha) / sin(alpha);
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z * mult;
	// ����
	return result;
}