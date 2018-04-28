#include "stdafx.h"
#include "MathUtil.h"
#include <assert.h>

const Vector3 kZeroVector(0.0f, 0.0f, 0.0f);

// ͨ�������ʵ��� 2Pi ����, ���Ƕ������� -Pi �� Pi ��������
float wrapPi(float theta)
{
	theta += kPi;
	theta -= floor(theta * k1Over2Pi) * k2Pi;
	theta -= kPi;
	return theta;
}

// �� acos(x) ��ͬ, ����� x ������Χ��������Ϊ�ӽ�����Чֵ
// ����ֵ�� 0 �� Pi ֮��, �� C �����еı�׼ acos() ������ͬ
float safeAcos(float x)
{
	// ���߽�����
	if (x <= -1.0f)
	{
		return kPi;
	}
	else if (x >= 1.0f)
	{
		return 0.0f;
	}
	else
	{
		// ʹ�ñ�׼ C ����
		return acos(x);
	}
}

// ����㼯�����ƽ��
Vector3 computeBestFitNormal(const Vector3 v[], int n)
{
	// ������
	Vector3 result = kZeroVector;
	// �����һ�����㿪ʼ, ������ѭ������ if �ж�
	const Vector3 *p = &v[n - 1];
	// �������ж���
	for (int i = 0; i < n; ++i)
	{
		// �õ�"��ǰ"����
		const Vector3 *c = &v[i];
		// �������˻����
		result.x += (p->z + c->z) * (p->y - c->y);
		result.y += (p->x + c->x) * (p->z - c->z);
		result.z += (p->y + c->y) * (p->x - c->x);
		// ��һ������
		p = c;
	}
	// ���򻯽��������
	result.normalize();
	return result;
}

// ���� 3D ����������������
bool computeBarycentricCoords3d(const Vector3 v[3], const Vector3 &p, float b[3])
{
	// ����, ��������������, ��˳ʱ�뷽��
	Vector3 d1 = v[1] - v[0];
	Vector3 d2 = v[2] - v[1];
	// �ò�˼��㷨����, ��������, ��һ��������ʡ��, ��Ϊ��������Ԥ�ȼ����
	// ����Ҫ����, ����Ԥ�ȼ���ķ������Ƿ������
	Vector3 n = crossProduct(d1, d2);
	// �жϷ�������ռ���Ƶ���, ѡ��ͶӰƽ��
	float u1, u2, u3, u4;
	float v1, v2, v3, v4;
	if ((fabs(n.x) >= fabs(n.y)) && (fabs(n.x) >= fabs(n.z)))
	{
		// ���� x, �� yz ƽ��ͶӰ
		u1 = v[0].y - v[2].y;
		u2 = v[1].y - v[2].y;
		u3 = p.y - v[0].y;
		u4 = p.y - v[2].y;
		v1 = v[0].z - v[2].z;
		v2 = v[1].z - v[2].z;
		v3 = p.z - v[0].z;
		v4 = p.z - v[2].z;
	}
	else if (fabs(n.y) >= fabs(n.z))
	{
		// ���� y, �� xz ƽ��ͶӰ
		u1 = v[0].z - v[2].z;
		u2 = v[1].z - v[2].z;
		u3 = p.z - v[0].z;
		u4 = p.z - v[2].z;
		v1 = v[0].x - v[2].x;
		v2 = v[1].x - v[2].x;
		v3 = p.x - v[0].x;
		v4 = p.x - v[2].x;
	}
	else
	{
		u1 = v[0].x - v[2].x;
		u2 = v[1].x - v[2].x;
		u3 = p.x - v[0].x;
		u4 = p.x - v[2].x;
		v1 = v[0].y - v[2].y;
		v2 = v[1].y - v[2].y;
		v3 = p.y - v[0].y;
		v4 = p.y - v[2].y;
	}
	// �����ĸ, ���ж��Ƿ�Ϸ�
	float denom = v1 * u2 - v2 * u1;
	if (denom == 0.0f)
	{
		// �˻�������, ���Ϊ���������
		return false;
	}
	// ������������
	float oneOverDenom = 1.0f / denom;
	b[0] = (v4 * u2 - v2 * u4) * oneOverDenom;
	b[1] = (v1 * u3 - v3 * u1) * oneOverDenom;
	b[2] = 1.0f - b[0] - b[1];
	return true;
}

// �ж϶�����Ƿ�Ϊ͹�����, ����������ƽ���
// ����:
// n ������Ŀ
// vl ָ�򶥵������ָ��
bool isConvex(int n, const Vector3 vl[])
{
	// �ͳ�ʼ��Ϊ��
	float angleSum = 0.0f;
	// �������������, ���Ƕ����
	for (int i = 0; i < n; ++i)
	{
		// ���������, ����С�ĵ�һ�������һ������, ��Ȼ�����Ż������
		Vector3 e1;
		if (i == 0)
		{
			e1 = vl[n - 1] - vl[i];
		}
		else
		{
			e1 = vl[i - 1] - vl[i];
		}
		Vector3 e2;
		if (i == n - 1)
		{
			e2 = vl[0] - vl[i];
		}
		else
		{
			e2 = vl[i + 1] - vl[i];
		}
		// ��׼����������
		e1.normalize();
		e2.normalize();
		float dot = e1 * e2;
		// �����С�Ľ�, ��"��ȫ"�����Ǻ���, ���ⷢ����ֵ���ȵ�����
		float theta = safeAcos(dot);
		// �Ӻ�
		angleSum += theta;
	}
	// �����ڽǺ�
	float convexAngleSum = (float)(n - 2) * kPi;
	// ���ڿ����жϰ�͹��, ����һЩ������ֵ���
	if (angleSum < convexAngleSum - (float)n * 0.0001f)
	{
		// �������
		return false;
	}
	// ͹�����, ��һ�����
	return true;
}

// ����һ�����ƽ��, �ж�����ƽ�����һ��
// ������ֵ:
// < 0 ��ʾ����ȫ�ڱ���
// > 0 ��ʾ����ȫ������
// 0 ��ʾ����ƽ��
int classifySpherePlane(const Vector3 &planeNormal, float planeD, const Vector3 &sphereCenter, float sphereRadius)
{
	// �������ĵ�ƽ��ľ���
	float d = planeNormal * sphereCenter - planeD;
	if (d >= sphereRadius) // ��ȫ������
	{
		return 1;
	}
	else if (d <= -sphereRadius) // ��ȫ�ڱ���
	{
		return -1;
	}
	else // ����ƽ��
	{
		return 0;
	}
}

float rayTriangleIntersect(const Vector3 &rayOrg, const Vector3 &rayDelta, const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, float minT)
{
	// ���û���ཻ�ͷ����������
	const float kNoIntersection = 1e30f;
	// ����˳ʱ��ı�����
	Vector3 e1 = p1 - p0;
	Vector3 e2 = p2 - p1;
	// ������淨����(����Ҫ����)
	Vector3 n = crossProduct(e1, e2);
	// ������б��, ��ʾ����������"����"�ĳ̶�
	float dot = n * rayDelta;
	// �������ƽ���������λ�δָ����������������
	// ע���⽫�ܾ��˻������κ�����, ����ı��뷽ʽ�ǳ�����, NAN ������ͨ�����(���漰�� NAN ʱ, �� dot >= 0.0f ʱ�ı��ֲ�ͬ)
	if (!(dot < 0.0f))
	{
		return kNoIntersection;
	}
	// ����ƽ�淽�̵� d ֵ, ���ұ�ʹ�ô� d ��ƽ�淽��
	// Ax + By + Cz = d
	float d = n * p0;
	// ����Ͱ��������ε�ƽ��Ĳ�������
	float t = d - n * rayOrg;
	// ��������ڶ���εı���? Is ray origin on the backside of the polygon? Again, we phrase the check so that NANs will fail
	if (!(t <= 0.0f))
	{
		return kNoIntersection;
	}
	// �����Ѿ��ҵ�(�����߲�δ����ƽ��)?
	// ��Ϊ dot < 0: t/dot > minT �� t < dot*minT ����ͬ��
	// (And then we invert it for NAN checking...)
	if (!(t >= dot * minT))
	{
		return kNoIntersection;
	}
	// �õ�, ���ߺ�ƽ���ཻ. ����ʵ�ʵĽ���
	t /= dot;
	assert(t >= 0.0f);
	assert(t <= minT);
	// ���� 3D ����
	Vector3 p = rayOrg + rayDelta * t;
	// �ҵ�����Ҫ����, ѡ��ͶӰƽ��
	float u0, u1, u2;
	float v0, v1, v2;
	if (fabs(n.x) > fabs(n.y))
	{
		if (fabs(n.x) > fabs(n.z))
		{
			u0 = p.y - p0.y;
			u1 = p1.y - p0.y;
			u2 = p2.y - p0.y;
			v0 = p.z - p0.z;
			v1 = p1.z - p0.z;
			v2 = p2.z - p0.z;
		}
		else
		{
			u0 = p.x - p0.x;
			u1 = p1.x - p0.x;
			u2 = p2.x - p0.x;
			v0 = p.y - p0.y;
			v1 = p1.y - p0.y;
			v2 = p2.y - p0.y;
		}
	}
	else
	{
		if (fabs(n.y) > fabs(n.z))
		{
			u0 = p.x - p0.x;
			u1 = p1.x - p0.x;
			u2 = p2.x - p0.x;
			v0 = p.z - p0.z;
			v1 = p1.z - p0.z;
			v2 = p2.z - p0.z;
		}
		else
		{
			u0 = p.x - p0.x;
			u1 = p1.x - p0.x;
			u2 = p2.x - p0.x;
			v0 = p.y - p0.y;
			v1 = p1.y - p0.y;
			v2 = p2.y - p0.y;
		}
	}
	// �����ĸ, �������Ч��
	float temp = u1 * v2 - v1 * u2;
	if (!(temp != 0.0f))
	{
		return kNoIntersection;
	}
	temp = 1.0f / temp;
	// ������������, ÿһ�������߽�����
	float alpha = (u0 * v2 - v0 * u2) * temp;
	if (!(alpha >= 0.0f))
	{
		return kNoIntersection;
	}
	float beta = (u1 * v0 - v1 * u0) * temp;
	if (!(beta >= 0.0f))
	{
		return kNoIntersection;
	}
	float gamma = 1.0f - alpha - beta;
	if (!(gamma >= 0.0f))
	{
		return kNoIntersection;
	}
	// ���ز�������
	return t;
}