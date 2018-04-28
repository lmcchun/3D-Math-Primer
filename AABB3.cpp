#include "stdafx.h"
#include "AABB3.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "Matrix4x3.h"

// AABB3::corner
// 0 => (0, 0, 0)
// 1 => (1, 0, 0)
// 2 => (0, 1, 0)
// 3 => (1, 1, 0)
// 4 => (0, 0, 1)
// 5 => (1, 0, 1)
// 6 => (0, 1, 1)
// 7 => (1, 1, 1)

Vector3 AABB3::corner(int i) const
{
	// ȷ�������Ϸ�
	assert(i >= 0);
	assert(i <= 7);
	// ���ص�
	return Vector3((i & 1) ? max.x : min.x, (i & 2) ? max.y : min.y, (i & 4) ? max.z : min.z);
}

// ��ֵ��Ϊ����/��Сֵ����վ��α߽��
void AABB3::empty()
{
	const float kBigNumber = 1e37f;
	min.x = min.y = min.z = kBigNumber;
	max.x = max.y = max.z = -kBigNumber;
}

// ����α߽���м�һ����
void AABB3::add(const Vector3 &p)
{
	// ��Ҫ��ʱ�����ž��α߽���԰��������
	if (p.x < min.x) min.x = p.x;
	if (p.x > max.x) max.x = p.x;
	if (p.y < min.y) min.y = p.y;
	if (p.y > max.y) max.y = p.y;
	if (p.z < min.z) min.z = p.z;
	if (p.z > max.z) max.z = p.z;
}

// ����α߽������� AABB
void AABB3::add(const AABB3 &box)
{
	// ��Ҫ��ʱ�����ž��α߽��
	if (box.min.x < min.x) min.x = box.min.x;
	if (box.max.x > max.x) max.x = box.max.x;
	if (box.min.y < min.y) min.y = box.min.y;
	if (box.max.y > max.y) max.y = box.max.y;
	if (box.min.z < min.z) min.z = box.min.z;
	if (box.max.z > max.z) max.z = box.max.z;
}

// �任���α߽�򲢼����µ� AABB
// ��ס, �⽫�õ�һ�����ٺ�ԭ AABB һ����� AABB, ���ö�� AABB
void AABB3::setToTransformedBox(const AABB3 &box, const Matrix4x3 &m)
{
	// ���Ϊ��, �򷵻�
	if (box.isEmpty())
	{
		empty();
		return;
	}
	// ��ƽ�Ʋ��ֿ�ʼ
	min = max = getTranslation(m);
	// ���μ������ 9 ��Ԫ��, �����µ� AABB
	if (m.m11 > 0.0f)
	{
		min.x += m.m11 * box.min.x;
		max.x += m.m11 * box.max.x;
	}
	else
	{
		min.x += m.m11 * box.max.x;
		max.x += m.m11 * box.min.x;
	}
	if (m.m12 > 0.0f)
	{
		min.y += m.m12 * box.min.y;
		max.y += m.m12 * box.max.y;
	}
	else
	{
		min.y += m.m12 * box.max.y;
		max.y += m.m12 * box.min.y;
	}
	if (m.m13 > 0.0f)
	{
		min.z += m.m13 * box.min.z;
		max.z += m.m13 * box.max.z;
	}
	else
	{
		min.z += m.m13 * box.max.z;
		max.z += m.m13 * box.min.z;
	}
	if (m.m21 > 0.0f)
	{
		min.x += m.m21 * box.min.x;
		max.x += m.m21 * box.max.x;
	}
	else
	{
		min.x += m.m21 * box.max.x;
		max.x += m.m21 * box.min.x;
	}
	if (m.m22 > 0.0f)
	{
		min.y += m.m22 * box.min.y;
		max.y += m.m22 * box.max.y;
	}
	else
	{
		min.y += m.m22 * box.max.y;
		max.y += m.m22 * box.min.y;
	}
	if (m.m23 > 0.0f)
	{
		min.z += m.m23 * box.min.z;
		max.z += m.m23 * box.max.z;
	}
	else
	{
		min.z += m.m23 * box.max.z;
		max.z += m.m23 * box.min.z;
	}
	if (m.m31 > 0.0f)
	{
		min.x += m.m31 * box.min.x;
		max.x += m.m31 * box.max.x;
	}
	else
	{
		min.x += m.m31 * box.max.x;
		max.x += m.m31 * box.min.x;
	}
	if (m.m32 > 0.0f)
	{
		min.y += m.m32 * box.min.y;
		max.y += m.m32 * box.max.y;
	}
	else
	{
		min.y += m.m32 * box.max.y;
		max.y += m.m32 * box.min.y;
	}
	if (m.m33 > 0.0f)
	{
		min.z += m.m33 * box.min.z;
		max.z += m.m33 * box.max.z;
	}
	else
	{
		min.z += m.m33 * box.max.z;
		max.z += m.m33 * box.min.z;
	}
}

// ���Ϊ���򷵻� true
bool AABB3::isEmpty() const
{
	// ����Ƿ���ĳ�����Ϸ�������
	return (min.x > max.x) || (min.y > max.y) || (min.z > max.z);
}

// ��������õ��򷵻� true
bool AABB3::contains(const Vector3 &p) const
{
	// ���ÿ�������Ƿ�����ص�����
	return (p.x >= min.x) && (p.x <= max.x) && (p.y >= min.y) && (p.y <= max.y) && (p.z >= min.z) && (p.z <= max.z);
}

// ���� AABB �ϵ������
Vector3 AABB3::closestPointTo(const Vector3 &p) const
{
	// ��ÿһ���Ͻ� p "����"���α߽��
	Vector3 r;
	if (p.x < min.x)
	{
		r.x = min.x;
	}
	else if (p.x > max.x)
	{
		r.x = max.x;
	}
	else
	{
		r.x = p.x;
	}
	if (p.y < min.y)
	{
		r.y = min.y;
	}
	else if (p.y > max.y)
	{
		r.y = max.y;
	}
	else
	{
		r.y = p.y;
	}
	if (p.z < min.z)
	{
		r.z = min.z;
	}
	else if (p.z > max.z)
	{
		r.z = max.z;
	}
	else
	{
		r.z = p.z;
	}
	// ����
	return r;
}

// �����ཻ�򷵻� true, ʹ�� Arvo ���㷨
bool AABB3::intersectsSphere(const Vector3 &center, float radius) const
{
	// �ҵ����α߽��������������ĵ�
	Vector3 closestPoint = closestPointTo(center);
	// ������������ĵľ����Ƿ�С�ڰ뾶
	return distanceSquared(center, closestPoint) < radius * radius;
}

// �Ͳ��������ཻ, ����ཻ�Ļ����� 0 �� 1 ֮��Ĳ���ֵ, ���򷵻ش��� 1 ��ֵ
// �㷨������ Woo ��"������������α߽���ཻ�Լ��"����
float AABB3::rayIntersect(const Vector3 &rayOrg /* ������� */, const Vector3 &rayDelta /* ���߳��Ⱥͷ��� */, Vector3 *returnNormal /* ��ѡ��, �ཻ�� */) const 
{
	// ���δ�ཻ�򷵻��������
	const float kNoIntersection = 1e30f;
	// �����ھ��α߽���ڵ����, �����㵽ÿ����ľ���
	bool inside = true;
	float xt, xn;
	if (rayOrg.x < min.x)
	{
		xt = min.x - rayOrg.x;
		if (xt > rayDelta.x)
		{
			return kNoIntersection;
		}
		xt /= rayDelta.x;
		inside = false;
		xn = -1.0f;
	}
	else if (rayOrg.x > max.x)
	{
		xt = max.x - rayOrg.x;
		if (xt < rayDelta.x)
		{
			return kNoIntersection;
		}
		xt /= rayDelta.x;
		inside = false;
		xn = 1.0f;
	}
	else
	{
		xt = -1.0f;
	}
	float yt, yn;
	if (rayOrg.y < min.y)
	{
		yt = min.y - rayOrg.y;
		if (yt > rayDelta.y)
		{
			return kNoIntersection;
		}
		yt /= rayDelta.y;
		inside = false;
		yn = -1.0f;
	}
	else if (rayOrg.y > max.y)
	{
		yt = max.y - rayOrg.y;
		if (yt < rayDelta.y)
		{
			return kNoIntersection;
		}
		yt /= rayDelta.y;
		inside = false;
		yn = 1.0f;
	}
	else
	{
		yt = -1.0f;
	}
	float zt, zn;
	if (rayOrg.z < min.z)
	{
		zt = min.z - rayOrg.z;
		if (zt > rayDelta.z)
		{
			return kNoIntersection;
		}
		zt /= rayDelta.z;
		inside = false;
		zn = -1.0f;
	}
	else if (rayOrg.z > max.z)
	{
		zt = max.z - rayOrg.z;
		if (zt < rayDelta.z)
		{
			return kNoIntersection;
		}
		zt /= rayDelta.z;
		inside = false;
		zn = 1.0f;
	}
	else
	{
		zt = -1.0f;
	}
	// �Ƿ��ھ��α߽����?
	if (inside)
	{
		if (returnNormal != NULL)
		{
			*returnNormal = -rayDelta;
			returnNormal->normalize();
		}
		return 0.0f;
	}
	// ѡ����Զ��ƽ��-�����ཻ�ĵط�
	int witch = 0;
	float t = xt;
	if (yt > t)
	{
		witch = 1;
		t = yt;
	}
	if (zt > t)
	{
		witch = 2;
		t = zt;
	}
	switch (witch)
	{
	case 0: // �� yz ƽ���ཻ
	{
		float y = rayOrg.y + rayDelta.y * t;
		if (y < min.y || y > max.y)
		{
			return kNoIntersection;
		}
		float z = rayOrg.z + rayDelta.z * t;
		if (z < min.z || z > max.z)
		{
			return kNoIntersection;
		}
		if (returnNormal != NULL)
		{
			returnNormal->x = xn;
			returnNormal->y = 0.0f;
			returnNormal->z = 0.0f;
		}
		break;
	}
	case 1: // �� xz ƽ���ཻ
	{
		float x = rayOrg.x + rayDelta.x * t;
		if (x < min.x || x > max.x)
		{
			return kNoIntersection;
		}
		float z = rayOrg.z + rayDelta.z * t;
		if (z < min.z || z > max.z)
		{
			return kNoIntersection;
		};
		if (returnNormal != NULL)
		{
			returnNormal->x = 0.0f;
			returnNormal->y = yn;
			returnNormal->z = 0.0f;
		}
		break;
	}
	case 2: // �� xy ƽ���ཻ
	{
		float x = rayOrg.x + rayDelta.x * t;
		if (x < min.x || x > max.x)
		{
			return kNoIntersection;
		}
		float y = rayOrg.y + rayDelta.y * t;
		if (y < min.y || y > max.y)
		{
			return kNoIntersection;
		}
		if (returnNormal != NULL)
		{
			returnNormal->x = 0.0f;
			returnNormal->y = 0.0f;
			returnNormal->z = zn;
		}
		break;
	}
	default:
		assert(false);
	}
	// ���ؽ���Ĳ���ֵ
	return t;
}

// ��ֹ AABB ��ƽ����ཻ�Լ��
// ����ֵ:
// <0 ���α߽����ȫ��ƽ��ı���
// >0 ���α߽����ȫ��ƽ�������
// =0 ���α߽���ƽ���ཻ
int AABB3::classifyPlane(const Vector3 &n, float d) const
{
	// ��鷨����, ������С����� D ֵ, ������
	float minD, maxD;
	if (n.x > 0.0f)
	{
		minD = n.x * min.x;
		maxD = n.x * max.x;
	}
	else
	{
		minD = n.x * max.x;
		maxD = n.x * min.x;
	}
	if (n.y > 0.0f)
	{
		minD += n.y * min.y;
		maxD += n.y * max.y;
	}
	else
	{
		minD += n.y * min.y;
		maxD += n.y * max.y;
	}
	if (n.z > 0.0f)
	{
		minD += n.z * min.z;
		maxD += n.z * max.z;
	}
	else
	{
		minD += n.z * max.z;
		maxD += n.z * min.z;
	}
	// ��ȫ��ƽ���ǰ��?
	if (minD >= d)
	{
		return 1;
	}
	if (maxD <= d)
	{
		return -1;
	}
	// ���ƽ��
	return 0;
}

// ��̬ AABB ��ƽ���ཻ�Լ��
// n Ϊƽ�淨����(����Ϊ��׼������)
// planeD ��ƽ�淽�� p * n = d �е� D ֵ
// dir �� AABB �ƶ��ķ���
// ����ƽ���Ǿ�ֹ��
// ���ؽ���Ĳ���ֵ-�ཻʱ AABB �ƶ��ľ���, ���δ�ཻʱ����һ������
// ֻ̽���ƽ��������ཻ
float AABB3::intersectPlane(const Vector3 &n, float planeD, const Vector3 &dir) const
{
	// ��������Ƿ�Ϊ��������
	assert(fabs(n * n - 1.0f) < 0.01f);
	assert(fabs(dir * dir - 1.0f) < 0.01f);
	// ���δ�ཻʱ�����������
	const float kNoIntersection = 1e30f;
	// ����н�, ȷ������������ƽ��������ƶ�
	float dot = n * dir;
	if (dot >= 0.0f)
	{
		return kNoIntersection;
	}
	// ��鷨����, ������С����� D ֵ, minD ��"������ǰ���"����� D ֵ
	float minD, maxD;
	if (n.x > 0.0f)
	{
		minD = n.x * min.x;
		maxD = n.x * max.x;
	}
	else
	{
		minD = n.x * max.x;
		maxD = n.x * min.x;
	}
	if (n.y > 0.0f)
	{
		minD += n.y * min.y;
		maxD += n.y * max.y;
	}
	else
	{
		minD += n.y * min.y;
		maxD += n.y * max.y;
	}
	if (n.z > 0.0f)
	{
		minD += n.z * min.z;
		maxD += n.z * max.z;
	}
	else
	{
		minD += n.z * max.z;
		maxD += n.z * min.z;
	}
	// ����Ƿ��Ѿ�ȫ����ƽ�����һ��
	if (maxD <= planeD)
	{
		return kNoIntersection;
	}
	// ����ǰ��������׼���߷���
	float t = (planeD - minD) / dot;
	// �Ѿ�������
	if (t < 0.0f)
	{
		return 0.0f;
	}
	// ������, ������ >1, ��δ�ܼ�ʱ����ƽ��, ������Ҫ�����߽��м��
	return t;
}

// ������� AABB �Ƿ��ཻ, ����ཻ�򷵻� true, Ҳ���Է����ཻ���ֵ� AABB
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2, AABB3 *boxIntersect)
{
	// �ж��Ƿ����ص�
	if (box1.min.x > box2.max.x) return false;
	if (box1.max.x < box2.min.x) return false;
	if (box1.min.y > box2.max.y) return false;
	if (box1.max.y < box2.min.y) return false;
	if (box1.min.z > box2.max.z) return false;
	if (box1.max.z < box2.min.z) return false;
	// ���ص�, �����ص����ֵ� AABB, �����Ҫ�Ļ�
	if (boxIntersect != NULL)
	{
		boxIntersect->min.x = fmaxf(box1.min.x, box2.min.x);
		boxIntersect->max.x = fminf(box1.max.x, box2.max.x);
		boxIntersect->min.y = fmaxf(box1.min.y, box2.min.y);
		boxIntersect->max.y = fminf(box1.max.y, box2.max.y);
		boxIntersect->min.z = fmaxf(box1.min.z, box2.min.z);
		boxIntersect->max.z = fminf(box1.max.z, box2.max.z);
	}
	// �����ཻ
	return true;
}

// XXX
static void swap(float &a, float &b)
{
	float temp = a;
	a = b;
	b = temp;
}

// ��̬ AABB ���ཻ�Բ���, ������� >1 ��δ�ཻ
float intersectMovingAABB(const AABB3 &stationaryBox, const AABB3 &movingBox, const Vector3 &d)
{
	// ���δ�ཻ�����������
	const float kNoIntersection = 1e30f;
	// ��ʼ��ʱ������, �԰�����Ҫ���ǵ�ȫ��ʱ���
	float tEnter = 0.0f;
	float tLeave = 1.0f;
	// ����ÿһά�ϵ��ص�����, �ٽ�����ص����ֺ�ǰ����ص��������ཻ
	// �����һά���ص�����Ϊ���򷵻�(�����ཻ)
	// ÿһά�϶����뵱�����ص�
	// ��� x ��
	if (d.x == 0.0f)
	{
		// x �����ص�����Ϊ��
		if ((stationaryBox.min.x >= movingBox.max.x) || (stationaryBox.max.x <= movingBox.min.x))
		{
			// �����ཻ
			return kNoIntersection;
		}
		// ������ʱ������, û�б�Ҫ����
	}
	else
	{
		// ֻ��һ��
		float oneOverD = 1.0f / d.x;
		// ���㿪ʼ�Ӵ�������Ӵ���ʱ��
		float xEnter = (stationaryBox.min.x - movingBox.max.x) * oneOverD;
		float xLeave = (stationaryBox.max.x - movingBox.min.x) * oneOverD;
		// ���˳��
		if (xEnter > xLeave)
		{
			swap(xEnter, xLeave);
		}
		// ��������
		if (xEnter > tEnter) tEnter = xEnter;
		if (xLeave < tLeave) tLeave = xLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}
	// ��� y ��
	if (d.y == 0.0f)
	{
		// y �����ص�����Ϊ��
		if ((stationaryBox.min.y >= movingBox.max.y) || (stationaryBox.max.y <= movingBox.min.y))
		{
			// �����ཻ
			return kNoIntersection;
		}
		// ������ʱ������, û�б�Ҫ����
	}
	else
	{
		// ֻ��һ��
		float oneOverD = 1.0f / d.y;
		// ���㿪ʼ�Ӵ�������Ӵ���ʱ��
		float yEnter = (stationaryBox.min.y - movingBox.max.y) * oneOverD;
		float yLeave = (stationaryBox.max.y - movingBox.min.y) * oneOverD;
		// ���˳��
		if (yEnter > yLeave)
		{
			swap(yEnter, yLeave);
		}
		// ��������
		if (yEnter > tEnter) tEnter = yEnter;
		if (yLeave < tLeave) tLeave = yLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}
	// ��� z ��
	if (d.z == 0.0f)
	{
		// z �����ص�����Ϊ��
		if ((stationaryBox.min.z >= movingBox.max.z) || (stationaryBox.max.z <= movingBox.min.z))
		{
			// �����ཻ
			return kNoIntersection;
		}
		// ������ʱ������, û�б�Ҫ����
	}
	else
	{
		// ֻ��һ��
		float oneOverD = 1.0f / d.z;
		// ���㿪ʼ�Ӵ�������Ӵ���ʱ��
		float zEnter = (stationaryBox.min.z - movingBox.max.z) * oneOverD;
		float zLeave = (stationaryBox.max.z - movingBox.min.z) * oneOverD;
		// ���˳��
		if (zEnter > zLeave)
		{
			swap(zEnter, zLeave);
		}
		// ��������
		if (zEnter > tEnter) tEnter = zEnter;
		if (zLeave < tLeave) tLeave = zLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}
	// ����, ���ཻ����, ���ؽ���Ĳ���ֵ
	return tEnter;
}