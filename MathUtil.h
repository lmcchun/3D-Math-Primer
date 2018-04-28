#pragma once

#include <math.h>

const float kPi = 3.14159265f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

// ͨ�����ʵ��� 2Pi �������Ƕ����� -Pi �� Pi ��������
extern float wrapPi(float theta);

// "��ȫ"�����Ǻ���

extern float safeAcos(float x);

// ����Ƕȵ� sin �� cos ֵ
// ��ĳЩƽ̨��, �����Ҫ������ֵ,ͬʱ����Ҫ�ȷֿ������
inline void sinCos(float *returnSin, float *returnCos, float theta)
{
	// Ϊ�˼�, ����ֻʹ�ñ�׼���Ǻ���
	// ע����ĳЩƽ̨�Ͽ������ø���һЩ
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

inline float distanceSquared(const Vector3 &a, const Vector3 &b)
{
	float xDistance = a.x - b.x;
	float yDistance = a.y - b.y;
	float zDistance = a.z - b.z;
	return xDistance * xDistance + yDistance * yDistance + zDistance * zDistance;
}

// ����㼯�����ƽ��
extern Vector3 computeBestFitNormal(const Vector3 v[], int n);

// ���� 3D ����������������
extern bool computeBarycentricCoords3d(const Vector3 v[3] /* �����ζ��� */, const Vector3 &p /* Ҫ����������ĵ� */, float b[3] /* ������������ */);

// 3D ���ýǶȺ��ж�͹�����
extern bool isConvex(int n, const Vector3 vl[]);

// �ж�����ƽ�����һ��
extern int classifySpherePlane(const Vector3 &planeNormal /* �������� */, float planeD /* p * planeNormal = planeD */, const Vector3 &sphereCenter /* ���� */, float sphereRadius /* ��뾶 */);

// �����������ε��ཻ�Լ��, �㷨���� Didier Badouel, Graphics Gess I, pp 390-393
extern float rayTriangleIntersect(const Vector3 &rayOrg /* ������� */, const Vector3 &rayDelta /* ���߳��Ⱥͷ��� */, const Vector3 &p0, const Vector3 &p1, const Vector3 &p2 /* �����ζ��� */, float minT /* ĿǰΪֹ����ĵ�,�� 1.0 ��ʼ */);