#pragma once

#include <math.h>

const float kPi = 3.14159265f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

// 通过加适当的 2Pi 倍数将角度限制 -Pi 到 Pi 的区间内
extern float wrapPi(float theta);

// "安全"反三角函数

extern float safeAcos(float x);

// 计算角度的 sin 和 cos 值
// 在某些平台上, 如果需要这两个值,同时计算要比分开计算快
inline void sinCos(float *returnSin, float *returnCos, float theta)
{
	// 为了简单, 我们只使用标准三角函数
	// 注意在某些平台上可以做得更好一些
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

// 计算点集的最佳平面
extern Vector3 computeBestFitNormal(const Vector3 v[], int n);

// 计算 3D 中任意点的重心坐标
extern bool computeBarycentricCoords3d(const Vector3 v[3] /* 三角形顶点 */, const Vector3 &p /* 要求重心坐标的点 */, float b[3] /* 保存重心坐标 */);

// 3D 中用角度和判断凸多边形
extern bool isConvex(int n, const Vector3 vl[]);

// 判断球在平面的哪一边
extern int classifySpherePlane(const Vector3 &planeNormal /* 必须正则化 */, float planeD /* p * planeNormal = planeD */, const Vector3 &sphereCenter /* 球心 */, float sphereRadius /* 球半径 */);

// 射线与三角形的相交性检测, 算法来自 Didier Badouel, Graphics Gess I, pp 390-393
extern float rayTriangleIntersect(const Vector3 &rayOrg /* 射线起点 */, const Vector3 &rayDelta /* 射线长度和方向 */, const Vector3 &p0, const Vector3 &p1, const Vector3 &p2 /* 三角形顶点 */, float minT /* 目前为止最近的点,从 1.0 开始 */);