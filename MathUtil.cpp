#include "stdafx.h"
#include "MathUtil.h"
#include <assert.h>

const Vector3 kZeroVector(0.0f, 0.0f, 0.0f);

// 通过加上适当的 2Pi 倍数, 将角度限制在 -Pi 到 Pi 的区间内
float wrapPi(float theta)
{
	theta += kPi;
	theta -= floor(theta * k1Over2Pi) * k2Pi;
	theta -= kPi;
	return theta;
}

// 和 acos(x) 相同, 但如果 x 超出范围将返回最为接近的有效值
// 返回值在 0 到 Pi 之间, 和 C 语言中的标准 acos() 函数相同
float safeAcos(float x)
{
	// 检查边界条件
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
		// 使用标准 C 函数
		return acos(x);
	}
}

// 计算点集的最佳平面
Vector3 computeBestFitNormal(const Vector3 v[], int n)
{
	// 和置零
	Vector3 result = kZeroVector;
	// 从最后一个顶点开始, 避免在循环中做 if 判断
	const Vector3 *p = &v[n - 1];
	// 迭代所有顶点
	for (int i = 0; i < n; ++i)
	{
		// 得到"当前"顶点
		const Vector3 *c = &v[i];
		// 边向量乘积相加
		result.x += (p->z + c->z) * (p->y - c->y);
		result.y += (p->x + c->x) * (p->z - c->z);
		result.z += (p->y + c->y) * (p->x - c->x);
		// 下一个顶点
		p = c;
	}
	// 正则化结果并返回
	result.normalize();
	return result;
}

// 计算 3D 中任意点的重心坐标
bool computeBarycentricCoords3d(const Vector3 v[3], const Vector3 &p, float b[3])
{
	// 首先, 计算两个边向量, 呈顺时针方向
	Vector3 d1 = v[1] - v[0];
	Vector3 d2 = v[2] - v[1];
	// 用叉乘计算法向量, 许多情况下, 这一步都可以省略, 因为法向量是预先计算的
	// 不需要正则化, 不管预先计算的法向量是否正则过
	Vector3 n = crossProduct(d1, d2);
	// 判断法向量中占优势的轴, 选择投影平面
	float u1, u2, u3, u4;
	float v1, v2, v3, v4;
	if ((fabs(n.x) >= fabs(n.y)) && (fabs(n.x) >= fabs(n.z)))
	{
		// 抛弃 x, 向 yz 平面投影
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
		// 抛弃 y, 向 xz 平面投影
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
	// 计算分母, 并判断是否合法
	float denom = v1 * u2 - v2 * u1;
	if (denom == 0.0f)
	{
		// 退化三角形, 面积为零的三角形
		return false;
	}
	// 计算重心坐标
	float oneOverDenom = 1.0f / denom;
	b[0] = (v4 * u2 - v2 * u4) * oneOverDenom;
	b[1] = (v1 * u3 - v3 * u1) * oneOverDenom;
	b[2] = 1.0f - b[0] - b[1];
	return true;
}

// 判断多边形是否为凸多边形, 假设多边形是平面的
// 输入:
// n 顶点数目
// vl 指向顶点数组的指针
bool isConvex(int n, const Vector3 vl[])
{
	// 和初始化为零
	float angleSum = 0.0f;
	// 遍历整个多边形, 将角度相加
	for (int i = 0; i < n; ++i)
	{
		// 计算边向量, 必须小心第一个和最后一个顶点, 当然还有优化的余地
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
		// 标准化并计算点乘
		e1.normalize();
		e2.normalize();
		float dot = e1 * e2;
		// 计算较小的角, 用"安全"反三角函数, 避免发生数值精度的问题
		float theta = safeAcos(dot);
		// 加和
		angleSum += theta;
	}
	// 计算内角和
	float convexAngleSum = (float)(n - 2) * kPi;
	// 现在可以判断凹凸性, 允许一些经验数值误差
	if (angleSum < convexAngleSum - (float)n * 0.0001f)
	{
		// 凹多边形
		return false;
	}
	// 凸多边形, 有一定误差
	return true;
}

// 给定一个球和平面, 判断球在平面的哪一边
// 若返回值:
// < 0 表示球完全在背面
// > 0 表示球完全在正面
// 0 表示球横跨平面
int classifySpherePlane(const Vector3 &planeNormal, float planeD, const Vector3 &sphereCenter, float sphereRadius)
{
	// 计算球心到平面的距离
	float d = planeNormal * sphereCenter - planeD;
	if (d >= sphereRadius) // 完全在正面
	{
		return 1;
	}
	else if (d <= -sphereRadius) // 完全在背面
	{
		return -1;
	}
	else // 球横跨平面
	{
		return 0;
	}
}

float rayTriangleIntersect(const Vector3 &rayOrg, const Vector3 &rayDelta, const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, float minT)
{
	// 如果没有相交就返回这个大数
	const float kNoIntersection = 1e30f;
	// 计算顺时针的边向量
	Vector3 e1 = p1 - p0;
	Vector3 e2 = p2 - p1;
	// 计算表面法向量(不需要正则化)
	Vector3 n = crossProduct(e1, e2);
	// 计算倾斜角, 表示靠近三角形"正面"的程度
	float dot = n * rayDelta;
	// 检查射线平行于三角形或未指向三角形正面的情况
	// 注意这将拒绝退化三角形和射线, 这里的编码方式非常特殊, NAN 将不能通过检测(当涉及到 NAN 时, 与 dot >= 0.0f 时的表现不同)
	if (!(dot < 0.0f))
	{
		return kNoIntersection;
	}
	// 计算平面方程的 d 值, 在右边使用带 d 的平面方程
	// Ax + By + Cz = d
	float d = n * p0;
	// 计算和包含三角形的平面的参数交点
	float t = d - n * rayOrg;
	// 射线起点在多边形的背面? Is ray origin on the backside of the polygon? Again, we phrase the check so that NANs will fail
	if (!(t <= 0.0f))
	{
		return kNoIntersection;
	}
	// 交点已经找到(或射线并未到达平面)?
	// 因为 dot < 0: t/dot > minT 与 t < dot*minT 是相同的
	// (And then we invert it for NAN checking...)
	if (!(t >= dot * minT))
	{
		return kNoIntersection;
	}
	// 好的, 射线和平面相交. 计算实际的交点
	t /= dot;
	assert(t >= 0.0f);
	assert(t <= minT);
	// 计算 3D 交点
	Vector3 p = rayOrg + rayDelta * t;
	// 找到最主要的轴, 选择投影平面
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
	// 计算分母, 检查其有效性
	float temp = u1 * v2 - v1 * u2;
	if (!(temp != 0.0f))
	{
		return kNoIntersection;
	}
	temp = 1.0f / temp;
	// 计算重心坐标, 每一步都检查边界条件
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
	// 返回参数交点
	return t;
}