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
	// 确保索引合法
	assert(i >= 0);
	assert(i <= 7);
	// 返回点
	return Vector3((i & 1) ? max.x : min.x, (i & 2) ? max.y : min.y, (i & 4) ? max.z : min.z);
}

// 将值赋为极大/极小值以清空矩形边界框
void AABB3::empty()
{
	const float kBigNumber = 1e37f;
	min.x = min.y = min.z = kBigNumber;
	max.x = max.y = max.z = -kBigNumber;
}

// 向矩形边界框中加一个点
void AABB3::add(const Vector3 &p)
{
	// 必要的时候扩张矩形边界框以包含这个点
	if (p.x < min.x) min.x = p.x;
	if (p.x > max.x) max.x = p.x;
	if (p.y < min.y) min.y = p.y;
	if (p.y > max.y) max.y = p.y;
	if (p.z < min.z) min.z = p.z;
	if (p.z > max.z) max.z = p.z;
}

// 向矩形边界框中添加 AABB
void AABB3::add(const AABB3 &box)
{
	// 必要的时候扩张矩形边界框
	if (box.min.x < min.x) min.x = box.min.x;
	if (box.max.x > max.x) max.x = box.max.x;
	if (box.min.y < min.y) min.y = box.min.y;
	if (box.max.y > max.y) max.y = box.max.y;
	if (box.min.z < min.z) min.z = box.min.z;
	if (box.max.z > max.z) max.z = box.max.z;
}

// 变换矩形边界框并计算新的 AABB
// 记住, 这将得到一个至少和原 AABB 一样大的 AABB, 或大得多的 AABB
void AABB3::setToTransformedBox(const AABB3 &box, const Matrix4x3 &m)
{
	// 如果为空, 则返回
	if (box.isEmpty())
	{
		empty();
		return;
	}
	// 从平移部分开始
	min = max = getTranslation(m);
	// 依次检查矩阵的 9 个元素, 计算新的 AABB
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

// 如果为空则返回 true
bool AABB3::isEmpty() const
{
	// 检查是否在某个轴上反过来了
	return (min.x > max.x) || (min.y > max.y) || (min.z > max.z);
}

// 如果包含该点则返回 true
bool AABB3::contains(const Vector3 &p) const
{
	// 检查每个轴上是否存在重叠部分
	return (p.x >= min.x) && (p.x <= max.x) && (p.y >= min.y) && (p.y <= max.y) && (p.z >= min.z) && (p.z <= max.z);
}

// 返回 AABB 上的最近点
Vector3 AABB3::closestPointTo(const Vector3 &p) const
{
	// 在每一堆上将 p "推向"矩形边界框
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
	// 返回
	return r;
}

// 和球相交则返回 true, 使用 Arvo 的算法
bool AABB3::intersectsSphere(const Vector3 &center, float radius) const
{
	// 找到矩形边界框上离球心最近的点
	Vector3 closestPoint = closestPointTo(center);
	// 检查最近点和球心的距离是否小于半径
	return distanceSquared(center, closestPoint) < radius * radius;
}

// 和参数射线相交, 如果相交的话返回 0 到 1 之间的参数值, 否则返回大于 1 的值
// 算法来自于 Woo 的"快速射线与矩形边界框相交性检测"方法
float AABB3::rayIntersect(const Vector3 &rayOrg /* 射线起点 */, const Vector3 &rayDelta /* 射线长度和方向 */, Vector3 *returnNormal /* 可选的, 相交点 */) const 
{
	// 如果未相交则返回这个大数
	const float kNoIntersection = 1e30f;
	// 检查点在矩形边界框内的情况, 并计算到每个面的距离
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
	// 是否在矩形边界框内?
	if (inside)
	{
		if (returnNormal != NULL)
		{
			*returnNormal = -rayDelta;
			returnNormal->normalize();
		}
		return 0.0f;
	}
	// 选择最远的平面-发生相交的地方
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
	case 0: // 和 yz 平面相交
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
	case 1: // 和 xz 平面相交
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
	case 2: // 和 xy 平面相交
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
	// 返回交点的参数值
	return t;
}

// 静止 AABB 与平面的相交性检测
// 返回值:
// <0 矩形边界框完全在平面的背面
// >0 矩形边界框完全在平面的正面
// =0 矩形边界框和平面相交
int AABB3::classifyPlane(const Vector3 &n, float d) const
{
	// 检查法向量, 计算最小和最大 D 值, 即距离
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
	// 完全在平面的前面?
	if (minD >= d)
	{
		return 1;
	}
	if (maxD <= d)
	{
		return -1;
	}
	// 横跨平面
	return 0;
}

// 动态 AABB 与平面相交性检测
// n 为平面法向量(假设为标准化向量)
// planeD 是平面方程 p * n = d 中的 D 值
// dir 是 AABB 移动的方向
// 假设平面是静止的
// 返回交点的参数值-相交时 AABB 移动的距离, 如果未相交时返回一个大数
// 只探测和平面正面的相交
float AABB3::intersectPlane(const Vector3 &n, float planeD, const Vector3 &dir) const
{
	// 检测它们是否为正则化向量
	assert(fabs(n * n - 1.0f) < 0.01f);
	assert(fabs(dir * dir - 1.0f) < 0.01f);
	// 如果未相交时返回这个大数
	const float kNoIntersection = 1e30f;
	// 计算夹角, 确保我们是在向平面的正面移动
	float dot = n * dir;
	if (dot >= 0.0f)
	{
		return kNoIntersection;
	}
	// 检查法向量, 计算最小和最大 D 值, minD 是"跑在最前面的"顶点的 D 值
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
	// 检测是否已经全部在平面的另一面
	if (maxD <= planeD)
	{
		return kNoIntersection;
	}
	// 将最前顶点代入标准射线方程
	float t = (planeD - minD) / dot;
	// 已经穿过了
	if (t < 0.0f)
	{
		return 0.0f;
	}
	// 返回它, 如果结果 >1, 则未能及时到达平面, 这事需要调用者进行检查
	return t;
}

// 检测两个 AABB 是否相交, 如果相交则返回 true, 也可以返回相交部分的 AABB
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2, AABB3 *boxIntersect)
{
	// 判断是否有重叠
	if (box1.min.x > box2.max.x) return false;
	if (box1.max.x < box2.min.x) return false;
	if (box1.min.y > box2.max.y) return false;
	if (box1.max.y < box2.min.y) return false;
	if (box1.min.z > box2.max.z) return false;
	if (box1.max.z < box2.min.z) return false;
	// 有重叠, 计算重叠部分的 AABB, 如果需要的话
	if (boxIntersect != NULL)
	{
		boxIntersect->min.x = fmaxf(box1.min.x, box2.min.x);
		boxIntersect->max.x = fminf(box1.max.x, box2.max.x);
		boxIntersect->min.y = fmaxf(box1.min.y, box2.min.y);
		boxIntersect->max.y = fminf(box1.max.y, box2.max.y);
		boxIntersect->min.z = fmaxf(box1.min.z, box2.min.z);
		boxIntersect->max.z = fminf(box1.max.z, box2.max.z);
	}
	// 它们相交
	return true;
}

// XXX
static void swap(float &a, float &b)
{
	float temp = a;
	a = b;
	b = temp;
}

// 动态 AABB 的相交性测试, 如果返回 >1 则未相交
float intersectMovingAABB(const AABB3 &stationaryBox, const AABB3 &movingBox, const Vector3 &d)
{
	// 如果未相交返回这个大数
	const float kNoIntersection = 1e30f;
	// 初始化时间区间, 以包含需要考虑的全部时间段
	float tEnter = 0.0f;
	float tLeave = 1.0f;
	// 计算每一维上的重叠部分, 再将这个重叠部分和前面的重叠部分作相交
	// 如果有一维上重叠部分为零则返回(不会相交)
	// 每一维上都必须当心零重叠
	// 检查 x 轴
	if (d.x == 0.0f)
	{
		// x 轴上重叠部分为空
		if ((stationaryBox.min.x >= movingBox.max.x) || (stationaryBox.max.x <= movingBox.min.x))
		{
			// 不会相交
			return kNoIntersection;
		}
		// 无穷大的时间区间, 没有必要更新
	}
	else
	{
		// 只除一次
		float oneOverD = 1.0f / d.x;
		// 计算开始接触和脱离接触的时间
		float xEnter = (stationaryBox.min.x - movingBox.max.x) * oneOverD;
		float xLeave = (stationaryBox.max.x - movingBox.min.x) * oneOverD;
		// 检查顺序
		if (xEnter > xLeave)
		{
			swap(xEnter, xLeave);
		}
		// 更新区间
		if (xEnter > tEnter) tEnter = xEnter;
		if (xLeave < tLeave) tLeave = xLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}
	// 检查 y 轴
	if (d.y == 0.0f)
	{
		// y 轴上重叠部分为空
		if ((stationaryBox.min.y >= movingBox.max.y) || (stationaryBox.max.y <= movingBox.min.y))
		{
			// 不会相交
			return kNoIntersection;
		}
		// 无穷大的时间区间, 没有必要更新
	}
	else
	{
		// 只除一次
		float oneOverD = 1.0f / d.y;
		// 计算开始接触和脱离接触的时间
		float yEnter = (stationaryBox.min.y - movingBox.max.y) * oneOverD;
		float yLeave = (stationaryBox.max.y - movingBox.min.y) * oneOverD;
		// 检查顺序
		if (yEnter > yLeave)
		{
			swap(yEnter, yLeave);
		}
		// 更新区间
		if (yEnter > tEnter) tEnter = yEnter;
		if (yLeave < tLeave) tLeave = yLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}
	// 检查 z 轴
	if (d.z == 0.0f)
	{
		// z 轴上重叠部分为空
		if ((stationaryBox.min.z >= movingBox.max.z) || (stationaryBox.max.z <= movingBox.min.z))
		{
			// 不会相交
			return kNoIntersection;
		}
		// 无穷大的时间区间, 没有必要更新
	}
	else
	{
		// 只除一次
		float oneOverD = 1.0f / d.z;
		// 计算开始接触和脱离接触的时间
		float zEnter = (stationaryBox.min.z - movingBox.max.z) * oneOverD;
		float zLeave = (stationaryBox.max.z - movingBox.min.z) * oneOverD;
		// 检查顺序
		if (zEnter > zLeave)
		{
			swap(zEnter, zLeave);
		}
		// 更新区间
		if (zEnter > tEnter) tEnter = zEnter;
		if (zLeave < tLeave) tLeave = zLeave;
		if (tEnter > tLeave)
		{
			return kNoIntersection;
		}
	}
	// 好了, 有相交发生, 返回交点的参数值
	return tEnter;
}