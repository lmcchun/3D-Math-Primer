#pragma once

#include "Vector3.h"
#include "Matrix4x3.h"

class AABB3
{
public:
	Vector3 min; // 最小点
	Vector3 max; // 最大点
	Vector3 size() const { return max - min; }
	float xSize() const { return max.x - min.x; }
	float ySize() const { return max.y - min.y; }
	float zSize() const { return max.z - min.z; }
	Vector3 center() const { return (min + max) * 0.5f; }
	// 提取 8 个顶点中的一个, 参考 .cpp 文件中点的编号
	Vector3 corner(int i) const;
	// 矩形边界操作
	// "清空"矩形边界框
	void empty();
	// 向矩形边界框中添加点
	void add(const Vector3 &p);
	// 向矩形边界框中添加 AABB
	void add(const AABB3 &box);
	// 变换矩形边界框, 计算新的 AABB
	void setToTransformedBox(const AABB3 &box, const Matrix4x3 &m);
	// 包含/相交性测试
	// 返回 true, 如果矩形边界框为空
	bool isEmpty() const;
	// 返回 true, 如果矩形边界框包含该点
	bool contains(const Vector3 &p) const;
	// 返回矩形边界框上的最近点
	Vector3 closestPointTo(const Vector3 &p) const;
	// 返回 true, 如果和球相交
	bool intersectsSphere(const Vector3 &center, float radius) const;
	// 和参数射线的相交性测试, 如果不相交则返回值大于 1
	float rayIntersect(const Vector3 &rayOrg, const Vector3 &rayDelta, Vector3 *returnNormal = 0) const;
	// 矩形边界框在平面的哪一面
	int classifyPlane(const Vector3 &n, float d) const;
	// 和平面的动态相交性测试
	float intersectPlane(const Vector3 &n, float planeD, const Vector3 &dir) const;
};
// 检测两个 AABB 的相交性, 如果相交返回 true, 还可以返回相交部分的 AABB
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2, AABB3 *boxIntersect = 0);
// 返回运动 AABB 和静止 AABB 相交时的参数点, 如果不相交则返回值大于 1
float intersectMovingAABB(const AABB3 &stationaryBox, const AABB3 &movingBox, const Vector3 &d);