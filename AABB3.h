#pragma once

#include "Vector3.h"
#include "Matrix4x3.h"

class AABB3
{
public:
	Vector3 min; // ��С��
	Vector3 max; // ����
	Vector3 size() const { return max - min; }
	float xSize() const { return max.x - min.x; }
	float ySize() const { return max.y - min.y; }
	float zSize() const { return max.z - min.z; }
	Vector3 center() const { return (min + max) * 0.5f; }
	// ��ȡ 8 �������е�һ��, �ο� .cpp �ļ��е�ı��
	Vector3 corner(int i) const;
	// ���α߽����
	// "���"���α߽��
	void empty();
	// ����α߽������ӵ�
	void add(const Vector3 &p);
	// ����α߽������� AABB
	void add(const AABB3 &box);
	// �任���α߽��, �����µ� AABB
	void setToTransformedBox(const AABB3 &box, const Matrix4x3 &m);
	// ����/�ཻ�Բ���
	// ���� true, ������α߽��Ϊ��
	bool isEmpty() const;
	// ���� true, ������α߽������õ�
	bool contains(const Vector3 &p) const;
	// ���ؾ��α߽���ϵ������
	Vector3 closestPointTo(const Vector3 &p) const;
	// ���� true, ��������ཻ
	bool intersectsSphere(const Vector3 &center, float radius) const;
	// �Ͳ������ߵ��ཻ�Բ���, ������ཻ�򷵻�ֵ���� 1
	float rayIntersect(const Vector3 &rayOrg, const Vector3 &rayDelta, Vector3 *returnNormal = 0) const;
	// ���α߽����ƽ�����һ��
	int classifyPlane(const Vector3 &n, float d) const;
	// ��ƽ��Ķ�̬�ཻ�Բ���
	float intersectPlane(const Vector3 &n, float planeD, const Vector3 &dir) const;
};
// ������� AABB ���ཻ��, ����ཻ���� true, �����Է����ཻ���ֵ� AABB
bool intersectAABBs(const AABB3 &box1, const AABB3 &box2, AABB3 *boxIntersect = 0);
// �����˶� AABB �;�ֹ AABB �ཻʱ�Ĳ�����, ������ཻ�򷵻�ֵ���� 1
float intersectMovingAABB(const AABB3 &stationaryBox, const AABB3 &movingBox, const Vector3 &d);