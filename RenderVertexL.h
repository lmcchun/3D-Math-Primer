#pragma once

#include "Vector3.h"

// δ�任, �й��յĶ���
struct RenderVertexL
{
	Vector3 p; // ����
	unsigned argb; // ������
	unsigned spec; // ���淴��
	float u, v; // ����ӳ������
};
