#pragma once

#include "Vector3.h"

// �任��, �й��ն���
struct RenderVertexTL
{
	Vector3 p; // ��Ļ��������
	float w; // 1/z
	unsigned argb; // ������
	unsigned spec; // ���淴��
	float u, v; // ����ӳ������
};
