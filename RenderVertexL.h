#pragma once

#include "Vector3.h"

// 未变换, 有光照的顶点
struct RenderVertexL
{
	Vector3 p; // 坐标
	unsigned argb; // 漫反射
	unsigned spec; // 镜面反射
	float u, v; // 纹理映射坐标
};
