#pragma once

#include "Vector3.h"

// 变换后, 有光照顶点
struct RenderVertexTL
{
	Vector3 p; // 屏幕坐标和深度
	float w; // 1/z
	unsigned argb; // 漫反射
	unsigned spec; // 镜面反射
	float u, v; // 纹理映射坐标
};
