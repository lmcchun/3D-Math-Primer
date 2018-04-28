#pragma once

#include "Vector3.h"

struct RenderVertex
{
	Vector3 p; // 坐标
	Vector3 n; // 法向量
	float u, v; // 纹理映射坐标
};