#pragma once

class Vector3;
class EulerAngles;

class Quaternion
{
public:
	// 四元数的四个值, 通常是不需要直接处理它们的
	// 然而仍然把它们设置为 public, 这是为了不给某些操作(如文件I/O)带来不必要的复杂性
	float w, x, y, z;

	// 置为单位四元数
	void identity()
	{
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	// 构造执行旋转的四元数
	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3 &axis, float theta);

	// 构造执行物体-惯性旋转的四元数, 方位参数用欧拉角形式给出
	void setToRotateObjectToInertial(const EulerAngles &orientation);
	void setToRotateInertialToObject(const EulerAngles &orientation);

	// 叉乘
	Quaternion operator*(const Quaternion &a) const;

	// 赋值乘法, 这是符合 C++ 习惯的写法
	Quaternion &operator*=(const Quaternion &a);

	// 将四元数正则化
	void normalize();

	// 提取旋转角和旋转轴
	float getRotationAngle() const;
	Vector3 getRotationAxis() const;
};

// 全局 "单位" 四元数
extern const Quaternion kQuaternionIdentity;

// 四元数点乘
extern float dotProduct(const Quaternion &a, const Quaternion &b);

// 球面插值
extern Quaternion slerp(const Quaternion &p, const Quaternion &q, float t);

// 四元数共轭
extern Quaternion conjugate(const Quaternion &q);

// 四元数幂
extern Quaternion pow(const Quaternion &q, float exponent);