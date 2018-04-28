#pragma once

class Quaternion;
class Matrix4x3;
class RotationMatrix;

// 该类用于表示 heading-pitch-bank 欧拉角系统
class EulerAngles
{
public:
	// 用弧度保存三个角度
	float heading;
	float pitch;
	float bank;

	EulerAngles() {}

	EulerAngles(float h, float p, float b) : heading(h), pitch(p), bank(b) {}

	// 置零
	void identity()
	{
		heading = 0.0f;
		pitch = 0.0f;
		bank = 0.0f;
	}

	// 变换为"限制集"欧拉角
	void canonize();

	// 从四元数转换到欧拉角
	// 输入的四元数假设为物体-惯性或惯性-物体四元数, 如其名所示
	void fromObjectToInertialQuaternion(const Quaternion &q);
	void fromInertialToObjectQuaternion(const Quaternion &q);

	// 从矩阵转换到欧拉角
	// 输入矩阵假设为物体-世界或世界-物体转换矩阵
	// 平移部分被省略, 并且假设矩阵是正交的
	void fromObjectToWorldMatrix(const Matrix4x3 &m);
	void fromWorldToObjectMatrix(const Matrix4x3 &m);

	// 从旋转矩阵转换到欧拉角
	void fromRotationMatrix(const RotationMatrix &m);
};

// 全局的"单位"欧拉角
extern const EulerAngles kEulerAnglesIdentity;