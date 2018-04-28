#pragma once

class Quaternion;
class Matrix4x3;
class RotationMatrix;

// �������ڱ�ʾ heading-pitch-bank ŷ����ϵͳ
class EulerAngles
{
public:
	// �û��ȱ��������Ƕ�
	float heading;
	float pitch;
	float bank;

	EulerAngles() {}

	EulerAngles(float h, float p, float b) : heading(h), pitch(p), bank(b) {}

	// ����
	void identity()
	{
		heading = 0.0f;
		pitch = 0.0f;
		bank = 0.0f;
	}

	// �任Ϊ"���Ƽ�"ŷ����
	void canonize();

	// ����Ԫ��ת����ŷ����
	// �������Ԫ������Ϊ����-���Ի����-������Ԫ��, ��������ʾ
	void fromObjectToInertialQuaternion(const Quaternion &q);
	void fromInertialToObjectQuaternion(const Quaternion &q);

	// �Ӿ���ת����ŷ����
	// ����������Ϊ����-���������-����ת������
	// ƽ�Ʋ��ֱ�ʡ��, ���Ҽ��������������
	void fromObjectToWorldMatrix(const Matrix4x3 &m);
	void fromWorldToObjectMatrix(const Matrix4x3 &m);

	// ����ת����ת����ŷ����
	void fromRotationMatrix(const RotationMatrix &m);
};

// ȫ�ֵ�"��λ"ŷ����
extern const EulerAngles kEulerAnglesIdentity;