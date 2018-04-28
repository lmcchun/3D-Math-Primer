#pragma once

#include "Vector3.h"

class Matrix4x3;

// ��һ�ַǳ����ĸ�ʽ����������������,
// ʹ������༭�Ͳ�������ʵ��(δ����Ⱦ, ��ײ���, �����������Ż�)
//֧������ӳ������Ͷ��㷨����
class EditTriMesh
{
public:
	// Vertex �����ڱ��涥����Ϣ
	class Vertex
	{
	public:
		Vertex() { setDefaults(); }
		void setDefaults();
		// 3D ����
		Vector3 p;
		// ���㼶����ӳ������
		// ע����Щ����ֵ����ʧЧ, "����"�� UV ���걣������������
		// ������Ⱦ, ������Ҫ���㼶����������
		// ���������Ż�, ������Ҫ���Ӳ�ͬ UV ֵ�Ķ���
		float u, v;
		// ���㼶�ı��淨����, ͬ������ʧЧ
		Vector3 normal;
		// ���߱���, �ܷ���
		int mark;
	};
	// �� Tri ���ڱ�����������Ϣ
	class Tri
	{
	public:
		Tri() { setDefaults(); }
		void setDefaults();
		// �涥��
		struct Vert
		{
			int index; // �����б������
			float u, v; // ��������
		};
		Vert v[3];
		// ���淨����
		Vector3 normal;
		// ����������Ĳ���
		int part;
		// �����б�����
		int material;
		// ���߱���, ����
		int mark;
		// �ж��Ƿ�Ϊ"�˻�"������-ͬһ����ʹ�ó���һ��
		bool isDegenerate() const;
		// ���ض������� 0, 1, 2 �� -1, ���δʹ�øö���
		int findVertex(int vertexIndex) const;
	};
	// ���������Ϣ
	// ����ֻ����һ���򵥵�����������ӳ��
	// ���ʾ��������������ӵ���Ϣ
	class Material
	{
	public:
		Material() { setDefaults(); }
		void setDefaults();
		char diffuseTextureName[256];
		// ���߱���
		int mark;
	};
	// �����Ż���ѡ��
	class OptimationParameters
	{
	public:
		OptimationParameters() { setDefaults(); }
		void setDefaults();
		// �ж����������Ƿ��غϵ��ݲ�
		float coincidentVertexTolerance;
		// �����νǶ��ݲ�
		// ���������ĳһ�߱���ͬ������������, ����Щ�����η������ǶȺܴ�
		// ��ô�������ϵĶ��㲻�ܱ�����
		// �������Ǳ�������Ƕȵ� cos ֵ, ���������������������
		float cosOfEdgeAngelTolerance;
		void setEdgeAngleToleranceInDegrees(float degrees);
	};
	// ��׼�����
	EditTriMesh();
	EditTriMesh(const EditTriMesh &x);
	~EditTriMesh();
	// = ����������������
	EditTriMesh &operator=(const EditTriMesh &src);
	// ��ȡ��������
	int vertexCount() const { return vCount; }
	int triCount() const { return tCount; }
	int materialCount() const { return mCount; }
	Vertex &vertex(int vertexIndex);
	const Vertex &vertex(int vertexIndex) const;
	Tri &tri(int triIndex);
	const Tri &tri(int triIndex) const;
	Material &material(int materialIndex);
	const Material &material(int materialIndex) const;
	// �����������
	// ��������Ϊ��
	void empty();
	// �����б��С
	// ����б�����, �����ӵ�ֵ���к��ʵĳ�ֵ
	// ����б�����, ���������Ч�Լ��
	void setVertexCount(int vc);
	void setTriCount(int tc);
	void setMaterialCount(int mc);
	// ���������/����/����, �����¼���Ԫ�ص�����
	int addTri();
	int addTri(const Tri &t);
	int addVertex();
	int addVertex(const Vertex &v);
	int dupVertex(int srcVertexIndex);
	int addMaterial(const Material &m);
	// ͬʱ�������� mark ����
	void markAllVertices(int mark);
	void markAllTris(int mark);
	void markAllMaterials(int mark);
	// ɾ������
	void deleteVertex(int vertexIndex);
	void deleteTri(int triIndex);
	void deleteMaterial(int materialIndex);
	void deleteUnusedMaterials();
	void deleteMarkedTris(int mark);
	void deleteDegenerateTris();
	// �������������
	// ������������һ���µĶ����б�
	// ����ÿ������ֻ����һ��������
	// ͬʱɾ��δ�õĵ�
	void detachAllFaces();
	// �任���ж���
	void transformVertices(const Matrix4x3 &m);
	// ����
	// ���������εı��淨����
	void computeOneTriNormal(int triIndex);
	void computeOneTriNormal(Tri &t);
	void computeTriNormals();
	// ���㶥�㷨����, �Զ����������η�����
	void computeVertexNormals();
	// �Ż�
	// ����ʹ������������ж����б�, �ܹ���ǿ cache ������
	// ����ɾ��δ�õĶ���
	void optimizeVertexOrder(bool removeUnusedVertices);
	// ���ݲ���Ϊ����������, ���ڿ�����Ⱦ�ǳ���Ҫ
	void sortTrisByMaterial();
	// ���Ӷ���
	void weldVertices(const OptimationParameters &opt);
	// ȷ������ UV ֵ��ȷ, �����Ҫ���ܻḴ�ƶ���
	void copyUvsIntoVertices();
	// �������е��Ż�, Ϊ������Ⱦ׼����ģ��, ���й���, ����"�����"��Ⱦϵͳ��˵
	// ����/���� S3D ģ��
	bool importS3d(const char *filename, char *returnErrMsg);
	bool exportS3d(const char *filename, char *returnErrMsg);
	// ������
	void validityCheck();
	bool validityCheck(char *returnErrMsg);
private:
	// �����б�
	int vAlloc;
	int vCount;
	Vertex *vList;
	int tAlloc;
	int tCount;
	Tri *tList;
	int mCount;
	Material &mList;
	// ʵ��ϸ��
	void construct();
};