#pragma once

#include "Vector3.h"

class Matrix4x3;

// 用一种非常灵活的格式保存索引三角网格,
// 使得网格编辑和操作容易实现(未对渲染, 碰撞检测, 或其他方面优化)
//支持纹理映射坐标和顶点法向量
class EditTriMesh
{
public:
	// Vertex 类用于保存顶点信息
	class Vertex
	{
	public:
		Vertex() { setDefaults(); }
		void setDefaults();
		// 3D 坐标
		Vector3 p;
		// 顶点级纹理映射坐标
		// 注意这些坐标值可能失效, "真正"的 UV 坐标保存在三角形中
		// 对于渲染, 经常需要顶点级的纹理坐标
		// 对于其他优化, 经常需要焊接不同 UV 值的顶点
		float u, v;
		// 顶点级的表面法向量, 同样可能失效
		Vector3 normal;
		// 工具变量, 很方便
		int mark;
	};
	// 类 Tri 用于保存三角形信息
	class Tri
	{
	public:
		Tri() { setDefaults(); }
		void setDefaults();
		// 面顶点
		struct Vert
		{
			int index; // 顶点列表的索引
			float u, v; // 纹理坐标
		};
		Vert v[3];
		// 表面法向量
		Vector3 normal;
		// 属于网格的哪部分
		int part;
		// 材质列表索引
		int material;
		// 工具变量, 方便
		int mark;
		// 判断是否为"退化"三角形-同一顶点使用超过一次
		bool isDegenerate() const;
		// 返回定点索引 0, 1, 2 或 -1, 如果未使用该顶点
		int findVertex(int vertexIndex) const;
	};
	// 保存材质信息
	// 这里只保存一个简单的漫反射纹理映射
	// 材质经常还有许多更复杂的信息
	class Material
	{
	public:
		Material() { setDefaults(); }
		void setDefaults();
		char diffuseTextureName[256];
		// 工具变量
		int mark;
	};
	// 控制优化的选项
	class OptimationParameters
	{
	public:
		OptimationParameters() { setDefaults(); }
		void setDefaults();
		// 判断两个顶点是否重合的容差
		float coincidentVertexTolerance;
		// 三角形角度容差
		// 如果三角形某一边被不同三角形所共享, 且这些三角形法向量角度很大
		// 那么这条边上的顶点不能被焊接
		// 所以我们保存这个角度的 cos 值, 并用下面这个函数设置它
		float cosOfEdgeAngelTolerance;
		void setEdgeAngleToleranceInDegrees(float degrees);
	};
	// 标准类操作
	EditTriMesh();
	EditTriMesh(const EditTriMesh &x);
	~EditTriMesh();
	// = 操作符用于网格复制
	EditTriMesh &operator=(const EditTriMesh &src);
	// 存取网格数据
	int vertexCount() const { return vCount; }
	int triCount() const { return tCount; }
	int materialCount() const { return mCount; }
	Vertex &vertex(int vertexIndex);
	const Vertex &vertex(int vertexIndex) const;
	Tri &tri(int triIndex);
	const Tri &tri(int triIndex) const;
	Material &material(int materialIndex);
	const Material &material(int materialIndex) const;
	// 基本网格操作
	// 重置网格为空
	void empty();
	// 设置列表大小
	// 如果列表增长, 新增加的值会有合适的初值
	// 如果列表缩减, 不会进行有效性检查
	void setVertexCount(int vc);
	void setTriCount(int tc);
	void setMaterialCount(int mc);
	// 添加三角形/顶点/材质, 返回新加入元素的索引
	int addTri();
	int addTri(const Tri &t);
	int addVertex();
	int addVertex(const Vertex &v);
	int dupVertex(int srcVertexIndex);
	int addMaterial(const Material &m);
	// 同时设置所有 mark 变量
	void markAllVertices(int mark);
	void markAllTris(int mark);
	void markAllMaterials(int mark);
	// 删除操作
	void deleteVertex(int vertexIndex);
	void deleteTri(int triIndex);
	void deleteMaterial(int materialIndex);
	void deleteUnusedMaterials();
	void deleteMarkedTris(int mark);
	void deleteDegenerateTris();
	// 逐个分离所有面
	// 这项操作会产生一个新的顶点列表
	// 其中每个顶点只用于一个三角形
	// 同时删除未用的点
	void detachAllFaces();
	// 变换所有顶点
	void transformVertices(const Matrix4x3 &m);
	// 计算
	// 计算三角形的表面法向量
	void computeOneTriNormal(int triIndex);
	void computeOneTriNormal(Tri &t);
	void computeTriNormals();
	// 计算顶点法向量, 自动计算三角形法向量
	void computeVertexNormals();
	// 优化
	// 根据使用情况重新排列顶点列表, 能够增强 cache 的性能
	// 还会删除未用的顶点
	void optimizeVertexOrder(bool removeUnusedVertices);
	// 根据材质为三角形排序, 对于快速渲染非常重要
	void sortTrisByMaterial();
	// 焊接顶点
	void weldVertices(const OptimationParameters &opt);
	// 确保顶点 UV 值正确, 如果需要可能会复制顶点
	void copyUvsIntoVertices();
	// 进行所有的优化, 为快速渲染准备好模型, 还有光照, 对于"大多数"渲染系统来说
	// 导入/导出 S3D 模型
	bool importS3d(const char *filename, char *returnErrMsg);
	bool exportS3d(const char *filename, char *returnErrMsg);
	// 调试用
	void validityCheck();
	bool validityCheck(char *returnErrMsg);
private:
	// 网格列表
	int vAlloc;
	int vCount;
	Vertex *vList;
	int tAlloc;
	int tCount;
	Tri *tList;
	int mCount;
	Material &mList;
	// 实现细节
	void construct();
};