#pragma once
#include "../../Math/mathlib.h"
#include "../../MISC/Utility.h"
#include <GL/glut.h>

class CModel;

class CSuppPlane
{
public:

	typedef std::map<CString, std::vector<Vector2>> ModelInfo;

	CSuppPlane(void);
	~CSuppPlane(void);

	CSuppPlane(std::vector<Vector3> &PointSet);
	CSuppPlane(std::vector<Vector3> &PointSet, std::vector<CModel*> SuppChildren, std::vector<Vector3> &SuppChildrenPos);
	CSuppPlane(Vector3 newCorners[4], ModelInfo &suppModels);
	CSuppPlane(Vector3 newCorners[4]);

	void Build(std::vector<Vector3> &PointSet);
	void Draw();

	Vector3* GetCorners() {return m_corners;};
	ModelInfo& GetSuppModels() {return m_SuppModels;};
	int GetSuppModelTypeNum() {m_SuppModels.size();};
	int GetSuppModelNum(); // same instances will appear, the number is different from m_SuppModels.size()
	Vector3 GetCenter() {return center;};
	Vector3 GetConrner(int i) {return m_corners[i];};
	FTP GetWidth() { return width;};
	FTP GetLength() {return length;};
	FTP GetZ() {return m_corners[0].z;};

private:
	double length;
	double width;
	Vector3 center;
	Vector3 normal;

	Vector3 m_corners[4];
	
	ModelInfo m_SuppModels;
	std::vector<Vector3> m_SuppPointSet;

};

