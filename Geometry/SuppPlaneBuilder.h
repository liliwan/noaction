#pragma once
#include "../Math/mathlib.h"
#include "SurfaceMeshHelper.h"
#include "../Utilities/utility.h"

class CModel;
class SuppPlane;
class CScene;

class SuppPlaneBuilder
{
public:
	enum BuildMethod{ AABB_PLANE, OBB_PLANE, CONVEX_HULL_PLANE };
	SuppPlaneBuilder();
	~SuppPlaneBuilder();

	SuppPlaneBuilder(CModel *m);
	//SuppPlaneBuilder(CModel *m, CScene *sc);
	void build(int method);

	void collectSuppPtsSet();
	void seperateSuppSoupByZlevel(const std::vector<Surface_mesh::Point> &pts);

	void draw();

	std::vector<SuppPlane*> getAllSuppPlanes() { return m_suppPlanes; };
	SuppPlane* getLargestAreaSuppPlane();
	SuppPlane* getSuppPlane(int id) { return m_suppPlanes[id]; };
	int getSuppPlaneNum() { return m_suppPlanes.size(); };
	
	// MathLib::Vector3 getLocationRandom(); // wll


private:
	CModel *m_model;
	CScene *m_scene;
	int m_method;

	// each inner vector corresponds to a support plane
	std::vector<std::vector<MathLib::Vector3>> m_SuppPtsSet;
	std::vector<double> m_zLevelVals;
	std::vector<SuppPlane*> m_suppPlanes;

	MathLib::Vector3 m_upRightVec;
};

