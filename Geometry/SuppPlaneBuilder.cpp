#include "SuppPlaneBuilder.h"
#include "CModel.h"
#include "SimplePointCloud.h"
#include "SuppPlane.h"
#include <QTime>

SuppPlaneBuilder::SuppPlaneBuilder()
{
}

SuppPlaneBuilder::SuppPlaneBuilder(CModel *m)
{
	m_model = m;
	m_scene = NULL;
	m_upRightVec = MathLib::Vector3(0, 0, 1);
}

//SuppPlaneBuilder::SuppPlaneBuilder(CModel *m, CScene *sc)
//{
//	m_model = m;
//	m_scene = sc;
//	m_upRightVec = MathLib::Vector3(0, 0, 1);
//}

SuppPlaneBuilder::~SuppPlaneBuilder()
{

}

void SuppPlaneBuilder::build(int method)
{
	m_method = method;
	collectSuppPtsSet();

	for (int i = 0; i < m_SuppPtsSet.size(); i++)
	{
		if (m_SuppPtsSet[i].size() > 0)
		{
			m_suppPlanes.push_back(new SuppPlane(m_SuppPtsSet[i], m_model->getOBBAxis()));

			SuppPlane* currPlane = m_suppPlanes[m_suppPlanes.size() - 1];
			currPlane->setModelID(m_model->getID()); // which model

			if (currPlane->isTooSmall())
			{
				m_suppPlanes.pop_back();
				delete currPlane;
			}
		}
	}

	// plane id w.r.t current supplane list
	for (int i = 0; i < m_suppPlanes.size(); i++)
	{
		m_suppPlanes[i]->setSuppPlaneID(i);
	}
}

// need to think: didn't consider model transform
void SuppPlaneBuilder::collectSuppPtsSet()
{
	SimplePointCloud& pts = m_model->getPointCloud();
	std::vector<Surface_mesh::Point> SupportPointSoup;

	if (pts.isEmpty())
	{
		m_model->extractToPointCloud();
		pts = m_model->getPointCloud();
	}

	Surface_mesh::Vertex_property<Surface_mesh::Vector3> vpoints = m_model->meshData()->vertex_property<Surface_mesh::Vector3>("v:point");
	Surface_mesh::Vertex_property<Surface_mesh::Vector3> vnormals = m_model->meshData()->vertex_property<Surface_mesh::Vector3>("v:normal");
	Surface_mesh::Vertex_iterator vit, vend = m_model->meshData()->vertices_end();

	
	int id = 0;
	for (vit = m_model->meshData()->vertices_begin(); vit != vend; vit++)
	{
		//// need to fix: perhaps use face normals
		//MathLib::Vector3 vn = MathLib::Vector3(vnormals[vit][0], vnormals[vit][1], vnormals[vit][2]);
		//if (MathLib::Acos(vn.dot(m_upRightVec)) < AngleThreshold)
		//{
		//	SupportPointSoup.push_back(pts[id]);
		//}

		if (m_model->isVertexUpRight(vit, AngleThreshold))
		{
			SupportPointSoup.push_back(pts[id]);
		}

		id++;
	}

	if (!SupportPointSoup.empty())
	{
		seperateSuppSoupByZlevel(SupportPointSoup);
	}
}

void SuppPlaneBuilder::seperateSuppSoupByZlevel(const std::vector<Surface_mesh::Point> &pts)
{
	// similar to nearest neighbor classifier
	std::vector<double> allZvals;
	double Gap_Threshold = 0.1;

	for (int i = 0; i < pts.size(); i++)
	{
		allZvals.push_back(pts[i][2]);
	}

	double zMax = *std::max_element(allZvals.begin(), allZvals.end());
	double zMin = *std::min_element(allZvals.begin(), allZvals.end());

	double zRange = zMax - zMin;
	//double zGap = Gap_Threshold*zRange;

	if (zRange != 0)
	{
		std::vector<double> tempZLevelVals(zRange / Gap_Threshold + 1, 0);
		std::vector<std::vector<MathLib::Vector3>> tempSuppPtsSet(tempZLevelVals.size());

		for (int i = 0; i < allZvals.size(); i++)
		{
			int zID = (allZvals[i] - zMin) / Gap_Threshold;

			tempZLevelVals[zID] = zMin + zID*Gap_Threshold;
			tempSuppPtsSet[zID].push_back(MathLib::Vector3(pts[i][0], pts[i][1], pts[i][2]));
		}

		for (int i = 0; i < tempZLevelVals.size(); i++)
		{
			if (tempZLevelVals[i] != 0)
			{
				m_zLevelVals.push_back(tempZLevelVals[i]);
			}

			if (tempSuppPtsSet[i].size() != 0)
			{
				m_SuppPtsSet.push_back(tempSuppPtsSet[i]);
			}
		}
	}
	else
	{
		std::vector<MathLib::Vector3> tempSuppPts;

		for (int i = 0; i < pts.size(); i++)
		{
			tempSuppPts.push_back(MathLib::Vector3(pts[i][0], pts[i][1], pts[i][2]));
		}

		m_SuppPtsSet.push_back(tempSuppPts);
	}
}

void SuppPlaneBuilder::draw()
{
	for (int i = 0; i < m_suppPlanes.size(); i++)
	{
		QColor c = GetColorFromSet(m_model->getID());
		m_suppPlanes[i]->Draw(c);
	}
}

SuppPlane* SuppPlaneBuilder::getLargestAreaSuppPlane()
{
	double maxArea = -1e6;
	int maxPlaneID = -1;

	for (int i = 0; i < m_suppPlanes.size(); i++)
	{
		double currArea = m_suppPlanes[i]->GetArea();
		if (currArea > maxArea)
		{
			maxArea = currArea;
			maxPlaneID = i;
		}
	}

	return m_suppPlanes[maxPlaneID];
}

//MathLib::Vector3 SuppPlaneBuilder::getLocationRandom()
//{
//	// random number
//	QTime time;
//	time = QTime::currentTime();
//	qsrand(time.msec() + time.second() * 1000);
//
//	int idx = qrand() % m_SuppPtsSet[m_SuppPtsSet.size() - 1].size();
//
//	MathLib::Vector3 loc;
//	loc = m_SuppPtsSet[m_SuppPtsSet.size() - 1][idx];
//	loc.z = m_suppPlanes[m_SuppPtsSet.size() - 1]->GetZ();
//	return loc;
//}
