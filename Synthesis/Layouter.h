#pragma once

#include <qgl.h>

#include "../Math/mathlib.h"
//#include "../../MISC/Utility.h"
#include <map>

class CSceneList;
class CModel;


class CLayouter
{
public:
	CLayouter(void);
	~CLayouter(void);

	typedef std::map<QString, std::map<QString, std::vector<MathLib::Vector2>>> Observations;

	CLayouter(CSceneList *pSceneList);

	void DetectSuppPlanes();
	/*void OpenSuppPlanes();
	void SaveSuppPlanes();*/

	void FitGM();
	void SaveObservations(QString path);


private:
	CSceneList *m_pSceneList;
	// std::vector<std::vector<CModel*>> m_ModelList;
	Observations m_observations;
};

