#pragma once

#include "Scene.h"
//#include "Assets.h"
//#include "../../../GlobalFunc.h"
#include "OBB.h"
//#include "../MISC/Utility.h"
//#include "Kernel/ModelKernel.h"
//#include "Kernel/RootKernel.h"

#include "Synthesis/Layouter.h"

//class CUDGraph;
//class CGraphKernel;
class CLayouter;

struct SynScene;
struct SimpleModel;

class CSceneList
{

public:
	typedef std::vector<CScene*>		SceneList;

protected:
	SceneList				m_SceneL;	// polygonal mesh
	/*CAssets					m_Assets;*/
	int						m_iCurr;	// current index

	std::map<QString, int>	m_NLMap;	// pointer to global Name-Label map maintained in CSceneList

	QString					m_fileName;	// scene list file name
	QString					m_path;	// scene list path 

	std::vector<QString> m_sSceneNameList;
	std::vector<int> m_SceneSizeList;

	//std::vector<std::vector<CModel*>> m_ModelList;
	//std::vector<std::vector<int>> m_ModelClusters;


	////////////////////////////////////////////////////////////////////////// Synthesis
	CLayouter *m_Layouter;

public:
	// life cycle
	CSceneList();
	CSceneList(const CSceneList &SceneL);
	~CSceneList();

	int init(const QString filename);
	int Add(const QString filename);
	int Add(CScene *pM);

	int Remove(int iID=-1);
	int RemoveAll(void);
	int Reload(int iID);
	
	void SetCurr(int i);
	CScene* GetCurr(void);
	int GetCurrId(void);
	//void DrawCurr(void);
	//void Draw(int i);
	CScene* GetScene(int i);
	//std::vector<std::vector<CModel*>>& GetModelList() { return m_ModelList; };

	unsigned int Size() const { return m_SceneL.size(); }
	const SceneList& GetSceneList() const { return m_SceneL; }
	bool IsEmpty() const { return m_SceneL.empty(); }
	CSceneList& operator=(const CSceneList& e);

	//void UpdataModelList();
	void InitLayouter();
	
};
