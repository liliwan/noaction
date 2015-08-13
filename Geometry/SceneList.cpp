
#include "SceneList.h"

#include <omp.h>
#include "Model.h"



CSceneList::CSceneList()
{
	m_iCurr = -1;
}

CSceneList::CSceneList(const CSceneList &SL)
{
	m_SceneL = SL.m_SceneL;
}

CSceneList::~CSceneList()
{
	for (unsigned int i=0; i<m_SceneL.size(); i++) {
		SAFE_DELETE(m_SceneL[i]);
	}
}

int CSceneList::init(const QString filename)
{
	QFile inFile(filename);
	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return -1;

	QFileInfo s(filename);
	m_path = s.absolutePath() + "/";

	int num;
	ifs >> num;

	QString sceneName;

	// read the rest of line
	ifs.readLine();

	for (int i = 0; i < num; i++)
	{
		sceneName = ifs.readLine();

		CScene *pS = new CScene();
		pS->loadScene(m_path + sceneName + "/" + sceneName + ".txt");
		m_SceneL.push_back(pS);
	}

	m_iCurr = 0;

	InitLayouter();
	return m_SceneL.size();

}

int CSceneList::Remove(int iID)
{
	if (iID == -1) {
		iID = m_iCurr;
	}
	if (iID == -1) {
		return -1;
	}
	m_iCurr = (iID+1)%m_SceneL.size();
	SAFE_DELETE(m_SceneL[iID]);
	m_SceneL.erase(m_SceneL.begin()+iID);
	if (m_SceneL.empty()) {
		m_iCurr = -1;
	}
	return 0;
}

int CSceneList::RemoveAll(void)
{
	for (unsigned int i=0; i<m_SceneL.size(); i++) {
		SAFE_DELETE(m_SceneL[i]);
	}
	m_SceneL.clear();
	m_iCurr = -1;
	return 0;
}

int CSceneList::Add(CScene *pM)
{
	if (pM == NULL) {
		return -1;
	}
	m_SceneL.push_back(pM);
	return m_SceneL.size() - 1;	// id
}

int CSceneList::Add(const QString filename)
{
	CScene *pS = new CScene();
	pS->loadScene(filename);

	m_SceneL.push_back(pS);
	return m_SceneL.size() - 1;	// id

}

void CSceneList::SetCurr(int i)
{
	m_iCurr = i;
}

int CSceneList::GetCurrId(void)
{
	return m_iCurr;
}

CScene* CSceneList::GetCurr(void)
{
	if (m_iCurr<0 || m_iCurr >= (int)m_SceneL.size()) {
		return NULL;
	}
	return m_SceneL[m_iCurr];
}

CScene* CSceneList::GetScene(int i)
{
	if (i<0 || i >= (int)m_SceneL.size()) {
		return NULL;
	}
	return m_SceneL[i];
}

//void CSceneList::UpdataModelList()
//{
//	m_ModelList.clear();
//	for (int i = 0; i < m_SceneL.size(); i++)
//	{
//		std::vector<CModel*> modelList = m_SceneL[i]->GetModelList();
//		m_ModelList.push_back(modelList);
//	}
//}

void CSceneList::InitLayouter()
{
	// UpdataModelList();
	m_Layouter = new CLayouter(this);
}