#include "Layouter.h"

#include "../Geometry/SceneList.h"
#include "../Model.h"
//#include "SuppPlane.h"


CLayouter::CLayouter(void)
{

}

CLayouter::CLayouter( CSceneList *pSceneList )
{
	m_pSceneList = pSceneList;
}


CLayouter::~CLayouter(void)
{

}

void CLayouter::DetectSuppPlanes()
{
	for (int i = 0; i < m_pSceneList->Size(); i++)
	{	
		CScene *pS = m_pSceneList->GetScene(i);
		pS->buildModelSuppPlane();
	}
	
}

void CLayouter::FitGM()
{
	// count the observation
	for (int i = 0; i < m_pSceneList->Size(); i++)
	{
		CScene* pS = m_pSceneList->GetScene(i);
		for (int j = 0; j < pS->getModelNum(); j++)
		{
			CModel* pM = pS->getModel(j);
			{	//std::vector<CSuppPlane*> suppPlanes = m_ModelList[i][j]->GetSuppPlanes();
				QString modelName = pM->label();

				for (int pi = 0; pi < pM->getSuppPlaneNum(); pi++)
				{
					SuppPlane::ModelInfo suppModels = pM->getSuppModels(pi);
					for (SuppPlane::ModelInfo::const_iterator it = suppModels.begin(); it!=suppModels.end();it++)
					{
						QString currSuppModelName = it->first;
						std::vector<Vector2> currSuppVec = it->second;
						for (int pos_i = 0; pos_i < currSuppVec.size(); pos_i++)
						{
							m_observations[modelName][currSuppModelName].push_back(currSuppVec[pos_i]);
						}
					}
				}
			}
		}
	}
}

//void CLayouter::OpenSuppPlanes()
//{
//	m_pSceneList->OpenSuppPlanes();
//}
//
//void CLayouter::SaveSuppPlanes()
//{
//	m_pSceneList->SaveSuppPlanes();
//}

void CLayouter::SaveObservations(QString path)
{
	QString layoutFolder = path + "LayoutData\\";

	for (Observations::const_iterator ob_it = m_observations.begin(); ob_it!=m_observations.end(); ob_it++)
	{
		QString modelName = ob_it->first;
		SuppPlane::ModelInfo suppModels = ob_it->second;

		QFile outFile(layoutFolder + modelName + ".sch"); //supported children
		QTextStream ofs(&outFile);
		
		std::vector<QString> suppModelNames;
		
		for (SuppPlane::ModelInfo::const_iterator sup_it = suppModels.begin(); sup_it != suppModels.end(); sup_it++)
		{
			// collect supported model names
			suppModelNames.push_back(sup_it->first);
			ofs << sup_it->first << "\n";
		}

		outFile.close();
		
		int id = 0;
		for (SuppPlane::ModelInfo::const_iterator sup_it = suppModels.begin(); sup_it != suppModels.end(); sup_it++)
		{
			QFile outFile2(layoutFolder + modelName + "_" + suppModelNames[id] + ".csv");
			QTextStream ofs_sup(&outFile2);

			std::vector<Vector2> dataPts = sup_it->second;
			for (int i=0; i< dataPts.size();i++)
			{
				ofs_sup<<dataPts[i].x<<","<<dataPts[i].y<<"\n";
			}

			outFile2.close();
			id++;
		}


	}
	


}
