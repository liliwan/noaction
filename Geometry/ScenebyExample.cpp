#include "ScenebyExample.h"

#include <QFileDialog>
#include <QTextStream>
#include <iostream>
#include <fstream>

CScenebyExample::CScenebyExample()
{
}


CScenebyExample::~CScenebyExample()
{
}

void CScenebyExample::GenerateScene()
{
	QFile inFile(filename);
	QTextStream ifs(&inFile);

	if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) return;

	QFileInfo sceneFileInfo(inFile.fileName());
	m_sceneFileName = sceneFileInfo.baseName();
	m_sceneFilePath = sceneFileInfo.absolutePath() + "/";

	QString modelName;

	ifs >> m_modelNum;

	// read the rest of line
	ifs.readLine();

	for (int i = 0; i < m_modelNum; i++)
	{
		modelName = ifs.readLine();
		CModel *newModel = new CModel();
		newModel->loadModel(m_sceneFilePath + "/" + modelName + ".obj");
		newModel->setLabel(modelName);
		newModel->setID(i);
		m_modelList.push_back(newModel);
	}
}