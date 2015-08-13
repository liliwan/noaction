#include "../../MISC/Utility.h"

struct SynScene{
	CString label;
	int modelNum;
	CString baseName;
	std::vector<SimpleModel> models;
	void OutputSceneFile(CString fileName);

} SynScene;

struct SimpleModel{
	CString name;
	CString label;
	Matrix4 transMat;

} SimpleModel;

void SynScene::OutputSceneFile(CString fileName)
{
	std::ofstream ofs(fileName);
	if (!ofs.is_open())
	{
		MSG_BOX_ERROR("Cannot open file %s", fileName);
	}
	else
	{
		ofs<<"StanfordSceneDatabase"<<std::endl;
		ofs<<"version 1.0"<<std::endl;

		ofs<<"modelCount "<<modelNum+1<<std::endl;

		int id = 0;
		ofs<<"newModel "<<id<<" "<<"room"<<std::endl;
		ofs<<"transform ";
		Matrix4 transMat = Matrix4::Identity_Matrix;
		for (int j=0;j<15;j++)
		{
			ofs<<0<<" ";
		}
		ofs<<transMat.M[15]<<std::endl;
		id++;

		for (int i=0; i < modelNum;i++)
		{
			ofs<<"newModel "<<id<<" "<<models[i].name<<std::endl;
			Matrix4 transMat = models[i].transMat;
			ofs<<"transform ";
			for (int j=0;j<15;j++)
			{
				ofs<<transMat.M[j]<<" ";
			}
			ofs<<transMat.M[15]<<std::endl;
			id++;
		}
	}

	ofs.close();
}