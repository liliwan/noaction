#include "mess_mode.h"
#include "SurfaceMeshPlugins.h"
#include "ModePluginDockWidget.h"

#include <QFileDialog>

//#include "Kinect/RgbdViewer.h"
#include "Geometry/Scene.h"
//#include "Action/ActionLearner.h"
//#include "Kinect/kinect_grabber.h"

mess_mode::mess_mode()
{
	m_widget = NULL;
	m_scene = NULL;
	//actionLabeler = NULL;
	//actionViewer = NULL;
	
	//rgbdViewer = new RgbdViewer(this);
	//actionLearner = new ActionLearner(this);

	//m_actionViewerWidgetCreated = false;
}

mess_mode::~mess_mode()
{
	//if (m_widget) delete m_widget;

}

void mess_mode::create()
{
	if (!m_widget)
	{
		ModePluginDockWidget * dockwidget = new ModePluginDockWidget("Scene mess", mainWindow());
		m_widget = new mess_widget(this);
		dockwidget->setWidget(m_widget);
		mainWindow()->addDockWidget(Qt::RightDockWidgetArea, dockwidget);

		//rgbdViewer->parentMainWin = mainWindow();
		
	}
}

void mess_mode::destory()
{

}

void mess_mode::decorate()
{
	if (m_scene)
	{
		m_scene->draw();		
	}

	//if (rgbdViewer->hasWidget())
	//{
	//	rgbdViewer->updateWidget();
	//}

	//if (actionLearner->hasJob())
	//{
	//	actionLearner->drawSkeleton();

	//	if (actionLearner->isShowSampledSkeletons() && m_actionViewerWidgetCreated)
	//	{
	//		actionLearner->drawSampledSkeletons(actionViewer->getSelectModelID(), actionViewer->getSelectActionID());
	//	}
	//}	
}

void mess_mode::loadScene()
{
	m_scene = new CScene();
	m_scene->loadScene(m_widget->loadScanSceneName());
	m_scene->setSceneDrawArea(drawArea());

	// set viewer
	setSceneBounds();

	//test();
}

void mess_mode::loadSceneList()
{
	m_sceneList.init(m_widget->loadSceneListName());

}

void mess_mode::setSceneBounds()
{
	if (!m_scene)
	{
		drawArea()->setSceneRadius(2);
		drawArea()->setSceneCenter(qglviewer::Vec(0, 0, 0));
		drawArea()->setSceneBoundingBox(qglviewer::Vec(-1, -1, -1), qglviewer::Vec(1, 1, 1));
		drawArea()->camera()->setPosition(qglviewer::Vec(-1, -3, 2));
		drawArea()->showEntireScene();
		drawArea()->updateGL();
		return;
	}

	else
	{
		Eigen::AlignedBox3d scene_bbox = m_scene->bbox();
		SurfaceMesh::Vector3 a = scene_bbox.min();
		SurfaceMesh::Vector3 b = scene_bbox.max();

		qglviewer::Vec vecA(a.x(), a.y(), a.z());
		qglviewer::Vec vecB(b.x(), b.y(), b.z());

		drawArea()->setSceneCenter((vecA + vecB) * 0.5);
		drawArea()->setSceneBoundingBox(vecA, vecB);

		drawArea()->camera()->setViewDirection(qglviewer::Vec(0,1,0));
		drawArea()->showEntireScene();
		drawArea()->updateGL();
	}
}

//void mess_mode::loadActionJob()
//{
//	m_scene = new CScene();
//	//actionLearner->init(rgbdViewer, m_scene, drawArea());
//	actionLearner->init(this);
//
//	if (actionLearner->loadJob(m_widget->loadActionJobName()))
//	{
//		connect(rgbdViewer->depthSensor, SIGNAL(updateFrame(bool)), this, SLOT(syncWithScan(bool)));
//
//		// set scene bounds after action learner load the scene
//		setSceneBounds();
//
//		actionLearner->alignSceneToScan();
//		actionLearner->setTrackingObj();
//	}
//}

//void mess_mode::syncWithScan(bool state)
//{
//	if (state)
//	{
//		actionLearner->syncWithScan();
//		drawArea()->updateGL();
//	}
//}

//void mess_mode::test()
//{
//	// Create Cloud Viewer
//	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
//
//	// Callback Function to be called when Updating Data
//	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> function =
//		[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
//		if (!viewer.wasStopped()){
//			viewer.showCloud(cloud);
//		}
//	};
//
//	//// Create KinectGrabber
//	//pcl::Grabber* grabber = new pcl::KinectGrabber();
//
//	//// Regist Callback Function
//	//grabber->registerCallback(function);
//
//	//// Start Retrieve Data
//	//grabber->start();
//
//	while (!viewer.wasStopped()){
//		// Input Key ( Exit ESC key )
//		if (GetKeyState(VK_ESCAPE) < 0){
//			break;
//		}
//	}
//
//	// Stop Retrieve Data
//	grabber->stop();
//
//}

//void mess_mode::openActionLabeler()
//{
//	if (!actionLearner)
//	{
//		actionLabeler = new ActionLabeler(rgbdViewer->depthSensor, m_scene);
//		actionLabeler->createWidget();
//
//		m_scene->setPickModelMode(true);
//	}
//}


//void mess_mode::openActionViewer()
//{
//	if (actionLearner != NULL && actionLearner->isFinishPredict())
//	{
//		actionViewer = new ActionViewer(actionLearner);
//		actionViewer->createWidget();
//
//		m_actionViewerWidgetCreated = true;
//	}
//
//	else
//	{
//		Simple_Message_Box("Please do prediction first");
//	}
//}


void mess_mode::setSceneShowModel(int state)
{
	if (m_scene)
	{
		m_scene->setShowModel((bool)state);
		drawArea()->updateGL();
	}
}

void mess_mode::setSceneShowOBB(int state)
{
	if (m_scene)
	{
		m_scene->setShowOBB((bool)state);
		drawArea()->updateGL();
	}
}

void mess_mode::setSceneShowRG(int state)
{
	if (m_scene)
	{
		m_scene->setShowRG((bool)state);
		drawArea()->updateGL();
	}
}

void mess_mode::setSceneShowModelName(int state)
{
	if (m_scene)
	{
		m_scene->setShowModelName((bool)state);
		drawArea()->updateGL();
	}
}

bool mess_mode::mousePressEvent(QMouseEvent* event)
{
	if (event->modifiers() & Qt::SHIFT && m_scene->isPickModelModeOn())
	{
		mouseClickPos = event->pos();

		m_scene->pickModelAt(mouseClickPos);
		
		//if (actionLabeler != NULL && actionLabeler->hasWidget())
		//{
		//	actionLabeler->updateSelectModelInWidget(m_scene->getSelectedModelIDs());
		//}

		return true;
	}

	return false;
}



void mess_mode::arrangeScene()
{
	m_scene->arrangeSceneByOrder(m_widget->loadArrangeName());
	m_scene->setSceneDrawArea(drawArea());

	// set viewer
	setSceneBounds();
}
