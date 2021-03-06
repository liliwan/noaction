#include <QFileDialog>

#include "mess_widget.h"
#include "ui_mess_widget.h"

#include "mess_mode.h"
//#include "Kinect/RgbdViewer.h"
//#include "Action/ActionLearner.h"
//#include "Action/ActionLabeler.h"


mess_widget::mess_widget(mess_mode * m) :
    ui(new Ui::mess_widget)
{
    ui->setupUi(this);
	this->m_mode = m;

	connect(ui->load3DScanButton, SIGNAL(clicked()), m_mode, SLOT(loadScene()));

	connect(ui->isShowModelBox, SIGNAL(stateChanged(int)), m_mode, SLOT(setSceneShowModel(int)));
	connect(ui->isShowOBBBox, SIGNAL(stateChanged(int)), m_mode, SLOT(setSceneShowOBB(int)));
	connect(ui->isShowRGBox, SIGNAL(stateChanged(int)), m_mode, SLOT(setSceneShowRG(int)));
	connect(ui->isShowModelNameBox, SIGNAL(stateChanged(int)), m_mode, SLOT(setSceneShowModelName(int)));
	
	connect(ui->testButton, SIGNAL(clicked()), m_mode, SLOT(arrangeScene()));
	connect(ui->loadSceneListButton, SIGNAL(clicked()), m_mode, SLOT(loadSceneList()));


	//connect(ui->openRgbdViewerButton, SIGNAL(clicked()), m_mode->rgbdViewer, SLOT(createWidget()));

	//connect(ui->loadActionJobButton, SIGNAL(clicked()), m_mode, SLOT(loadActionJob()));
	//connect(ui->openActionLabelerButton, SIGNAL(clicked()), m_mode, SLOT(openActionLabeler()));
	//connect(ui->resetAlignButton, SIGNAL(clicked()), m_mode->actionLearner, SLOT(resetAlignView()));
	//connect(ui->startLearnButton, SIGNAL(clicked()), m_mode->actionLearner, SLOT(startLearning()));

	//connect(ui->startPredictButton, SIGNAL(clicked()), m_mode->actionLearner, SLOT(startPredicting()));
	connect(ui->openActionViewerButton, SIGNAL(clicked()), m_mode, SLOT(openActionViewer()));
}

mess_widget::~mess_widget()
{
    delete ui;
}

QString mess_widget::loadScanSceneName()
{
	QString filename = QFileDialog::getOpenFileName(0, tr("Load scene"),
		m_mode->mainWindow()->settings()->getString("lastUsedDirectory"),
		tr("Scene File (*.txt)"));

	if (filename.isEmpty()) return "";

	// Keep folder active
	QFileInfo fileInfo(filename);
	m_mode->mainWindow()->settings()->set("lastUsedDirectory", fileInfo.absolutePath());

	return filename;
}

QString mess_widget::loadSceneListName()
{
	QString filename = QFileDialog::getOpenFileName(0, tr("Load scene list"),
		m_mode->mainWindow()->settings()->getString("lastUsedDirectory"),
		tr("Scene List File (*.txt)"));

	if (filename.isEmpty()) return "";

	// Keep folder active
	QFileInfo fileInfo(filename);
	m_mode->mainWindow()->settings()->set("lastUsedDirectory", fileInfo.absolutePath());

	return filename;

}

QString mess_widget::loadArrangeName()
{
	QString filename = QFileDialog::getOpenFileName(0, tr("Load Arrangement"),
		m_mode->mainWindow()->settings()->getString("lastUsedDirectory"),
		tr("Arrangement File (*.txt)"));

	if (filename.isEmpty()) return "";

	// Keep folder active
	QFileInfo fileInfo(filename);
	m_mode->mainWindow()->settings()->set("lastUsedDirectory", fileInfo.absolutePath());

	return filename;
}

//QString mess_widget::loadActionJobName()
//{
//	QString filename = QFileDialog::getOpenFileName(0, tr("Load Action Learning Job"),
//		m_mode->mainWindow()->settings()->getString("lastUsedDirectory"),
//		tr("Action Learning File (*.job)"));
//
//	if (filename.isEmpty()) return "";
//
//	// Keep folder active
//	QFileInfo fileInfo(filename);
//	m_mode->mainWindow()->settings()->set("lastUsedDirectory", fileInfo.absolutePath());
//
//	return filename;
//}

