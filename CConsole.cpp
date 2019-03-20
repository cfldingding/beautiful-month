#include "CDatabaseManager.h"
#include <QCoreApplication>
#include <QTranslator>
#include <QMessageBox>
#include <QDir>
#include "CConsole.h"
#include "CLogData.h"
#include "COLDLAB.h"
#include "CDlgShowStatus.h"
#include "CDlgShowData.h"
#include  "CDlgPLCBusy.h"
#include "CDlgPlcStatus.h"
#include "CDlgMoldNumMannul.h"
#include "CLogData.h"

CConsole::CConsole(QObject *parent)
	: QObject(parent)
{
	m_coldLabFrame = (COLDLAB *)parent;

	Initiate();

	qDebug() << "1: " << connect(m_pcTPLCComunication, SIGNAL(signals_cylinderArmGripper(bool)), this, SLOT(slots_cylinderArmGripper(bool)));
	qDebug() << "2: " << connect(m_pcTPLCComunication, SIGNAL(signals_motorCameraRotateRotating(bool)), this, SLOT(slots_motorCameraRotateRotating(bool)));
	qDebug() << "3: " << connect(m_pcTPLCComunication, SIGNAL(signals_hasBottle(bool)), this, SLOT(slots_hasBottle(bool)));
	qDebug() << "4: " << connect(m_pcTPLCComunication, SIGNAL(signals_outfeedConveyor(bool)), this, SLOT(slots_outfeedConveyor(bool)));
	qDebug() << "5: " << connect(m_pcTPLCComunication, SIGNAL(signals_trigCamera(bool, int)), this, SLOT(slots_trigCamera(bool, int)));
	qDebug() << "6: " << connect(m_pcTPLCComunication, SIGNAL(signals_trigThickness(bool)), this, SLOT(slots_trigThickness(bool)));
	qDebug() << "7: " << connect(m_pcTPLCComunication, SIGNAL(signals_trigInner(bool)), this, SLOT(slots_trigInner(bool)));
	qDebug() << "8: " << connect(m_pcTPLCComunication, SIGNAL(signals_trigPLCBusy(bool)), this, SLOT(slots_trigPLCBusy(bool)));

	//log
	qRegisterMetaType<EFileNameType>("EFileNameType");
	qDebug() << "9: " << connect(this, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "10: " << connect(m_pcTPLCComunication, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "11: " << connect(m_pcWeight, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "12: " << connect(m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "13: " << connect(m_pcThicknessProcessing->m_pReadThicknessThread, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "14: " << connect(m_pcThicknessProcessing->m_pCalculateThicknessThread, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "15: " << connect(m_pcInner, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	qDebug() << "16: " << connect(m_pcImageProcessing, SIGNAL(signals_addLogText(QString, EFileNameType)), this, SLOT(slots_addLogText(QString, EFileNameType)));
	
	//message box
	qDebug() << "17: " << connect(m_pcImageProcessing, SIGNAL(signals_showMessageBox(QString, QString)), this, SLOT(slots_showMessageBox(QString, QString)));
}
CConsole::~CConsole()
{
	if (m_pcWeight != NULL)
	{
		delete m_pcWeight;
		m_pcWeight = NULL;
	}
	if (m_pcImageProcessing != NULL)
	{
		delete m_pcImageProcessing;
		m_pcImageProcessing = NULL;
	}
	if (m_pcThicknessProcessing != NULL)
	{
		delete m_pcThicknessProcessing;
		m_pcThicknessProcessing = NULL;
	}
	if (m_pcInner != NULL)
	{
		delete m_pcInner;
		m_pcInner = NULL;
	}
	if (m_pcTInnerSimulation != NULL)
	{
		delete m_pcTInnerSimulation;
		m_pcTInnerSimulation = NULL;
	}
	if (m_pcIQScan != NULL)
	{
		delete m_pcIQScan;
		m_pcIQScan = NULL;
	}
	if (m_pcTPLCComunication != NULL)
	{
		delete m_pcTPLCComunication;
		m_pcTPLCComunication = NULL;
	}
	m_coldLabFrame = NULL;
}

void CConsole::Initiate()
{
	m_pcTPLCComunication = new CTPLCComunication(this);
	m_pcWeight = new CWeight(this);
	m_pcImageProcessing = new CImagePorcessing(this);
	m_pcThicknessProcessing = new CThickness(this);
	m_pcInner = new CTInner(this);
	m_pcTInnerSimulation = new CInnerDiaRoThread();
	m_pcIQScan = new CTIQScan(this);

	//initiate necessary opera parameters (current params, etc)
	m_sysOperaPara.strAppPath = QDir(QCoreApplication::applicationDirPath()).absolutePath();

	//read system parameters
	DatabaseManager::DALSystemConfigurationManager::ReadSysInfo(&m_sysConfigPara, m_pcImageProcessing->m_sCamInfo);
	//read current model name
	DatabaseManager::DALModelManager::ReadModel(m_sysConfigPara.strCurModelName, &m_pcTPLCComunication->m_sPLCModelPara, &m_sCheckModelPara);
	DatabaseManager::DALModelManager::ReadPlotRange(m_sysConfigPara.strCurModelName, &m_sPlotRange);
	//[yixue]add to initiate the calib data; the cam calib info is initiated in InitCamera
	DatabaseManager::DALCalibrationManager::ReadMotorCalib(&m_pcTPLCComunication->m_sMotorCalibPara);
	DatabaseManager::DALCalibrationManager::ReadPLCLimitPos(&m_pcTPLCComunication->m_sPLCSensorLimit);		//20160517，读取各工位极限位置，wxy
	DatabaseManager::DALCalibrationManager::ReadInDiaCalib(&m_pcInner->m_sInnerCalibPara);

	//read translation items
	DatabaseManager::DALTranslationManager::ReadItemName(&m_sysOperaPara.sItemName, &m_sysOperaPara.sDefaultName, m_sysConfigPara.nCurLang);

	//init thickness


	//iqscan
	m_pcIQScan->slots_resetTcpServer(m_sysConfigPara.nOperatingMode);
	m_pcIQScan->GetProductionLines();
}

void CConsole::InitCameras()
{
	m_pcImageProcessing->InitCameras();
}

void CConsole::InitCheck()
{
	m_pcImageProcessing->InitCheck();
}

void CConsole::InitSerialPort()
{
	bool bComPortChanged = false;
	//initiate plc com port
	m_pcTPLCComunication->InitSerialPort(m_sysConfigPara.nPLCSerialNo);
	if (m_sysConfigPara.nPLCSerialNo != m_pcTPLCComunication->m_nComPort) 
	{
		bComPortChanged = true;
		m_sysConfigPara.nPLCSerialNo = m_pcTPLCComunication->m_nComPort;
	}
	//initiate weight com port
	m_pcWeight->InitSerialPort(m_sysConfigPara.nScaleType, m_sysConfigPara.nWeightSerialNo);
	if (m_sysConfigPara.nWeightSerialNo != m_pcWeight->m_nComPort)
	{
		bComPortChanged = true;
		m_sysConfigPara.nWeightSerialNo = m_pcWeight->m_nComPort;
	}
	//initiate thickness com port
	m_pcThicknessProcessing->m_pReadThicknessThread->InitSerialPort(m_sysConfigPara.nThicknessSerailNo, m_sysConfigPara.nWeightSerialNo);
	if (m_sysConfigPara.nThicknessSerailNo != m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->m_nComPort)
	{
		bComPortChanged = true;
		m_sysConfigPara.nThicknessSerailNo = m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->m_nComPort;
	}
	//save the new com port if changed
	if (bComPortChanged) 
	{
		DatabaseManager::DALSystemConfigurationManager::SaveSysInfo(m_sysConfigPara);
	}
}

bool CConsole::InitThickness()
{
	if(m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->m_nComPort == -1) return false;
	m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->SetfRI(m_sCheckModelPara.dSRI);
	m_sysOperaPara.bCHRInitiated = m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->InitThicknessUnit();
	return m_sysOperaPara.bCHRInitiated;
}

bool CConsole::InitInner()
{
	// 	if (!sysConfigPara.bSimulateMode)
	// 	{
	// 		readInnerZero();
	// 	}
	// 	sDSParam.nImgHeight = 55;
	// 	sDSParam.nImgWidth = 55;
	// 	set_window_attr("border_width", 0);
	// 	set_check("~father");
	// 	int nHeight = ui.frameNJCheck->height();
	// 	int nWidth = ui.frameNJCheck->height();
	// 	open_window(0,0,ui.frameNJCheck->width(), ui.frameNJCheck->height(), (Hlong)(ui.frameNJCheck->winId()), "visible", "", &lCheckNJWndID);
	// 	set_check("father");
	// 	set_draw(lCheckNJWndID, "margin");
	// 	set_part(lCheckNJWndID, -sDSParam.nImgWidth, -sDSParam.nImgWidth, sDSParam.nImgWidth, sDSParam.nImgHeight);
	// 	set_font(lCheckNJWndID,"-Meiryp UI-25-*-*-*-*-1-ANSI_CHARSET-");//-FontName-Height-Width-Italic-Underlined-Strikeout-Bold-CharSet-
	// 	set_part_style(lCheckNJWndID, 0);//0:速度最快 1：图像缩放质量中等，耗时中等 2：高质量，耗时高
	// 	set_line_width(lCheckNJWndID,1);

	return true; //for Marposs sensor 
}

void CConsole::ReadMoldCode()
{
}

bool CConsole::JudgeWeight(s_JudgePara & sJudge, double dWeight)
{
	if (dWeight < sJudge.sWeightError.dValueL || dWeight > sJudge.sWeightError.dValueH)
	{
		return false;
	}
	return true;
}

bool CConsole::JudgeDimention(s_JudgePara & sJudge, s_ExtResultCamera & sExtCamera)
{
	if (sJudge.bCheckBody)
	{

		if(sJudge.bCheckFlatness && (sExtCamera.sFlatness.dValueL < sJudge.sFlatnessError.dValueL
		|| sExtCamera.sFlatness.dValueH > sJudge.sFlatnessError.dValueH))
		{
			return false;
		}
		if (sJudge.bCheckBodyDiaExt)
		{
			QVector<s_Range> vsExtValues = sExtCamera.GetExtValues(EInspectionType::ExtremeBodyDiameter);
			QVector<double> vdMaxOval = sJudge.vdBodyExtOvalMax;
			if (vsExtValues.size() != vdMaxOval.size())
			{
				emit signals_addLogText(tr("Incorrect values count for BodyDiaExt!"));
				return false;
			}
			for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
			{
				if (vsExtValues[idxPos].dValueL < sJudge.vsBodyDiaExtError[idxPos].dValueL
					|| vsExtValues[idxPos].dValueH > sJudge.vsBodyDiaExtError[idxPos].dValueH
					|| (vsExtValues[idxPos].dValueH - vsExtValues[idxPos].dValueL) > vdMaxOval[idxPos])
				{
					return false;
				}
			}
		}
		if (sJudge.bCheckBodyDia)
		{
			QVector<s_Range> vsExtValues = sExtCamera.GetExtValues(EInspectionType::BodyDiameter);
			QVector<double> vdMaxOval = sJudge.vdBodyOvalMax;
			if (vsExtValues.size() != vdMaxOval.size())
			{
				emit signals_addLogText(tr("Incorrect values count for BodyDia!"));
				return false;
			}
			for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
			{
				if (vsExtValues[idxPos].dValueL < sJudge.vsBodyDiaError[idxPos].dValueL
					|| vsExtValues[idxPos].dValueH > sJudge.vsBodyDiaError[idxPos].dValueH
					|| (vsExtValues[idxPos].dValueH - vsExtValues[idxPos].dValueL) > vdMaxOval[idxPos])
				{
					return false;
				}
			}
		}
	}
	
	if (sJudge.bCheckFinish)
	{
		if (sJudge.bCheckTotalHei && (sExtCamera.sTotalHeight.dValueL < sJudge.sTotalHeiError.dValueL
			|| sExtCamera.sTotalHeight.dValueH > sJudge.sTotalHeiError.dValueH))
		{
			return false;
		}
		if (sJudge.bCheckFinHoriAng && (sExtCamera.sFinHoriAng.dValueL < sJudge.sFinHoriAngError.dValueL
			|| sExtCamera.sFinHoriAng.dValueH > sJudge.sFinHoriAngError.dValueH))
		{
			return false;
		}
		if (sJudge.bCheckFinDiaExt)
		{
			QVector<s_Range> vsExtValues = sExtCamera.GetExtValues(EInspectionType::ExtremeFinishDiameter);
			for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
			{
				if (vsExtValues[idxPos].dValueL < sJudge.vsFinDiaExtError[idxPos].dValueL
					|| vsExtValues[idxPos].dValueH > sJudge.vsFinDiaExtError[idxPos].dValueH)
				{
					return false;
				}
			}
		}
		if (sJudge.bCheckFinDia)
		{
			QVector<s_Range> vsExtValues = sExtCamera.GetExtValues(EInspectionType::FinishDiameter);
			for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
			{
				if (vsExtValues[idxPos].dValueL < sJudge.vsFinDiaError[idxPos].dValueL
					|| vsExtValues[idxPos].dValueH > sJudge.vsFinDiaError[idxPos].dValueH)
				{
					return false;
				}
			}
		}
		if (sJudge.bCheckFinHei && (sExtCamera.sFinHeight.dValueL < sJudge.sFinHeiError.dValueL
			|| sExtCamera.sFinHeight.dValueH > sJudge.sFinHoriAngError.dValueH))
		{
			return false;
		}
		if (sJudge.bCheckFinPFHei && 
			(sExtCamera.sFinPFHeiLeft.dValueL < sJudge.sFinPFHeiError.dValueL
			|| sExtCamera.sFinPFHeiLeft.dValueH > sJudge.sFinPFHeiError.dValueH
			|| sExtCamera.sFinPFHeiRight.dValueL < sJudge.sFinPFHeiError.dValueL
			|| sExtCamera.sFinPFHeiRight.dValueH > sJudge.sFinPFHeiError.dValueH))
		{
			return false;
		}
		
		if (sJudge.bCheckScrewDia && (sExtCamera.sScrewDia.dValueL < sJudge.sScrewDiaError.dValueL
			|| sExtCamera.sScrewDia.dValueH > sJudge.sScrewDiaError.dValueH))
		{
			return false;
		}
		if (sJudge.bCheckScrewSpace && (sExtCamera.sScrewSpace.dValueL < sJudge.sScrewDiaError.dValueL
			|| sExtCamera.sScrewSpace.dValueH > sJudge.sScrewDiaError.dValueH))
		{
			return false;
		}
		if (sJudge.bCheckScrewHei && (sExtCamera.sScrewSpace.dValueL < sJudge.sScrewSpaceError.dValueL
			|| sExtCamera.sFinHoriAng.dValueH > sJudge.sScrewSpaceError.dValueH))
		{
			return false;
		}
	}
	if (sJudge.bCheckAxisOffset)
	{
		QVector<s_Range> vsExtValues = sExtCamera.GetExtValues(EInspectionType::AxisOffset);
		for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
		{
			if (vsExtValues[idxPos].dValueL < sJudge.vsAxisOffsetError[idxPos].dValueL
				|| vsExtValues[idxPos].dValueH > sJudge.vsAxisOffsetError[idxPos].dValueH)
			{
				return false;
			}
		}
	}
	return true;
}

bool CConsole::JudgeThickness(s_JudgePara & sJudge, s_ExtResultThickness & sExtThickness)
{
	if (sJudge.bCheckThickness)
	{
		QVector<s_Range> vsExtValues = sExtThickness.vsThickness;
		for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
		{
			if (vsExtValues[idxPos].dValueL < sJudge.vsThicknessError[idxPos].dValueL
				|| vsExtValues[idxPos].dValueH > sJudge.vsThicknessError[idxPos].dValueH)
			{
				return false;
			}
		}
	}
	return true;
}

bool CConsole::JudgeInnerDia(s_JudgePara & sJudge, s_ExtResultInner & sExtInner)
{
	if (sJudge.bCheckAxisOffset)
	{
		QVector<s_Range> vsExtValues = sExtInner.vsInnerDia;
		for (int idxPos = 0; idxPos < vsExtValues.size(); idxPos++)
		{
			if (vsExtValues[idxPos].dValueL < sJudge.vsInnerDiaError[idxPos].dValueL
				|| vsExtValues[idxPos].dValueH > sJudge.vsInnerDiaError[idxPos].dValueH)
			{
				return false;
			}
		}
	}
	return true;
}

bool CConsole::UpdateModel(QString strModelName, s_PLCAutoUpg *sPLCModel, s_CheckModel *sCheckModel, s_PlotRange *sPlotRange, s_MotorCalibPara *sMotorCalibPara, s_CameraCalibrationPara &sCamCalibPara1, s_CameraCalibrationPara &sCamCalibPara2, s_InnerDiaCalibPara &sInnerDiaCalibPara)
{
	if (!RecalculateModel(sPLCModel, sCheckModel, sMotorCalibPara, sCamCalibPara1.vsCalibPara[0], sCamCalibPara2.vsCalibPara[0], sInnerDiaCalibPara))
	{
		return false;
	}
	//save the model modifications during the recalculation in db
	sPLCModel->strBodyCameraCalibDate = sCamCalibPara1.strCalibDate;
	sPLCModel->strFinishCameraCalibDate = sCamCalibPara2.strCalibDate;
	sPLCModel->strInnerDiaCalibDate = sInnerDiaCalibPara.strCalibDate;
	DatabaseManager::DALModelManager::SaveModel(strModelName, *sPLCModel, *sCheckModel, MODEL_MODIFY);
	return true;
}

bool CConsole::RecalculateModel(
	s_PLCAutoUpg *sPLCModel, 
	s_CheckModel *sCheckModel,
	s_MotorCalibPara *sMotorCalibPara, 
	s_CamCalibPara &sCamCalibPara1, 
	s_CamCalibPara &sCamCalibPara2, 
	s_InnerDiaCalibPara &sInnerDiaCalibPara, 
	QString *strMsg)
{
	bool bSuccess = true;

	sPLCModel->ClearTrigFlags();
	if (sCheckModel->bCheckMoldReader)
	{
		if (!calculateMR(sPLCModel, *sMotorCalibPara))
		{
			bSuccess = false;
		}
	}
	sPLCModel->dThicknessWorkPos = sMotorCalibPara->dThicknessStandbyPos;
	if (!sPLCModel->bFastMode && sCheckModel->bCheckThickness)
	{
		if (!calculateThickness(sPLCModel, *sCheckModel, sMotorCalibPara))
		{
			bSuccess = false;
			sPLCModel->dThicknessWorkPos = sMotorCalibPara->dThicknessStandbyPos;
		}
	}
	if (!sCheckModel->bCheckBody)
	{
		sCheckModel->bCheckFlatness = false;
		sCheckModel->bCheckBodyDiaExt = false;
		sCheckModel->bCheckBodyDia = false;
	}
	if (!sCheckModel->bCheckFinish)
	{
		sCheckModel->bCheckTotalHeight = false;
		sCheckModel->bCheckFinHoriAng = false;
		sCheckModel->bCheckFinHei = false;
		sCheckModel->bCheckFinHei = false;
		sCheckModel->bCheckScrewDia = false;
		sCheckModel->bCheckScrewSpace = false;
		sCheckModel->bCheckScrewHei = false;
		sCheckModel->bCheckFinDiaExt = false;
		sCheckModel->bCheckFinDia = false;
	}
	if (sCheckModel->bCheckBody || sCheckModel->bCheckFinish 
		|| sCheckModel->bCheckAxisOffset || sCheckModel->bCheckThickness)
	{
		sPLCModel->bCheckCamera = true;
		if (sCheckModel->bCheckBody || sCheckModel->bCheckFinish || sCheckModel->bCheckAxisOffset)
		{
			QString strCalCamPosInfo;
			if (!calculateCameraPo(sPLCModel, sCheckModel, sCamCalibPara1, sCamCalibPara2, &strCalCamPosInfo))
			{
				bSuccess = false;
			}
			if (strMsg != NULL)
			{
				*strMsg = strCalCamPosInfo;
			}
		}
	}
	else
	{
		sPLCModel->bCheckCamera = false;
	}
	if (sCheckModel->bCheckInner)
	{
		if (!calculateInner(sPLCModel, sCheckModel, sInnerDiaCalibPara))
		{
			bSuccess = false;
		}
	}

	//sPLCModel->bFastMode = sCheckModel->bFastmode;
	sPLCModel->bCheckWeight = sCheckModel->bCheckWeight;
	sPLCModel->bCheckCamera = sCheckModel->bCheckBody || sCheckModel->bCheckFinish || sCheckModel->bCheckAxisOffset || sCheckModel->bCheckThickness;
	sPLCModel->bCheckInner = sCheckModel->bCheckInner;
	return bSuccess;
}

bool CConsole::calculateMR(s_PLCAutoUpg *sPLCModelUpg, s_MotorCalibPara &sMotorCalibPara)
{
	// 首位插入读模器
	sPLCModelUpg->dMotorCameraLifterPos[0] = sMotorCalibPara.dMRHei;
	sPLCModelUpg->nTrigFlag[0] = MR_TRIG;
	sPLCModelUpg->nMotorCameraLifterPosCount += 1;
	return true;
}
bool CConsole::calculateThickness(s_PLCAutoUpg *sPLCModelUpg, s_CheckModel &sCheckModel, s_MotorCalibPara *sMotorCalibPara)
{
	sPLCModelUpg->bCheckCamera = true;					//bCheckCamera代表视觉工位有效，并非代表要检测视觉
	//sMotorCalibPara->nThicknessSpeed = sCheckModel.nThicknessSpeed;//this is defined in motor calib dlg, not changed here

	sPLCModelUpg->dThicknessWorkPos = (120 - sCheckModel.dBottleDiameter) / 2 + sMotorCalibPara->dThicknessStandbyPos + 5;
	QVector<s_Inspection> vectInsp = sCheckModel.GetInspections(EInspectionType::Thickness);
	for (int idxPos = 0; idxPos<vectInsp.size(); idxPos++)
	{
		sPLCModelUpg->dMotorCameraLifterPos[sPLCModelUpg->nMotorCameraLifterPosCount + idxPos] = sMotorCalibPara->dThicknessHei - vectInsp[idxPos].sPosition.dValueL;
		sPLCModelUpg->nTrigFlag[sPLCModelUpg->nMotorCameraLifterPosCount + idxPos] = Thickness_TRIG;
	}
	sPLCModelUpg->nMotorCameraLifterPosCount += vectInsp.size();
	return true;
}
bool CConsole::calculateCameraPo(s_PLCAutoUpg *sPLCModel, s_CheckModel *sCheckModel, s_CamCalibPara &sCamCalibPara1, s_CamCalibPara &sCamCalibPara2, QString *strMsg)
{
	QString strMessageGetCamPos;
	s_ModelPosition sModelPosition = getCameraPoInfo(sCheckModel, sCamCalibPara1, sCamCalibPara2, &strMessageGetCamPos);
	if (!sModelPosition.bSuccess)
	{
		if (strMsg != NULL)
		{
			*strMsg = strMessageGetCamPos;
		}
		return false;
	}
	QString strMessageCalCamPos = tr("<font color=green>Finish!</font><br>");
	if (sModelPosition.dBodyL1 == 0 && sModelPosition.dBodyH1 == 0)//只检瓶口
	{
		sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount] = CAM2_TRIG;
		sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] = (sModelPosition.dFinCamH + sModelPosition.dFinCamL) / 2.f
			- (sModelPosition.dFinishH + sModelPosition.dFinishL) / 2.f;
		sCheckModel->dFinishCheckHeight = sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount];
		sPLCModel->nMotorCameraLifterPosCount++;
		strMessageCalCamPos += tr("VertMotor stays at %1mm\n,check span is %2mm,only check finish.")
			.arg(sPLCModel->dMotorCameraLifterPos[0]).arg(sModelPosition.dFinishH - sModelPosition.dFinishL);
	}
	else if (sModelPosition.dFinishL == 0 && sModelPosition.dFinishH == 0 || sCheckModel->bBodyCamOnly)//只检瓶身
	{
		if (1 == sModelPosition.nBodyCheckNo)
		{
			sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount] = CAM1_TRIG;
			sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] = (sModelPosition.dBodyCamH + sModelPosition.dBodyCamL) / 2.f
				- (sModelPosition.dBodyH1 + sModelPosition.dBodyL1) / 2.f;
			sCheckModel->SetBodyCheckHeight(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount]);
			sPLCModel->nMotorCameraLifterPosCount++;
			strMessageCalCamPos += tr("VertMotor stays at %1mm,check span is %2mm,only check body.")
				.arg(sPLCModel->dMotorCameraLifterPos[0]).arg(sModelPosition.dBodyH1 - sModelPosition.dBodyL1);
		}
		else if (2 == sModelPosition.nBodyCheckNo)
		{
			sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount] = CAM1_TRIG;
			sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] = (sModelPosition.dBodyCamH + sModelPosition.dBodyCamL) / 2.f
				- (sModelPosition.dBodyH1 + sModelPosition.dBodyL1) / 2.f;
			sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount + 1] = CAM1_TRIG;
			sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1] = (sModelPosition.dBodyCamH + sModelPosition.dBodyCamL) / 2.f
				- (sModelPosition.dBodyH2 + sModelPosition.dBodyL2) / 2.f;
			sCheckModel->SetBodyCheckHeight(
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount],
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1]);
			sPLCModel->nMotorCameraLifterPosCount += 2;
		}
		else
		{
			return false;
		}

	}
	else
	{
		if (1 == sModelPosition.nBodyCheckNo)
		{
#pragma region BodyCheck1Time
			//瓶身瓶口都检测，分两步计算。
			//1. 检测高度跨度不超时，如果检测间距跨度超过相机间距跨度，则移动一次，同时检
			if (((sModelPosition.dFinishH - sModelPosition.dBodyL1) < (sModelPosition.dFinCamH - sModelPosition.dBodyCamL)) && ((sModelPosition.dFinishL - sModelPosition.dBodyH1) > (sModelPosition.dFinCamL - sModelPosition.dBodyCamH)))
			{
				// 				sPLCModel->nMotorCameraLifterPosCount = 0;
				sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount] = CAM_ALL_TRIG;
				double dUpExt, dDownExt;
				double dHei1, dHei2;
				//计算可都拍到的最高高度
				dHei1 = sModelPosition.dFinCamH - sModelPosition.dFinishH;
				dHei2 = sModelPosition.dBodyCamH - sModelPosition.dBodyH1;
				dUpExt = min(dHei1, dHei2);
				//计算可都拍到的最低高度
				dHei1 = sModelPosition.dFinCamL - sModelPosition.dFinishL;
				dHei2 = sModelPosition.dBodyCamL - sModelPosition.dBodyL1;
				dDownExt = max(dHei1, dHei2);
				//最佳高度为二者之平均
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] = (dUpExt + dDownExt) / 2.f;
				strMessageCalCamPos += tr("VertMotor stays at` %1mm,body check span is %2mm, finish check span is %3mm\
												  ,check both body and finish.")\
												  .arg(sPLCModel->dMotorCameraLifterPos[0])\
												  .arg(sModelPosition.dBodyH1-sModelPosition.dBodyL1)\
												  .arg(sModelPosition.dFinishH-sModelPosition.dFinishL);
				sCheckModel->dFinishCheckHeight = sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount];
				sCheckModel->SetBodyCheckHeight(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount]);
				sPLCModel->nMotorCameraLifterPosCount++;
			}
			//2. 检测高度跨度超出两个相机高度跨度时，必至少移动两次，先检上，后检下
			else// if ((dFinishH-dBodyL) >= (dFinCamH-dBodyCamL))
			{
				//				sPLCModel->nMotorCameraLifterPosCount = 0;
				sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount] = CAM2_TRIG;
				sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount + 1] = CAM1_TRIG;
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] = (sModelPosition.dFinCamH + sModelPosition.dFinCamL) / 2.f
					- (sModelPosition.dFinishH + sModelPosition.dFinishL) / 2.f;

				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1] = (sModelPosition.dBodyCamH + sModelPosition.dBodyCamL) / 2.f
					- (sModelPosition.dBodyH1 + sModelPosition.dBodyL1) / 2.f;

				strMessageCalCamPos += tr("VertMotor stays at %1mm and %2mm,body check span is %3mm,finish check span is %4mm.<br>")
					.arg(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount])
					.arg(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1])\
					.arg(sModelPosition.dBodyH1 - sModelPosition.dBodyL1)\
					.arg(sModelPosition.dFinishH - sModelPosition.dFinishL);
				strMessageCalCamPos += tr("<font color=red>Note:</font>If adjust camera's position and increase space between 2 cameras\
						   at least %1mm may make the vertmotor stays just 1 position.").
					arg(sModelPosition.dFinishH - sModelPosition.dBodyL1 - (sModelPosition.dFinCamH - sModelPosition.dBodyCamL));

				sCheckModel->dFinishCheckHeight = sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount];
				sCheckModel->SetBodyCheckHeight(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1]);
				sPLCModel->nMotorCameraLifterPosCount += 2;
			}
#pragma endregion BodyCheck1Time
		}
		else if (2 == sModelPosition.nBodyCheckNo)
		{
#pragma region BodyCheck2Time
			if ((sModelPosition.dBodyH1 - sModelPosition.dBodyL1) < 2 * (sModelPosition.dBodyCamH - sModelPosition.dBodyCamL))
			{
				// 				sPLCModel->nMotorCameraLifterPosCount = 0;
				sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount] = CAM2_TRIG;
				sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount + 1] = CAM1_TRIG;
				sPLCModel->nTrigFlag[sPLCModel->nMotorCameraLifterPosCount + 2] = CAM1_TRIG;
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] = (sModelPosition.dFinCamH + sModelPosition.dFinCamL) / 2.f\
					- (sModelPosition.dFinishH + sModelPosition.dFinishL) / 2.f;
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1] = (sModelPosition.dBodyCamH + sModelPosition.dBodyCamL) / 2.f\
					- (sModelPosition.dBodyH1 + sModelPosition.dBodyL1) / 2.f;
				sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 2] = (sModelPosition.dBodyCamH + sModelPosition.dBodyCamL) / 2.f\
					- (sModelPosition.dBodyH2 + sModelPosition.dBodyL2) / 2.f;
				strMessageCalCamPos += tr("VertMotor stays at %1mm,%2mm and %3mm,body check span is %4mm,finish check span is %5mm.<br>")
					.arg(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount])\
					.arg(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] + 1)\
					.arg(sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount] + 2)\
					.arg(sModelPosition.dBodyH2 - sModelPosition.dBodyL1)\
					.arg(sModelPosition.dFinishH - sModelPosition.dFinishL);
				sCheckModel->dFinishCheckHeight = sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount];
				sCheckModel->SetBodyCheckHeight(
					sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 1],
					sPLCModel->dMotorCameraLifterPos[sPLCModel->nMotorCameraLifterPosCount + 2]);
				sPLCModel->nMotorCameraLifterPosCount += 3;
			}
#pragma endregion BodyCheck2Time
		}
	}
	if (strMsg != NULL)
	{
		*strMsg = strMessageCalCamPos;
	}
	return true;
}
s_ModelPosition CConsole::getCameraPoInfo(s_CheckModel *sCheckModel, s_CamCalibPara &sCamCalibPara1, s_CamCalibPara &sCamCalibPara2, QString *strErrMsg)
{
	s_ModelPosition sModelPosition;
	sModelPosition.bSuccess = false;
	QVector <double> vDataBody, vDataFinish;
	QString strError = tr("<font color=red>Error:</font><br>");

	sModelPosition.dBodyCamL = sCamCalibPara1.dHeightL;
	sModelPosition.dBodyCamH = sCamCalibPara1.dHeightH;
	sModelPosition.dFinCamL = sCamCalibPara2.dHeightL;
	sModelPosition.dFinCamH = sCamCalibPara2.dHeightH;
	if (sCheckModel->bCheckFinish || sCheckModel->bCheckAxisOffset)
	{
		getFinishParam(&vDataFinish, CAMERASPAN, *sCheckModel);
	}
	if (sCheckModel->bCheckBody || sCheckModel->bCheckAxisOffset)
	{
		getBodyParam(&vDataBody, *sCheckModel);
	}

	if (sCheckModel->bBodyCamOnly)
	{
		for (int i = 0; i<vDataFinish.size(); i++)
		{
			vDataBody.append(vDataFinish.at(i));
		}
		qSort(vDataBody.begin(), vDataBody.end());
		sModelPosition.dBodyL1 = vDataBody[0];
		sModelPosition.dBodyH1 = vDataBody[vDataBody.size() - 1];
		if ((sModelPosition.dBodyH1 - sModelPosition.dBodyL1) >= (sModelPosition.dBodyCamH - sModelPosition.dBodyCamL))
		{
			strError += tr("BodyCamOnly mode only working with bottle smaller than %1 mm \
											  Body check height range <font color=red>%2\
											  </font> exceed camera height range,please adjust the parameter.")
				.arg(sModelPosition.dBodyH1 - sModelPosition.dBodyL1 - 5)
				.arg(sModelPosition.dBodyH1 - sModelPosition.dBodyL1);
			return sModelPosition;
		}
	}

	//判断body参数是否合法
	if (vDataBody.size() == 0)
	{
		sModelPosition.nBodyCheckNo = 0;
		sModelPosition.dBodyL1 = sModelPosition.dBodyH1 = 0;
		sModelPosition.dBodyL2 = sModelPosition.dBodyH2 = 0;
	}
	else
	{
		qSort(vDataBody.begin(), vDataBody.end());
		sModelPosition.dBodyL1 = vDataBody[0];
		sModelPosition.dBodyH1 = vDataBody[vDataBody.size() - 1];

		if (sModelPosition.dBodyH1 > sCheckModel->dBottleHeight)
		{
			strError += tr("Body maximum check height <font color=red>%1\
											  </font> exceed bottle height,please adjust the parameter.")
				.arg(sModelPosition.dBodyH1);
			return sModelPosition;
		}
		if ((sModelPosition.dBodyH1 - sModelPosition.dBodyL1) >= 2 * (sModelPosition.dBodyCamH - sModelPosition.dBodyCamL - 2 * BODYCAMEDGE))
		{
			strError += tr("Body check height range <font color=red>%1\
											  </font> exceed detectable height range,please adjust the parameter.")
				.arg(sModelPosition.dBodyH1 - sModelPosition.dBodyL1);
			return sModelPosition;
		}
		else if ((sModelPosition.dBodyH1 - sModelPosition.dBodyL1) >= (sModelPosition.dBodyCamH - sModelPosition.dBodyCamL))
		{
			sModelPosition.nBodyCheckNo = 2;
			sModelPosition.dBodyL1 = vDataBody[0];
			sModelPosition.dBodyH1 = sModelPosition.dBodyL1
				+ (sModelPosition.dBodyCamH - sModelPosition.dBodyCamL)
				- BODYCAMEDGE;

			sModelPosition.dBodyH2 = vDataBody[vDataBody.size() - 1];
			sModelPosition.dBodyL2 = sModelPosition.dBodyH2
				- (sModelPosition.dBodyCamH - sModelPosition.dBodyCamL)
				+ BODYCAMEDGE;
		}
		else
		{
			sModelPosition.nBodyCheckNo = 1;
			sModelPosition.dBodyL1 = vDataBody[0];
			sModelPosition.dBodyH1 = vDataBody[vDataBody.size() - 1];
		}
	}
	//判断finish参数是否合法
	if (vDataFinish.size() == 0)
	{
		sModelPosition.dFinishL = sModelPosition.dFinishH = 0;
	}
	else
	{
		qSort(vDataFinish.begin(), vDataFinish.end());
		sModelPosition.dFinishL = vDataFinish[0];

		sModelPosition.dFinishH = vDataFinish[vDataFinish.size() - 1];
	}
	if (!sCheckModel->bBodyCamOnly)
	{
		if (sModelPosition.dFinishL < 0)
		{
			strError += tr("Finish maximum check height <font color=red>%1</font> exceed bottle height,please adjust the parameter.")
				.arg(sCheckModel->dBottleHeight - sModelPosition.dFinishL);
			return sModelPosition;
		}
		if ((sModelPosition.dFinishH - sModelPosition.dFinishL) >= (sModelPosition.dFinCamH - sModelPosition.dFinCamL))
		{
			strError += tr("Finish check height range <font color=red>%1</font> exceed camera height range,please adjust the parameter.")
				.arg(sModelPosition.dFinishH - sModelPosition.dFinishL);
			return sModelPosition;
		}
	}
	if (sModelPosition.dBodyL1 == 0 && sModelPosition.dBodyH1 == 0 && sModelPosition.dFinishL == 0 && sModelPosition.dFinishH == 0)//没勾选检测项
	{
		strError += tr("No valid check item selected!");
		return sModelPosition;
	}
	sModelPosition.bSuccess = true;
	sCheckModel->nBodyCheckNo = sModelPosition.nBodyCheckNo;

	if (strErrMsg != NULL)
	{
		*strErrMsg = strError;
	}
	return sModelPosition;
}
void CConsole::getBodyParam(QVector<double> *vData, s_CheckModel &sCheckModel)
{
	//判断body参数是否合法
	vData->clear();
	if (sCheckModel.bCheckBody)
	{
		if (sCheckModel.bCheckFlatness)
		{
			vData->append(getBodyInspectionPositions(EInspectionType::Flatness, sCheckModel));
		}

		if (sCheckModel.bCheckBodyDiaExt)
		{
			vData->append(getBodyInspectionPositions(EInspectionType::ExtremeBodyDiameter, sCheckModel));
		}
		if (sCheckModel.bCheckBodyDia)
		{
			vData->append(getBodyInspectionPositions(EInspectionType::BodyDiameter, sCheckModel));
		}
	}
	if (sCheckModel.bCheckAxisOffset)
	{
		QVector<s_Inspection> vectInsp = sCheckModel.GetInspections(EInspectionType::AxisOffset);
		for(int idx=0; idx<vectInsp.size();idx++)
		{
			vData->push_back(vectInsp[idx].sPosition.dValueL);
			vData->push_back(-2);
		}
	}
}
void CConsole::getFinishParam(QVector<double> *vData, int dSpan, s_CheckModel &sCheckModel)
{
	vData->clear();

	if (sCheckModel.bCheckFinish)
	{
		if (sCheckModel.bCheckTotalHeight)
		{
			vData->push_back(sCheckModel.dBottleHeight);
			vData->push_back(sCheckModel.dBottleHeight - dSpan);
		}
		if (sCheckModel.bCheckFinHoriAng)
		{
			vData->push_back(sCheckModel.dBottleHeight);
			vData->push_back(sCheckModel.dBottleHeight - dSpan);
		}
		if (sCheckModel.bCheckFinHei)
		{
			s_Inspection sInsp = sCheckModel.GetInspections(EInspectionType::FinishHeight)[0];
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueL);
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueH);
		}
		if (sCheckModel.bCheckScrewDia)
		{
			vData->push_back(sCheckModel.dBottleHeight);
			s_Inspection sInsp = sCheckModel.GetInspections(EInspectionType::ScrewDiameter)[0];
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueH);
		}
		if (sCheckModel.bCheckScrewSpace)
		{
			vData->push_back(sCheckModel.dBottleHeight);
			s_Inspection sInsp = sCheckModel.GetInspections(EInspectionType::ScrewSpace)[0];
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueH);
		}
		if (sCheckModel.bCheckScrewHei)
		{
			vData->push_back(sCheckModel.dBottleHeight);
			s_Inspection sInsp = sCheckModel.GetInspections(EInspectionType::ScrewHeight)[0];
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueH);
		}
		if (sCheckModel.bCheckFinDiaExt)
		{
			QVector<s_Inspection> vectInsp = sCheckModel.GetInspections(EInspectionType::ExtremeFinishDiameter);
			for (int idx = 0; idx<vectInsp.size(); idx++)
			{
				vData->push_back(sCheckModel.dBottleHeight - vectInsp[idx].sPosition.dValueL);
				vData->push_back(sCheckModel.dBottleHeight - vectInsp[idx].sPosition.dValueH);
			}
		}
		if (sCheckModel.bCheckFinDia)
		{
			QVector<s_Inspection> vectInsp = sCheckModel.GetInspections(EInspectionType::FinishDiameter);
			for (int idx = 0; idx<vectInsp.size(); idx++)
			{
				vData->push_back(sCheckModel.dBottleHeight - vectInsp[idx].sPosition.dValueL);
			}
		}
		if (sCheckModel.bCheckFinPFHei)
		{
			s_Inspection sInsp = sCheckModel.GetInspections(EInspectionType::FinishPFHeight)[0];
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueL);
			vData->push_back(sCheckModel.dBottleHeight - sInsp.sPosition.dValueH);
		}
	}

	if (sCheckModel.bCheckAxisOffset)
	{
		QVector<s_Inspection> vectInsp = sCheckModel.GetInspections(EInspectionType::AxisOffset);
		for (int idx = 0; idx<vectInsp.size(); idx++)
		{
			vData->push_back(sCheckModel.dBottleHeight - vectInsp[idx].sPosition.dValueH);
		}
	}
}
bool CConsole::calculateInner(s_PLCAutoUpg *sPLCModelUpg, s_CheckModel *sCheckModel, s_InnerDiaCalibPara &sInnerDiaCalibPara)
{
	sPLCModelUpg->bCheckInner = sCheckModel->bCheckInner;
	//sPLCModelUpg->nInnerRotateTriggerDelay = sCheckModel->nInnerRotateTriggerDelay;//[yixue]define directly in dlg motor calib
	sPLCModelUpg->nInnerLifterPerPo = sInnerDiaCalibPara.dInnerZOri2Base - sCheckModel->dBottleHeight - 10;
	if (sCheckModel->dInnerClampPosition == 0)
	{
		sCheckModel->dInnerClampPosition = 20;
	}

	sPLCModelUpg->nInnerClipWorkPos = sCheckModel->dBottleHeight - sInnerDiaCalibPara.dInnerCliper2Base - sCheckModel->dInnerClampPosition;

	QVector<s_Inspection> vectInsp = sCheckModel->GetInspections(EInspectionType::InnerDiameter);
	for (int idxPos = 0; idxPos < sPLCModelUpg->vdInnerLifterWorkPos.size(); idxPos++)
	{
		sPLCModelUpg->vdInnerLifterWorkPos[idxPos] = sInnerDiaCalibPara.dInnerZOri2Base - sCheckModel->dBottleHeight + vectInsp[idxPos].sPosition.dValueL;

	}

	return true;
}

QVector<double> CConsole::getBodyInspectionPositions(EInspectionType eInsp, s_CheckModel & sCheckModel)
{
	QVector<double> vData;
	QVector<s_Inspection> vectInsp = sCheckModel.GetInspections(eInsp);
	for (int idxPos = 0; idxPos < vectInsp.size(); idxPos++)
	{
		vData.push_back(vectInsp[idxPos].sPosition.dValueL);
		vData.push_back(vectInsp[idxPos].sPosition.dValueH);
	}
	return vData;
}

void CConsole::slots_showMessageBox(QString strType, QString str)
{
	QMessageBox::critical(NULL, strType, str);
}

void CConsole::SetPlotRange(QString strModelName, s_CheckModel &sCheckModel, s_PlotRange *sPlotRange)
{
	double dRange = 0.3;
	//Body
	sPlotRange->sFlatness = s_Range(0, 1);
	QList<s_Inspection> listBodyDiaExt = sCheckModel.mhashInspections.values(EInspectionType::ExtremeBodyDiameter);
	sPlotRange->vectBodyDiaExt.clear();
	for (int indexBodyDiaExt = 0; indexBodyDiaExt < listBodyDiaExt.size(); indexBodyDiaExt++)
	{
		sPlotRange->vectBodyDiaExt.append(s_Range(
			listBodyDiaExt[indexBodyDiaExt].sStandard.dValueL - dRange, 
			listBodyDiaExt[indexBodyDiaExt].sStandard.dValueH + dRange));
	}
	QList<s_Inspection> listBodyDia = sCheckModel.mhashInspections.values(EInspectionType::BodyDiameter);
	sPlotRange->vectBodyDia.clear();
	for (int indexBodyDia = 0; indexBodyDia < listBodyDia.size(); indexBodyDia++)
	{
		sPlotRange->vectBodyDia.append(s_Range(
			listBodyDia[indexBodyDia].sStandard.dValueL - dRange, 
			listBodyDia[indexBodyDia].sStandard.dValueH + dRange));
	}

	//Finish
	QList<s_Inspection> listTotalHei = sCheckModel.mhashInspections.values(EInspectionType::TotalHeight);
	if(listTotalHei.size() > 0)
		sPlotRange->sTotalHei = s_Range(listTotalHei[0].sStandard.dValueL - dRange, listTotalHei[0].sStandard.dValueH + dRange);
	QList<s_Inspection> listFinHoriAng = sCheckModel.mhashInspections.values(EInspectionType::FinishHorizontalAngle);
	if (listFinHoriAng.size() > 0)
		sPlotRange->sFinHoriAng = s_Range(listFinHoriAng[0].sStandard.dValueL - dRange, listFinHoriAng[0].sStandard.dValueH + dRange);
	QList<s_Inspection> listFinDiaExt = sCheckModel.mhashInspections.values(EInspectionType::ExtremeFinishDiameter);
	sPlotRange->vectFinDiaExt.clear();
	for (int indexFinDiaExt = 0; indexFinDiaExt < listFinDiaExt.size(); indexFinDiaExt++)
	{
		sPlotRange->vectFinDiaExt.append(s_Range(
			listFinDiaExt[indexFinDiaExt].sStandard.dValueL - dRange, 
			listFinDiaExt[indexFinDiaExt].sStandard.dValueH + dRange));
	}
	QList<s_Inspection> listFinDia = sCheckModel.mhashInspections.values(EInspectionType::FinishDiameter);
	sPlotRange->vectFinDia.clear();
	for (int indexFinDia = 0; indexFinDia < listFinDia.size(); indexFinDia++)
	{
		sPlotRange->vectFinDia.append(s_Range(
			listFinDia[indexFinDia].sStandard.dValueL - dRange, 
			listFinDia[indexFinDia].sStandard.dValueH + dRange));
	}
	QList<s_Inspection> listFinHei = sCheckModel.mhashInspections.values(EInspectionType::FinishHeight);
	if (listFinHei.size() > 0)
		sPlotRange->sFinHei = s_Range(listFinHei[0].sStandard.dValueL - dRange, listFinHei[0].sStandard.dValueH + dRange);
	QList<s_Inspection> listScrewDia = sCheckModel.mhashInspections.values(EInspectionType::ScrewDiameter);
	if (listScrewDia.size() > 0)
		sPlotRange->sScrewDia = s_Range(listScrewDia[0].sStandard.dValueL - dRange, listScrewDia[0].sStandard.dValueH + dRange);
	QList<s_Inspection> listScrewSpace = sCheckModel.mhashInspections.values(EInspectionType::ScrewSpace);
	if (listScrewSpace.size() > 0)
		sPlotRange->sScrewSpace = s_Range(listScrewSpace[0].sStandard.dValueL - dRange, listScrewSpace[0].sStandard.dValueH + dRange);
	QList<s_Inspection> listScrewHeight = sCheckModel.mhashInspections.values(EInspectionType::ScrewHeight);
	if (listScrewHeight.size() > 0)
		sPlotRange->sScrewHeight = s_Range(listScrewHeight[0].sStandard.dValueL - dRange, listScrewHeight[0].sStandard.dValueH + dRange);
	
	//Verticality
	QList<s_Inspection> listAxisOffset = sCheckModel.mhashInspections.values(EInspectionType::AxisOffset);
	sPlotRange->vectAxisOffset.clear();
	for (int indexAxisOffset = 0; indexAxisOffset < listAxisOffset.size(); indexAxisOffset++)
	{
		sPlotRange->vectAxisOffset.append(s_Range(
			listAxisOffset[indexAxisOffset].sStandard.dValueL - dRange,
			listAxisOffset[indexAxisOffset].sStandard.dValueH + dRange));
	}
	
	//Thickness
	QList<s_Inspection> listThickness = sCheckModel.mhashInspections.values(EInspectionType::Thickness);
	sPlotRange->vectThickness.clear();
	for (int indexThickness = 0; indexThickness < listThickness.size(); indexThickness++)
	{
		sPlotRange->vectThickness.append(s_Range(0, 3.5));
	}

	//Inner Diameter
	QList<s_Inspection> listInnerDia = sCheckModel.mhashInspections.values(EInspectionType::InnerDiameter);
	sPlotRange->vectInnerDia.clear();
	for (int indexInnerDia = 0; indexInnerDia < listInnerDia.size(); indexInnerDia++)
	{
		sPlotRange->vectInnerDia.append(s_Range(
			listInnerDia[indexInnerDia].sStandard.dValueL - dRange,
			listInnerDia[indexInnerDia].sStandard.dValueH + dRange));
	}
	
	//weight
	QList<s_Inspection> listWeight = sCheckModel.mhashInspections.values(EInspectionType::WeightInspection);
	if (listWeight.size() > 0)
	{
		sPlotRange->sWeight = s_Range(listWeight[0].sStandard.dValueL -3, listWeight[0].sStandard.dValueH + 3);
	}
	//Save to model
	DatabaseManager::DALModelManager::SavePlotRange(strModelName, *sPlotRange, MODEL_MODIFY);
}

QVector<s_Measurement> CConsole::GetMeasurements()
{
	QVector<s_Measurement> measurements;
	s_CheckModel sCheckModel = m_sCheckModelPara;
	//int bottleIndex = m_sysOperaPara.nTotalCount - 1;//nTotalCount modified after this function => or nBottleIdxCP 
	int bottleIndex = 9999;
	int nCountCZ = m_sDataOfBottles.vExtResultWeight.size();
	int nCountSJ = m_sDataOfBottles.vExtResultCamera.size();
	int nCountBH = m_sDataOfBottles.vExtResultThickness.size();
	int nCountNJ = m_sDataOfBottles.vExtResultInner.size();

	if(nCountCZ > 0 && nCountCZ < bottleIndex)
	{
		bottleIndex = nCountCZ;
	}
	if (nCountBH > 0 && nCountBH < bottleIndex)
	{
		bottleIndex = nCountBH;
	}
	if (nCountSJ > 0 && nCountSJ < bottleIndex)
	{
		bottleIndex = nCountSJ;
	}
	if (nCountNJ > 0 && nCountNJ < bottleIndex)
	{
		bottleIndex = nCountNJ;
	}
	if (bottleIndex == 9999) return measurements;
	
	bottleIndex -= 1;

	s_ExtResultCamera resultCamera;
	s_ExtResultThickness resultThickness;
	s_ExtResultInner resultInner;
	double resultWeight;

#pragma region Measurements Camera
	QList<s_Inspection> listFlatness, listBodyDiaExt, listBodyDia, listAxisOffset, listTotalHei, listFinHoriAng, listFinHei, listFinPFHei,
		listScrewDia, listScrewSpace, listScrewHei, listFinDiaExt, listFinDia;
	listFlatness = sCheckModel.mhashInspections.values(EInspectionType::Flatness);
	listBodyDiaExt = sCheckModel.mhashInspections.values(EInspectionType::ExtremeBodyDiameter);
	listBodyDia = sCheckModel.mhashInspections.values(EInspectionType::BodyDiameter);
	listAxisOffset = sCheckModel.mhashInspections.values(EInspectionType::AxisOffset);
	listTotalHei = sCheckModel.mhashInspections.values(EInspectionType::TotalHeight);
	listFinHoriAng = sCheckModel.mhashInspections.values(EInspectionType::FinishHorizontalAngle);
	listFinHei = sCheckModel.mhashInspections.values(EInspectionType::FinishHeight);
	listFinPFHei = sCheckModel.mhashInspections.values(EInspectionType::FinishPFHeight);
	listScrewDia = sCheckModel.mhashInspections.values(EInspectionType::ScrewDiameter);
	listScrewSpace = sCheckModel.mhashInspections.values(EInspectionType::ScrewSpace);
	listScrewHei = sCheckModel.mhashInspections.values(EInspectionType::ScrewHeight);
	listFinDiaExt = sCheckModel.mhashInspections.values(EInspectionType::ExtremeFinishDiameter);
	listFinDia = sCheckModel.mhashInspections.values(EInspectionType::FinishDiameter);
	if (listFlatness.size() != 0
		|| listBodyDiaExt.size() != 0
		|| listBodyDia.size() != 0
		|| listAxisOffset.size() != 0
		|| listTotalHei.size() != 0
		|| listFinHoriAng.size() != 0
		|| listFinHei.size() != 0
		|| listFinPFHei.size() != 0
		|| listScrewDia.size() != 0
		|| listScrewSpace.size() != 0
		|| listScrewHei.size() != 0
		|| listFinDiaExt.size() != 0
		|| listFinDia.size() != 0)
	{
		resultCamera = m_sDataOfBottles.vExtResultCamera[bottleIndex];
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::Flatness).size() != 0)
	{
		s_Measurement resSink = AssignMeasurement(
			QString("SI0x"),
			resultCamera.sFlatness.dValueL,
			0,
			listFlatness[0].sWarning,
			listFlatness[0].sStandard);
		measurements.push_back(resSink);

		s_Measurement resBulge = AssignMeasurement(
			QString("BU0x"),
			resultCamera.sFlatness.dValueH,
			0,
			listFlatness[0].sWarning,
			listFlatness[0].sStandard);
		measurements.push_back(resBulge);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::ExtremeBodyDiameter).size() != 0)
	{
		for (int indexMeasurement = 0; indexMeasurement<listBodyDiaExt.size(); indexMeasurement++)
		{
			s_Measurement resBodyDiaExtm = AssignMeasurement(
				QString("BE%1m").arg(indexMeasurement),
				resultCamera.vsBodyDiaExt[indexMeasurement].dValueL,
				0,
				listBodyDiaExt[indexMeasurement].sWarning,
				listBodyDiaExt[indexMeasurement].sStandard);
			measurements.push_back(resBodyDiaExtm);
			s_Measurement resBodyDiaExtx = AssignMeasurement(
				QString("BE%1x").arg(indexMeasurement),
				resultCamera.vsBodyDiaExt[indexMeasurement].dValueH,
				0,
				listBodyDiaExt[indexMeasurement].sWarning,
				listBodyDiaExt[indexMeasurement].sStandard);
			measurements.push_back(resBodyDiaExtx);
		}
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::BodyDiameter).size() != 0)
	{
		for (int indexMeasurement = 0; indexMeasurement < listBodyDiaExt.size(); indexMeasurement++)
		{
			s_Measurement resBodyDiam = AssignMeasurement(
				QString("BD%1m").arg(indexMeasurement),
				resultCamera.vsBodyDiaExt[indexMeasurement].dValueL,
				0,
				listBodyDia[indexMeasurement].sWarning,
				listBodyDia[indexMeasurement].sStandard);
			measurements.push_back(resBodyDiam);
			s_Measurement resBodyDiax = AssignMeasurement(
				QString("BD%1x").arg(indexMeasurement),
				resultCamera.vsBodyDiaExt[indexMeasurement].dValueH,
				0,
				listBodyDia[indexMeasurement].sWarning,
				listBodyDia[indexMeasurement].sStandard);
			measurements.push_back(resBodyDiax);
		}
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::AxisOffset).size() != 0)
	{
		for (int indexMeasurement = 0; indexMeasurement < listBodyDiaExt.size(); indexMeasurement++)
		{
			s_Measurement resAxisOffset = AssignMeasurement(
				QString("AO%1x").arg(indexMeasurement),
				resultCamera.vsAxisOffset[indexMeasurement].dValueH,
				0,
				listAxisOffset[indexMeasurement].sWarning,
				listAxisOffset[indexMeasurement].sStandard);
			measurements.push_back(resAxisOffset);
		}
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::TotalHeight).size() != 0)
	{
		s_Measurement resTotalHeightm = AssignMeasurement(
			QString("TT0m"),
			resultCamera.sTotalHeight.dValueL,
			0,
			listTotalHei[0].sWarning,
			listTotalHei[0].sStandard);
		measurements.push_back(resTotalHeightm);
		s_Measurement resTotalHeightx = AssignMeasurement(
			QString("TT0x"),
			resultCamera.sTotalHeight.dValueH,
			0,
			listTotalHei[0].sWarning,
			listTotalHei[0].sStandard);
		measurements.push_back(resTotalHeightx);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::FinishHorizontalAngle).size() != 0)
	{
		s_Measurement resFinHoriAngm = AssignMeasurement(
			QString("FA0m"),
			resultCamera.sFinHoriAng.dValueL,
			0,
			listFinHoriAng[0].sWarning,
			listFinHoriAng[0].sStandard);
		measurements.push_back(resFinHoriAngm);
		s_Measurement resFinHoriAngx = AssignMeasurement(
			QString("FA0x"),
			resultCamera.sFinHoriAng.dValueH,
			0,
			listFinHoriAng[0].sWarning,
			listFinHoriAng[0].sStandard);
		measurements.push_back(resFinHoriAngx);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::FinishHeight).size() != 0)
	{
		s_Measurement resFinHeim = AssignMeasurement(
			QString("FH0m"),
			resultCamera.sFinHeight.dValueL,
			0,
			listFinHei[0].sWarning,
			listFinHei[0].sStandard);
		measurements.push_back(resFinHeim);
		s_Measurement resFinHeix = AssignMeasurement(
			QString("FH0x"),
			resultCamera.sFinHeight.dValueH,
			0,
			listFinHei[0].sWarning,
			listFinHei[0].sStandard);
		measurements.push_back(resFinHeix);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::FinishPFHeight).size() != 0)
	{
		double dFinPFHeiMin = resultCamera.sFinPFHeiLeft.dValueL > resultCamera.sFinPFHeiRight.dValueL
			? resultCamera.sFinPFHeiRight.dValueL
			: resultCamera.sFinPFHeiLeft.dValueL;
		s_Measurement resFinPFHeim = AssignMeasurement(
			QString("FP0m"),
			dFinPFHeiMin,
			0,
			listFinPFHei[0].sWarning,
			listFinPFHei[0].sStandard);
		measurements.push_back(resFinPFHeim);

		double dFinPFHeiMax = resultCamera.sFinPFHeiLeft.dValueH > resultCamera.sFinPFHeiRight.dValueH
			? resultCamera.sFinPFHeiLeft.dValueH
			: resultCamera.sFinPFHeiRight.dValueH;
		s_Measurement resFinPFHeix = AssignMeasurement(
			QString("FP0x"),
			dFinPFHeiMax,
			0,
			listFinPFHei[0].sWarning,
			listFinPFHei[0].sStandard);
		measurements.push_back(resFinPFHeix);
	}

	if (sCheckModel.mhashInspections.values(EInspectionType::ScrewDiameter).size() != 0)
	{
		s_Measurement resScrewDiam = AssignMeasurement(
			QString("SD0m"),
			resultCamera.sScrewDia.dValueL,
			0,
			listScrewDia[0].sWarning,
			listScrewDia[0].sStandard);
		measurements.push_back(resScrewDiam);
		s_Measurement resScrewDiax = AssignMeasurement(
			QString("SD0x"),
			resultCamera.sScrewDia.dValueH,
			0,
			listScrewDia[0].sWarning,
			listScrewDia[0].sStandard);
		measurements.push_back(resScrewDiax);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::ScrewSpace).size() != 0)
	{
		s_Measurement resScrewSpacem = AssignMeasurement(
			QString("SS0m"),
			resultCamera.sScrewSpace.dValueL,
			0,
			listScrewSpace[0].sWarning,
			listScrewSpace[0].sStandard);
		measurements.push_back(resScrewSpacem);
		s_Measurement resScrewSpacex = AssignMeasurement(
			QString("SS0x"),
			resultCamera.sScrewSpace.dValueH,
			0,
			listScrewSpace[0].sWarning,
			listScrewSpace[0].sStandard);
		measurements.push_back(resScrewSpacex);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::ScrewHeight).size() != 0)
	{
		s_Measurement resScrewHeightm = AssignMeasurement(
			QString("SH0m"),
			resultCamera.sScrewHeight.dValueL,
			0,
			listScrewHei[0].sWarning,
			listScrewHei[0].sStandard);
		measurements.push_back(resScrewHeightm);
		s_Measurement resScrewHeightx = AssignMeasurement(
			QString("SH0x"),
			resultCamera.sScrewHeight.dValueH,
			0,
			listScrewHei[0].sWarning,
			listScrewHei[0].sStandard);
		measurements.push_back(resScrewHeightx);
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::ExtremeFinishDiameter).size() != 0)
	{
		for (int indexMeasurement = 0; indexMeasurement < listBodyDiaExt.size(); indexMeasurement++)
		{
			s_Measurement resFinDiaExtm = AssignMeasurement(
				QString("FE%1m").arg(indexMeasurement),
				resultCamera.vsFinDiaExt[indexMeasurement].dValueL,
				0,
				listFinDiaExt[indexMeasurement].sWarning,
				listFinDiaExt[indexMeasurement].sStandard);
			measurements.push_back(resFinDiaExtm);
			s_Measurement resFinDiaExtx = AssignMeasurement(
				QString("FE%1x").arg(indexMeasurement),
				resultCamera.vsFinDiaExt[indexMeasurement].dValueH,
				0,
				listFinDiaExt[indexMeasurement].sWarning,
				listFinDiaExt[indexMeasurement].sStandard);
			measurements.push_back(resFinDiaExtx);
		}
	}
	if (sCheckModel.mhashInspections.values(EInspectionType::FinishDiameter).size() != 0)
	{
		for (int indexMeasurement = 0; indexMeasurement < listBodyDiaExt.size(); indexMeasurement++)
		{
			s_Measurement resFinDiam = AssignMeasurement(
				QString("FD%1m").arg(indexMeasurement),
				resultCamera.vsFinDia[indexMeasurement].dValueL,
				0,
				listFinDia[indexMeasurement].sWarning,
				listFinDia[indexMeasurement].sStandard);
			measurements.push_back(resFinDiam);
			s_Measurement resFinDiax = AssignMeasurement(
				QString("FD%1x").arg(indexMeasurement),
				resultCamera.vsFinDia[indexMeasurement].dValueH,
				0,
				listFinDia[indexMeasurement].sWarning,
				listFinDia[indexMeasurement].sStandard);
			measurements.push_back(resFinDiax);
		}
	}
#pragma endregion Measurements Camera
#pragma region Measurements Thickness
	QList<s_Inspection> listThickness = sCheckModel.mhashInspections.values(EInspectionType::Thickness);
	if (listThickness.size() != 0)
	{
		resultThickness = m_sDataOfBottles.vExtResultThickness[bottleIndex];
		for (int indexMeasurement = 0; indexMeasurement < listThickness.size(); indexMeasurement++)
		{
			s_Measurement resThicknessm = AssignMeasurement(
				QString("TH%1m").arg(indexMeasurement),
				resultThickness.vsThickness[indexMeasurement].dValueL,
				0,
				listThickness[indexMeasurement].sWarning,
				listThickness[indexMeasurement].sStandard);
			measurements.push_back(resThicknessm);
			s_Measurement resThicknessx = AssignMeasurement(
				QString("TH%1x").arg(indexMeasurement),
				resultThickness.vsThickness[indexMeasurement].dValueH,
				0,
				listThickness[indexMeasurement].sWarning,
				listThickness[indexMeasurement].sStandard);
			measurements.push_back(resThicknessx);
		}
	}
#pragma endregion Measurements Thickness
#pragma region Measurements Inner
	QList<s_Inspection> listInnerDia = sCheckModel.mhashInspections.values(EInspectionType::InnerDiameter);
	if (listInnerDia.size() != 0)
	{
		resultInner = m_sDataOfBottles.vExtResultInner[bottleIndex];
		for (int indexMeasurement = 0; indexMeasurement < listThickness.size(); indexMeasurement++)
		{
			s_Measurement resThicknessm = AssignMeasurement(
				QString("ID%1m").arg(indexMeasurement),
				resultInner.vsInnerDia[indexMeasurement].dValueL,
				0,
				listInnerDia[indexMeasurement].sWarning,
				listInnerDia[indexMeasurement].sStandard);
			measurements.push_back(resThicknessm);
			s_Measurement resThicknessx = AssignMeasurement(
				QString("ID%1x").arg(indexMeasurement),
				resultInner.vsInnerDia[indexMeasurement].dValueH,
				0,
				listInnerDia[indexMeasurement].sWarning,
				listInnerDia[indexMeasurement].sStandard);
			measurements.push_back(resThicknessx);
		}
	}
#pragma endregion Measurements Inner
#pragma region Measurements Weight
	QList<s_Inspection> listWeight = sCheckModel.mhashInspections.values(EInspectionType::WeightInspection);
	if (listWeight.size() != 0)
	{
		resultWeight = m_sDataOfBottles.vExtResultWeight[bottleIndex];
		s_Measurement resWeight = AssignMeasurement(
			QString("WT0m"),
			resultWeight,
			0,
			listWeight[0].sWarning,
			listWeight[0].sStandard);
		measurements.push_back(resWeight);
	}
#pragma endregion Measurements Weight
	return measurements;
}

s_Measurement CConsole::AssignMeasurement(QString strName, double dValue, double dSetPointValue, s_Range sWarningRange, s_Range sErrorRange)
{
	s_Measurement sMeasurement;
	sMeasurement.id = strName;
	sMeasurement.value = dValue; //result
	sMeasurement.setPointVal = dSetPointValue;
	sMeasurement.warningMin = sWarningRange.dValueL;
	sMeasurement.warningMax = sWarningRange.dValueH;
	sMeasurement.errorMin = sErrorRange.dValueL;
	sMeasurement.errorMax = sErrorRange.dValueH;
	return sMeasurement;
}

//功能：开始
void CConsole::Start()
{
	if (m_sysConfigPara.strCurModelName.isEmpty())
	{
		QMessageBox::information(NULL, tr("Information"), tr("Please define the bottle model!"));
		return;
	}
	if (!m_sCheckModelPara.bCheckMoldReader)
	{
		//IQSCAN & Manual input MOLD number
		if (m_sysConfigPara.nOperatingMode == EOperatingMode::Semi_Automatic)
		{
			//on auto mode, we do not need to enter the mold numbers, the mold number is returned by IQScan/iAFIS
			int nMsgRtn = QMessageBox::question(m_coldLabFrame, tr("Mold Number"), tr("Do you need to input mold numbers manually?"),
				QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
			if (QMessageBox::Yes == nMsgRtn)
			{
				m_sysOperaPara.bShowMoldNumber = true;

				CDlgMoldNumMannul dlgMoldNum(m_coldLabFrame);
				dlgMoldNum.exec();
			}
			else if (QMessageBox::No == nMsgRtn)
			{
				m_sysOperaPara.bShowMoldNumber = false;
			}
			else if (QMessageBox::Cancel == nMsgRtn)
			{
				m_sysOperaPara.bShowMoldNumber = false;
				//m_sysOperaPara.bstartDetect = false;
				return;
			}
		}
		else
		{
			m_sysOperaPara.bShowMoldNumber = true;
		}
	}
	else
	{
		m_sysOperaPara.bShowMoldNumber = true;
	}

	//ask if start detections
	int nRtn = QMessageBox::question(m_coldLabFrame, tr("MoldNum"), tr("Do you want to start detecting now ?"),
		QMessageBox::Yes | QMessageBox::No);
	if (QMessageBox::No == nRtn)
	{
		m_sysOperaPara.bStartDetect = false;
		return;
	}
	m_pcIQScan->m_vMoldNum.clear();

	//// Calculate PLC position =>Recal
	//if ((m_sCheckModelPara.bCheckInner && m_pcInner->m_sInnerCalibPara.strCalibDate != m_pcTPLCComunication->m_sPLCModelPara.strInnerDiaCalibDate)
	//	|| (m_sCheckModelPara.bCheckBody || m_sCheckModelPara.bCheckFinish) && 
	//		(m_pcImageProcessing->m_sCameraCalibrationPara[0].strCalibDate != m_pcTPLCComunication->m_sPLCModelPara.strBodyCameraCalibDate
	//		|| m_pcImageProcessing->m_sCameraCalibrationPara[1].strCalibDate != m_pcTPLCComunication->m_sPLCModelPara.strFinishCameraCalibDate))
	{
		if (m_pcInner->m_sInnerCalibPara.vsCalibInfo.size() == 0)
		{
			QMessageBox::warning(NULL, tr("Information"), tr("Please do the innner diameter calibration firstly!"));
			return;
		}
		if (m_pcImageProcessing->m_sCameraCalibrationPara[0].vsCalibPara.size() == 0
			|| m_pcImageProcessing->m_sCameraCalibrationPara[1].vsCalibPara.size() == 0)
		{
			QMessageBox::warning(NULL, tr("Information"), tr("Please do the camera calibration firstly!"));
			return;
		}
		bool bUpdateSuccess = UpdateModel(
			m_sysConfigPara.strCurModelName,
			&m_pcTPLCComunication->m_sPLCModelPara,
			&m_sCheckModelPara,
			&m_sPlotRange,
			&m_pcTPLCComunication->m_sMotorCalibPara,
			m_pcImageProcessing->m_sCameraCalibrationPara[0],
			m_pcImageProcessing->m_sCameraCalibrationPara[1],
			m_pcInner->m_sInnerCalibPara);

		if (!bUpdateSuccess) 
		{
			QMessageBox::warning(NULL, tr("Information"), tr("Failed to update the model, please reconfigure it!"));
			return;
		}
	}
	if (m_sCheckModelPara.bCheckBody || m_sCheckModelPara.bCheckFinish || m_sCheckModelPara.bCheckAxisOffset)
	{
		m_pcImageProcessing->UpdateCheckObject();
	}
	if (!m_pcTPLCComunication->IsPlcConnected())
	{
		QMessageBox::warning(NULL, tr("Error"), tr("Failed to connect to PLC!"));
		m_sysOperaPara.bStartDetect = false;
		return;
	}
	m_sysOperaPara.bStartDetect = true;
	//设置PLC控制指令
	s_PLCLightSouceControl sPLCLightSouceControl;
	m_pcTPLCComunication->SetLightSouceControl(sPLCLightSouceControl);
	m_pcTPLCComunication->SetParam(m_pcTPLCComunication->m_sPLCModelPara, m_pcTPLCComunication->m_sMotorCalibPara);
	m_pcTPLCComunication->SetMode(COMMAND_AUTO);
	m_pcTPLCComunication->SetCtrlCmd(CMDCTRL_START);

	m_coldLabFrame->ui.btnStart->setEnabled(false);
	//开始相机采集
	//for (int i = 0; i < m_sysConfigPara.nCamSceneCount; ++i)
	//{
	//	m_pcImageProcessing->m_sCamInfo[i].nImageSN = 0;
	//	m_pcImageProcessing->m_sCamInfo[i].bStartGrab = true;
	//	m_pcImageProcessing->m_pGrabber[i]->StartGrab();
	//}
	m_pcImageProcessing->StartCamGrab();
	if (m_sysConfigPara.bTest)
	{
		m_coldLabFrame->ui.btnDebug->setEnabled(true);
		m_coldLabFrame->ui.btnCalib->setEnabled(true);
		m_coldLabFrame->ui.btnModel->setEnabled(true);
	}
	else
	{
		m_coldLabFrame->ui.btnDebug->setEnabled(false);
		m_coldLabFrame->ui.btnCalib->setEnabled(false);
		m_coldLabFrame->ui.btnModel->setEnabled(false);
	}
	//*清空瓶子数据，重新统计
	m_sDataOfBottles.clear();
	m_sysOperaPara.startTime = QTime::currentTime();
	m_sysOperaPara.nTotalCount = 0;
	m_sysOperaPara.nWeightCount = 0;
	m_sysOperaPara.nDimensionCount = 0;
	m_sysOperaPara.nThicknessCount = 0;
	m_sysOperaPara.nInDiaCount = 0;
	m_sysOperaPara.nNGTotal = 0;
	m_sysOperaPara.nNGWeight = 0;
	m_sysOperaPara.nNGDimention = 0;
	m_sysOperaPara.nNGThickness = 0;
	m_sysOperaPara.nNGInnerDia = 0;
	vJudgeIdx.clear();//20141112新增		
	m_coldLabFrame->ui.btnStart->setEnabled(true);
	m_coldLabFrame->ui.btnStart->setIcon(QIcon(":/FuncBtns/Resources/FuncBtns/stop.png"));
	emit signals_addLogText(tr("Start check."), EFileNameType::Operate);
	m_coldLabFrame->ui.btnQuit->setEnabled(false);
	m_coldLabFrame->ui.lcdTotal->display(m_sysOperaPara.nTotalCount);
	m_coldLabFrame->ui.lcdWeight->display(m_sysOperaPara.nWeightCount);
	m_coldLabFrame->ui.lcdDimention->display(m_sysOperaPara.nDimensionCount);
	m_coldLabFrame->ui.lcdThickness->display(m_sysOperaPara.nThicknessCount);
	m_coldLabFrame->ui.lcdInnerDia->display(m_sysOperaPara.nInDiaCount);
	m_coldLabFrame->ui.labelTotalNG->setText(QString::number(m_sysOperaPara.nNGTotal));
	m_coldLabFrame->ui.labelWeightNG->setText(QString::number(m_sysOperaPara.nNGWeight));
	m_coldLabFrame->ui.labelDimentionNG->setText(QString::number(m_sysOperaPara.nNGDimention));
	m_coldLabFrame->ui.labelThicknessNG->setText(QString::number(m_sysOperaPara.nNGThickness));
	m_coldLabFrame->ui.labelInnerDiaNG->setText(QString::number(m_sysOperaPara.nNGInnerDia));
}
void CConsole::Stop()
{
	if (QMessageBox::No == QMessageBox::question(m_coldLabFrame, tr("Stop"), tr("Are you sure to stop this task?"),
		QMessageBox::Yes | QMessageBox::No))
		return;

	if (!m_pcTPLCComunication->IsPlcConnected())
	{
		QMessageBox::warning(NULL, tr("Error"), tr("Failed to connect to PLC!"));
		m_sysOperaPara.bStartDetect = false;
		return;
	}
	//设置PLC控制指令
	m_pcTPLCComunication->SetMode(COMMAND_AUTO);
	m_pcTPLCComunication->SetCtrlCmd(CMDCTRL_STOP);

	m_sysOperaPara.bStartDetect = false;
	m_coldLabFrame->ui.btnStart->setEnabled(false);
	//关闭相机采集
	for (int indexCamera = 0; indexCamera < m_sysConfigPara.nCamSceneCount; indexCamera++)
	{
		m_pcImageProcessing->m_sCamInfo[indexCamera].bStartGrab = false;
		m_pcImageProcessing->m_pGrabber[indexCamera]->StopGrab();
	}
	//m_pcImageProcessing->StopCamGrab();
	m_coldLabFrame->ui.btnQuit->setEnabled(true);
	m_coldLabFrame->ui.btnModel->setEnabled(true);
	m_coldLabFrame->ui.btnDebug->setEnabled(true && m_sysOperaPara.nLoginLevel >= 1);
	m_coldLabFrame->ui.btnCalib->setEnabled(true && m_sysOperaPara.nLoginLevel >= 1);
	m_coldLabFrame->ui.btnStart->setEnabled(true);
	m_coldLabFrame->ui.btnStart->setIcon(QIcon(":/FuncBtns/Resources/FuncBtns/start.png"));
	emit signals_addLogText(tr("Stop check."), EFileNameType::Operate);
	int nCheckTime = 0 - QTime::currentTime().secsTo(m_sysOperaPara.startTime);
	emit signals_addLogText(tr("Check %1 bottles in %2 mins %3 secs.").arg(m_sysOperaPara.nTotalCount).arg(nCheckTime / 60).arg(nCheckTime % 60), EFileNameType::Operate);
	//恢复动画到初始状态
	m_coldLabFrame->m_pStatusPage->AdaptView(0);
	//		if (!sPLCStatusUpg.bCylinder3)
	if (m_pcTPLCComunication->m_sPLCStatus.bCylinderInOut)
		m_coldLabFrame->m_pStatusPage->ui.viewSim->slots_eleCylinderArmStretch(false);//20140925防止伸缩气缸伸两次
	if (m_pcTPLCComunication->m_sPLCStatus.bCylinderCloseOpen)
		m_coldLabFrame->m_pStatusPage->ui.viewSim->slots_cylinderArmGripper(true);//20141029夹持气缸
	for (int indexBottleImg = 0; indexBottleImg < VIEW_BOTTLE_COUNT; indexBottleImg++)//清空瓶子
	{
		if (m_coldLabFrame->m_pStatusPage->m_bottles[indexBottleImg] != NULL)
		{
			delete m_coldLabFrame->m_pStatusPage->m_bottles[indexBottleImg];
			m_coldLabFrame->m_pStatusPage->m_bottles[indexBottleImg] = NULL;
		}
	}
	m_pcTPLCComunication->SetSystemReset();

	if (m_sysOperaPara.nTotalCount > 0)
	{
		m_coldLabFrame->slots_report();
	}
}
void CConsole::Quit()
{
	if (m_sysConfigPara.bSimulateMode) return;
	if (m_pcTPLCComunication->IsPlcConnected())
	{
		m_pcTPLCComunication->SetMode(COMMAND_AUTO);
		m_pcTPLCComunication->SetCtrlCmd(CMDCTRL_STOP);

		if (m_pcTPLCComunication != NULL)
		{
			if (m_pcTPLCComunication->isRunning())
			{
				m_pcTPLCComunication->stop();
				m_pcTPLCComunication->wait();
			}
			delete m_pcTPLCComunication;
			m_pcTPLCComunication = NULL;
		}
	}
	if (m_pcImageProcessing != NULL)
	{
		delete m_pcImageProcessing;
		m_pcImageProcessing = NULL;
	}
	if (m_pcThicknessProcessing != NULL)
	{
		delete m_pcThicknessProcessing;
		m_pcThicknessProcessing = NULL;
	}
	if (m_pcInner != NULL)
	{
		delete m_pcInner;
		m_pcInner = NULL;
	}
	if (m_pcTInnerSimulation != NULL)
	{
		if (m_pcTInnerSimulation->isRunning())
		{
			m_pcTInnerSimulation->stop();
		}
		delete m_pcTInnerSimulation;
		m_pcTInnerSimulation = NULL;
	}
}

QString CConsole::getInspectionName(EInspectionType eInsp, int indexWidget)
{
	QString strInspName = "";
	if (eInsp == EInspectionType::ExtremeBodyDiameter)
	{
		switch (indexWidget)
		{
		case 0:
			strInspName = m_sysOperaPara.sItemName.strBodyDiaExt1;
			break;
		case 1:
			strInspName = m_sysOperaPara.sItemName.strBodyDiaExt2;
			break;
		default:
			break;
		}
	}
	if (eInsp == EInspectionType::BodyDiameter)
	{
		switch (indexWidget)
		{
		case 0:
			strInspName = m_sysOperaPara.sItemName.strBodyDia1;
			break;
		case 1:
			strInspName = m_sysOperaPara.sItemName.strBodyDia2;
			break;
		case 2:
			strInspName = m_sysOperaPara.sItemName.strBodyDia3;
			break;
		case 3:
			strInspName = m_sysOperaPara.sItemName.strBodyDia4;
			break;
		case 4:
			strInspName = m_sysOperaPara.sItemName.strBodyDia5;
			break;
		default:
			break;
		}
	}
	if (eInsp == EInspectionType::ExtremeFinishDiameter)
	{
		switch (indexWidget)
		{
		case 0:
			strInspName = m_sysOperaPara.sItemName.strFinDiaExt1;
			break;
		case 1:
			strInspName = m_sysOperaPara.sItemName.strFinDiaExt2;
			break;
		default:
			break;
		}
	}
	if (eInsp == EInspectionType::FinishDiameter)
	{
		switch (indexWidget)
		{
		case 0:
			strInspName = m_sysOperaPara.sItemName.strFinDia1;
			break;
		case 1:
			strInspName = m_sysOperaPara.sItemName.strFinDia2;
			break;
		case 2:
			strInspName = m_sysOperaPara.sItemName.strFinDia3;
			break;
		case 3:
			strInspName = m_sysOperaPara.sItemName.strFinDia4;
			break;
		case 4:
			strInspName = m_sysOperaPara.sItemName.strFinDia5;
			break;
		default:
			break;
		}
	}
	if (eInsp == EInspectionType::Thickness)
	{
		switch (indexWidget)
		{
		case 0:
			strInspName = m_sysOperaPara.sItemName.strThickness1;
			break;
		case 1:
			strInspName = m_sysOperaPara.sItemName.strThickness2;
			break;
		case 2:
			strInspName = m_sysOperaPara.sItemName.strThickness3;
			break;
		case 3:
			strInspName = m_sysOperaPara.sItemName.strThickness4;
			break;
		case 4:
			strInspName = m_sysOperaPara.sItemName.strThickness5;
			break;
		default:
			break;
		}
	}
	if (eInsp == EInspectionType::InnerDiameter)
	{
		switch (indexWidget)
		{
		case 0:
			strInspName = m_sysOperaPara.sItemName.strInnerDia1;
			break;
		case 1:
			strInspName = m_sysOperaPara.sItemName.strInnerDia2;
			break;
		case 2:
			strInspName = m_sysOperaPara.sItemName.strInnerDia3;
			break;
		case 3:
			strInspName = m_sysOperaPara.sItemName.strInnerDia4;
			break;
		case 4:
			strInspName = m_sysOperaPara.sItemName.strInnerDia5;
			break;
		default:
			break;
		}
	}
	return strInspName;
}

QString CConsole::GetInspectionName(EInspectionType eInspection, int indexWidget)
{
	QString strName;
	switch (eInspection)
	{
	case EInspectionType::Flatness:
		strName = m_sysOperaPara.sItemName.strFlatness;
		break;
	case EInspectionType::ExtremeBodyDiameter:
		strName = getInspectionName(EInspectionType::ExtremeBodyDiameter, indexWidget);
		break;
	case EInspectionType::BodyDiameter:
		strName = getInspectionName(EInspectionType::BodyDiameter, indexWidget);
		break;
	case EInspectionType::AxisOffset:
		strName = m_sysOperaPara.sItemName.strAxisOffset;
		break;
	case EInspectionType::TotalHeight:
		strName = m_sysOperaPara.sItemName.strTotalHeight;
		break;
	case EInspectionType::FinishHorizontalAngle:
		strName = m_sysOperaPara.sItemName.strFinHoriAng;
		break;
	case EInspectionType::FinishHeight:
		strName = m_sysOperaPara.sItemName.strFinHei;
		break;
	case EInspectionType::FinishPFHeight:
		strName = m_sysOperaPara.sItemName.strFinPFHei;
		break;
	case EInspectionType::ScrewDiameter:
		strName = m_sysOperaPara.sItemName.strScrewDia;
		break;
	case EInspectionType::ScrewSpace:
		strName = m_sysOperaPara.sItemName.strScrewSpace;
		break;
	case EInspectionType::ScrewHeight:
		strName = m_sysOperaPara.sItemName.strScrewHeight;
		break;
	case EInspectionType::ExtremeFinishDiameter:
		strName = getInspectionName(EInspectionType::ExtremeFinishDiameter, indexWidget);
		break;
	case EInspectionType::FinishDiameter:
		strName = getInspectionName(EInspectionType::FinishDiameter, indexWidget);
		break;
	case EInspectionType::Thickness:
		strName = getInspectionName(EInspectionType::Thickness, indexWidget);
		break;
	case EInspectionType::InnerDiameter:
		strName = getInspectionName(EInspectionType::InnerDiameter, indexWidget);
		break;
	case EInspectionType::WeightInspection:
		strName = m_sysOperaPara.sItemName.strWeight;
		break;
	default:
		break;
	}
	return strName;
}

void CConsole::slots_addLogText(QString str, EFileNameType eFileNameType)
{
	m_coldLabFrame->ui.textEditLog->addToLog(str, eFileNameType);
}
//功能：当前工位数据压入队列，判断当前工位瓶子是否合格（在夹爪闭合的时候根据当前工位序号判断）
void CConsole::slots_cylinderArmGripper(bool bFlag)
{
	if (!bFlag)//夹爪闭合
	{
		switch (m_pcTPLCComunication->m_sPLCStatus.nArmPosition)
		{
		case EGripperPosition::InfeedPosition:
		{
			s_JudgeIdx sJudge;
			sJudge.nIdx = m_pcTPLCComunication->m_sPLCStatus.nBotIdxInfeed;
			vJudgeIdx.push_back(sJudge);

			emit m_pcIQScan->signals_getMouldNumber();
		}
		break;
		case EGripperPosition::WeightPosition:
		{
			if (!JudgeWeight(m_sJudgePara, m_coldLabFrame->m_pDataPage->m_dExtResultWeight))
			{
				m_sysOperaPara.nNGWeight++;
				m_coldLabFrame->ui.labelWeightNG->setText(QString::number(m_sysOperaPara.nNGWeight));
				for (int indexBottle = 0; indexBottle<vJudgeIdx.size(); indexBottle++)
				{
					if (vJudgeIdx[indexBottle].nIdx == m_pcTPLCComunication->m_sPLCStatus.nBotIdxWeight)
					{
						vJudgeIdx[indexBottle].bPass = false;
						break;
					}
				}
			}
		}
		break;
		case EGripperPosition::CameraPosition:
		{
			//压入视觉数据
			if (m_sysConfigPara.bSimulateMode)
			{
				QString strPath = m_sysOperaPara.strAppPath + "/Config/Totalheight.ini";
				for (int i = 0; i<m_coldLabFrame->m_pDataPage->m_vSynResult.vTotalHeight.size(); i++)
				{
					WritePrivateProfileString(
						QString("Totalheight").toStdWString().c_str(),
						QString::number(i).toStdWString().c_str(),
						QString::number(m_coldLabFrame->m_pDataPage->m_vSynResult.vTotalHeight.at(i)).toStdWString().c_str(),
						strPath.toStdWString().c_str());

				}
			}
			if (m_coldLabFrame->m_pConsole->m_sCheckModelPara.bCheckBody
				|| m_coldLabFrame->m_pConsole->m_sCheckModelPara.bCheckFinish
				|| m_coldLabFrame->m_pConsole->m_sCheckModelPara.bCheckAxisOffset
				|| m_coldLabFrame->m_pConsole->m_sCheckModelPara.bCheckMoldReader
				)
			{
				m_sysOperaPara.nDimensionCount++;
				m_coldLabFrame->ui.lcdDimention->display(m_sysOperaPara.nDimensionCount);
				m_sDataOfBottles.vExtResultCamera.push_back(m_coldLabFrame->m_pDataPage->m_sExtResultCamera);
				m_pcImageProcessing->m_vAxisFix.clear();
				//判断视觉
				if (!JudgeDimention(m_sJudgePara, m_coldLabFrame->m_pDataPage->m_sExtResultCamera))
				{
					m_sysOperaPara.nNGDimention++;
					m_coldLabFrame->ui.labelDimentionNG->setText(QString::number(m_sysOperaPara.nNGDimention));
					for (int indexBottle = 0; indexBottle<vJudgeIdx.size(); indexBottle++)
					{
						if (vJudgeIdx[indexBottle].nIdx == m_pcTPLCComunication->m_sPLCStatus.nBotIdxCamera)
						{
							vJudgeIdx[indexBottle].bPass = false;
							break;
						}
					}
				}
				//清零图像号
				for (int indexCamera = 0; indexCamera<CAMERA_MAX_COUNT; indexCamera++)
					m_pcImageProcessing->m_sCamInfo[indexCamera].nImageSN = 0;
				//清除读模器显示,20150130
				if (m_sCheckModelPara.bCheckMoldReader)
				{
					//[yixue]uncomment later after evolution of algorithm
					//clear_window(lMoldWndID);
					//m_coldLabFrame->ui.lcdMold->display(0);
				}
				//清除轴偏差上下相机标志位//2015.5.12[YL]
				m_coldLabFrame->m_pDataPage->ClearAxisSign();
				for (int indexInspAxisOffset = 0; indexInspAxisOffset < m_coldLabFrame->m_pDataPage->m_sExtResultCamera.vsAxisOffset.size(); indexInspAxisOffset++)
				{
					m_coldLabFrame->m_pDataPage->m_sExtResultCamera.vsAxisOffset[indexInspAxisOffset].dValueH = 0;
				}
			}
			//壁厚
			if (m_sCheckModelPara.bCheckThickness)
			{
				//压入队列
				m_sysOperaPara.nThicknessCount++;
				m_coldLabFrame->ui.lcdThickness->display(m_sysOperaPara.nThicknessCount);
				m_sDataOfBottles.vExtResultThickness.push_back(m_coldLabFrame->m_pDataPage->m_sExtResultThickness);
				//判断
				if (!JudgeThickness(m_sJudgePara, m_coldLabFrame->m_pDataPage->m_sExtResultThickness))
				{
					m_sysOperaPara.nNGThickness++;
					m_coldLabFrame->ui.labelThicknessNG->setText(QString::number(m_sysOperaPara.nNGThickness));
					for (int i = 0; i<vJudgeIdx.size(); ++i)
					{
						if (vJudgeIdx[i].nIdx == m_pcTPLCComunication->m_sPLCStatus.nBotIdxCamera)
						{
							vJudgeIdx[i].bPass = false;
							break;
						}
					}
				}
			}
			//emit signals_FinishCam();//[yixue]no connected slots, maybe should be removed
		}
		break;
		case EGripperPosition::InnerDiameterPosition:
		{
			if (m_sCheckModelPara.bCheckInner)
			{
				//压入队列
				m_sysOperaPara.nInDiaCount++;
				m_coldLabFrame->ui.lcdInnerDia->display(m_sysOperaPara.nInDiaCount);
				m_sDataOfBottles.vExtResultInner.push_back(m_coldLabFrame->m_pDataPage->m_sExtResultInner);
				//判断
				if (!JudgeInnerDia(m_sJudgePara, m_coldLabFrame->m_pDataPage->m_sExtResultInner))
				{
					m_sysOperaPara.nNGInnerDia++;
					m_coldLabFrame->ui.labelInnerDiaNG->setText(QString::number(m_sysOperaPara.nNGInnerDia));
					for (int i = 0; i<vJudgeIdx.size(); ++i)
					{
						if (vJudgeIdx[i].nIdx == m_pcTPLCComunication->m_sPLCStatus.nBotIdxInner)
						{
							vJudgeIdx[i].bPass = false;
							break;
						}
					}
				}
			}
			//emit signals_FinishInner();//[yixue]no connected slots, maybe should be removed
		}
		break;
		default: //case EGripperPosition::OutfeedPosition:出瓶时夹爪不会闭合
			break;
		}
	}
	else//夹爪松开,清空曲线数据
	{
		switch (m_pcTPLCComunication->m_sPLCStatus.nArmPosition)
		{
		case EGripperPosition::CameraPosition:
			m_coldLabFrame->m_pDataPage->m_vSynResult.clearCamera();//清空上一瓶的视觉检测曲线数据
			m_coldLabFrame->m_pDataPage->m_vSynResult.clearThickness();//清空上一瓶的壁厚检测曲线数据
			break;
		case EGripperPosition::InnerDiameterPosition:
			m_coldLabFrame->m_pDataPage->m_vSynResult.clearInner();//清空上一瓶的内径检测曲线数据
			break;
		case EGripperPosition::OutfeedPosition:
			emit m_coldLabFrame->m_pConsole->m_pcIQScan->signals_writeValues();
			break;
		default:
			break;
		}
	}
}

//功能：缺料判断
void CConsole::slots_hasBottle(bool bFlag)
{
	if (bFlag)
	{
		m_coldLabFrame->m_timerIdle->start(m_sysConfigPara.nIdleRemind * 60 * 1000);//开始计时，bRunning=false时停止
	}
	SetAlarmLight(ALARM_NORMAL);
}
//功能：设置报警灯
void CConsole::SetAlarmLight(int nCmd)
{
	m_pcTPLCComunication->SetAlarmLight(nCmd);
}
//功能：出瓶输送带运行时，统计总数，判断整体不合格数
void CConsole::slots_outfeedConveyor(bool bFlag)
{
	if (bFlag)
	{
		m_sysOperaPara.nTotalCount++;
		m_coldLabFrame->ui.lcdTotal->display(m_sysOperaPara.nTotalCount);
		for (int indexBottle = 0; indexBottle<vJudgeIdx.size(); indexBottle++)
		{
			if (vJudgeIdx[indexBottle].nIdx == m_pcTPLCComunication->m_sPLCStatus.nBotIdxOutfeed)
			{
				if (!vJudgeIdx[indexBottle].bPass)
				{
					m_sysOperaPara.nNGTotal++;
					m_coldLabFrame->ui.labelTotalNG->setText(QString::number(m_sysOperaPara.nNGTotal));
					SetAlarmLight(ALARM_RED);//20141202设置报警灯
				}
				break;
			}
		}
	}
}
//功能: 准备触发,用于在触发信号结束时判断缺图或误触发
void CConsole::slots_trigCamera(bool bFlag, int nTrigCam)
{
	m_coldLabFrame->m_pStatusPage->UpdateCameraLight(bFlag, nTrigCam);
	//触发完毕以后， 判断图像号， 并清零
	if (bFlag) return;
	int nCamTrigCount = m_pcTPLCComunication->m_sMotorCalibPara.nTriggerCount;
	if (nTrigCam == CAM1_TRIG || nTrigCam == CAM_ALL_TRIG)
	{
		if (m_pcImageProcessing->m_sCamInfo[0].nImageSN > nCamTrigCount)
		{
			emit signals_addLogText(tr("Body Camera has more trigger!"));
		}
		else if (m_pcImageProcessing->m_sCamInfo[0].nImageSN < nCamTrigCount)
		{
			emit signals_addLogText(tr("Body Camera has less trigger!"));
		}
		m_pcImageProcessing->m_sCamInfo[0].nImageSN = 0;//清零图像号
	}
	if (nTrigCam == CAM2_TRIG || nTrigCam == CAM_ALL_TRIG)
	{
		if (m_pcImageProcessing->m_sCamInfo[1].nImageSN > nCamTrigCount)
		{
			emit signals_addLogText(tr("Finish Camera has more trigger!"));
		}
		else if (m_pcImageProcessing->m_sCamInfo[1].nImageSN <  nCamTrigCount)
		{
			emit signals_addLogText(tr("Finish Camera has less trigger!"));
		}
		m_pcImageProcessing->m_sCamInfo[1].nImageSN = 0;//清零图像号
	}
}
//功能：触发壁厚传感器，开始采集线程
void CConsole::slots_trigThickness(bool bFlag)
{
	//m_sysOperaPara.nCurPosIdx = m_pcThicknessProcessing->m_pReadThicknessThread->m_nInspectionPositionCount - 1;
	if (m_pcThicknessProcessing->m_bDebug)
	{
		m_sysOperaPara.nCurPosIdx = 0;
		if (m_sysOperaPara.nCurPosIdx < 0
			|| m_sysOperaPara.nCurPosIdx > MAX_THICKNESS_POSITION_COUNT - 1)
		{
			emit signals_addLogText(tr("Wrong Thickness Position Number %1!").arg(m_sysOperaPara.nCurPosIdx));
			return;
		}
	}
	else
	{
		m_sysOperaPara.nCurPosIdx = m_pcThicknessProcessing->m_pReadThicknessThread->m_nInspectionPositionCount - 1;
		if (m_sysOperaPara.nCurPosIdx < 0
			|| m_sysOperaPara.nCurPosIdx > MAX_THICKNESS_POSITION_COUNT - 1
			|| m_sysOperaPara.nCurPosIdx > m_coldLabFrame->m_pDataPage->m_vSynResult.vovThickness.size() - 1)
		{
			emit signals_addLogText(tr("Wrong Thickness Position Number %1!").arg(m_sysOperaPara.nCurPosIdx));
			return;
		}
		m_coldLabFrame->m_pDataPage->m_vSynResult.vovThickness[m_sysOperaPara.nCurPosIdx].clear();
	}

	if (bFlag)
	{
		//[yixue]never used
		//m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->m_vsThicknessData.clear(); 
		/*
		//[yixue] will be done in readThicknessThread
		try
		{
			m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->m_pOCHR->ExecCommand("$STA");
		}
		catch (std::exception &e)
		{
			QString str = "runtime error:" + QString(QLatin1String(e.what()));
			emit signals_addLogText(str);
		}*/
		m_pcThicknessProcessing->m_pReadThicknessThread->start();
	}
	else
	{
		if (m_pcThicknessProcessing->m_pReadThicknessThread->isRunning())
		{
			m_pcThicknessProcessing->m_pReadThicknessThread->stop();
			m_pcThicknessProcessing->m_pReadThicknessThread->wait();
		}
		else
		{
			emit signals_addLogText(tr("pReadCHRThread ended before finish!").arg(m_sysOperaPara.nCurPosIdx));
		}
		/*
		//[yixue]will be done while stopping readThicknessThread
		try
		{
			m_pcThicknessProcessing->m_pReadThicknessThread->m_pCPrecitectThickness->m_pOCHR->ExecCommand("$STO");
		}
		catch (std::exception &e)
		{
			QString str = "runtime error:" + QString(QLatin1String(e.what()));
			emit signals_addLogText(str);
		}
		m_pcThicknessProcessing->m_pReadThicknessThread->mThicknessDataLocker.lock();
		m_pcThicknessProcessing->m_pReadThicknessThread->m_listThicknessInspection.append(m_pcThicknessProcessing->m_pReadThicknessThread->m_sThicknessInfomation);
		m_pcThicknessProcessing->m_pReadThicknessThread->mThicknessDataLocker.unlock();*/
	}
	m_coldLabFrame->m_pStatusPage->UpdateThicknessLight(bFlag);
}
//功能：触发内径传感器，开始采集线程
void CConsole::slots_trigInner(bool bFlag)
{
	//加入是否触发的判断
	if (bFlag) //[yixue]update simulation, prepare for the check while triggering 
	{
		//if (m_pcInner->isRunning())
		//{
		//	m_pcInner->stop();
		//	m_pcInner->wait();
		//}
		//else
		//{
		//	emit signals_addLogText(tr("CTInner ended before finish!").arg(m_sysOperaPara.nCurPosIdx));
		//}
		emit m_pcInner->signals_showCalibInfo(QString("Receive Inner Position %1 as begin!").arg(m_pcInner->m_nInnerPositionIndex + 1));
		m_pcInner->m_listsInnerPara.clear();
		m_pcInner->m_nListInner[0].clear();
		m_pcInner->m_nListInner[1].clear();
		m_pcTInnerSimulation->start();
	}
	else //check when triggering finished
	{
		try
		{
			//m_pcInner->start();
			m_pcInner->CheckInnerDia();
		}
		catch (...)
		{
			emit signals_addLogText(tr("Error in Methord CheckInnerDia"), EFileNameType::Algorithm);
		}
		emit m_pcInner->signals_innerCheckFinished();
		if (m_pcTInnerSimulation->isRunning())
		{
			m_pcTInnerSimulation->stop();
			m_pcTInnerSimulation->wait();
		}
	}
}
void CConsole::slots_trigPLCBusy(bool bPLCBusy)
{
	if (bPLCBusy)
	{
		m_coldLabFrame->m_pDlgPlcBusy->exec();
	}
	else
	{
		if (m_coldLabFrame->m_pDlgPlcBusy != NULL)
		{
			m_coldLabFrame->m_pDlgPlcBusy->accept();
		}
	}
}

//功能：判断当视觉工位旋转时，如果用于识别模号，则开启相机触发
void CConsole::slots_motorCameraRotateRotating(bool bFlag)
{
	int nCurPosNo = m_pcTPLCComunication->m_sPLCStatus.nMotorCameraLifterCurPos;
	if (nCurPosNo > MotorCameraLifter_MAX_POS)
	{
		emit signals_addLogText(tr("Wrong M1 Pos Number %1!").arg(nCurPosNo));
	}
	if (MR_TRIG != m_pcTPLCComunication->m_sPLCModelPara.nTrigFlag[nCurPosNo - 1])
	{
		return;
	}
	//todo: add elements in m_sCamInfo & m_pGrabber after integration of MR Camera
	if (bFlag) 
	{
		m_pcImageProcessing->m_sCamInfo[2].bStartGrab = true;
		m_pcImageProcessing->m_pGrabber[2]->StartGrab();
	}
	else
	{
		m_pcImageProcessing->m_sCamInfo[2].bStartGrab = false;
		m_pcImageProcessing->m_pGrabber[2]->StopGrab();
	}
}

