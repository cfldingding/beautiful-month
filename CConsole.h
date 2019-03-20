////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	CConsole.h
//
// summary:	Declares the console class. Main Frame Class, manage all the connection and normal issue
// 2017 Dec. 12th Yu Lei
// 
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <QObject>
#include <QTimer>
#include "InnerData.h"
#include "CWeight.h"
#include "CImagePorcessing.h"
#include "CTThickness.h"
#include "CTInner.h"
#include "CLogData.h"
#include "CTPLCComunication.h"
#include "CTIQScan.h"

typedef struct _SysOperatePara
{
	bool bCalib;											//标定标志位，影响检测线程
	bool bAutoCalib;										//自动标定标志位，20160112增加
	bool bAutoCaAdvanced;									//自动标定高级功能，20160125增加
	bool bShowMoldNumber;									

	bool bStartDetect;										//系统是否正在检测
	bool bLogoTimeout;										//logo加载时长
	QString strAppPath;										

	QTime startTime;
	int nTotalCount;					
	int nWeightCount;					
	int nDimensionCount;				
	int nThicknessCount;					
	int nInDiaCount;						
	int nNGTotal;							
	int nNGWeight;									
	int nNGDimention;					
	int nNGThickness;						
	int nNGInnerDia;							
	bool bManualCamera;						//当手动频闪时，不检测

	int nLoginLevel;						//0:普通 1：管理员 2：开发
	int bCHRInitiated;						
	int nCheckMode;

	s_ItemName sItemName; 
	s_ItemName sDefaultName;

	int nCurPosIdx;

	_SysOperatePara()
	{
		bCalib = false;	
		bAutoCalib = false;		
		bAutoCaAdvanced = false;	
		bStartDetect = false;
		bLogoTimeout = false;						
		bManualCamera = false;	
		bShowMoldNumber = false;
		nTotalCount = 0;					
		nWeightCount = 0;							
		nDimensionCount = 0;										
		nThicknessCount = 0;										
		nInDiaCount = 0;												
		nNGTotal = 0;													
		nNGWeight = 0;													
		nNGDimention = 0;											
		nNGThickness = 0;											
		nNGInnerDia = 0;													
		nLoginLevel = 0;
		bCHRInitiated = false;
		nCheckMode = 0;
		nCurPosIdx = 0;
	}
}s_SysOperatePara;

class COLDLAB;
class CDlgShowData;
class CDlgShowStatus;
class CConsole : public QObject
{
	Q_OBJECT

public:
	CConsole(QObject *parent);
	~CConsole();

	COLDLAB *m_coldLabFrame;
	s_SysConfigPara m_sysConfigPara;
	s_SysOperatePara m_sysOperaPara;

	s_CheckModel m_sCheckModelPara;			
	s_CheckModel m_sCheckModelPara1;
	s_CheckModel m_sCheckModelPara2;

	s_PlotRange m_sPlotRange;
	s_JudgePara m_sJudgePara;
	
	s_ExtResultVector m_sDataOfBottles;				//所有已测瓶子的数据容器
	QVector<s_JudgeIdx> vJudgeIdx;					//根据进瓶序号判断是否合格
	CTPLCComunication *m_pcTPLCComunication;
	CWeight *m_pcWeight;
	CImagePorcessing *m_pcImageProcessing;
	CThickness *m_pcThicknessProcessing;
	CTInner *m_pcInner;
	CInnerDiaRoThread *m_pcTInnerSimulation;	
	CTIQScan *m_pcIQScan;

	void Initiate();	
	void InitCameras();
	void InitCheck();
	void InitSerialPort();
	bool InitThickness();
	bool InitInner();
	void SetAlarmLight(int nCmd);
	void ReadMoldCode();
	bool JudgeWeight(s_JudgePara &sJudge, double dWeight);
	bool JudgeDimention(s_JudgePara & sJudge, s_ExtResultCamera &sExtCamera);
	bool JudgeThickness(s_JudgePara & sJudge, s_ExtResultThickness &sExtThickness);
	bool JudgeInnerDia(s_JudgePara & sJudge, s_ExtResultInner &sExtInner);
	bool UpdateModel(QString strModelName, s_PLCAutoUpg *sPLCModel, s_CheckModel *sCheckModel, s_PlotRange *sPlotRange, s_MotorCalibPara *sMotorCalibPara, s_CameraCalibrationPara &sCamCalibPara1, s_CameraCalibrationPara &sCamCalibPara2, s_InnerDiaCalibPara &sInnerDiaCalibPara);
	bool RecalculateModel(s_PLCAutoUpg *sPLCModel, s_CheckModel *sCheckModel, s_MotorCalibPara *sMotorCalibPara, s_CamCalibPara &sCamCalibPara1, s_CamCalibPara &sCamCalibPara2, s_InnerDiaCalibPara &sInnerDiaCalibPara, QString *strErrMsg = NULL);
	void SetPlotRange(QString strModelName, s_CheckModel &sCheckModel, s_PlotRange *sPlotRange);
	QVector<s_Measurement> GetMeasurements();
	s_Measurement AssignMeasurement(QString strName, double dValue, double dSetPointValue, s_Range sWarningRange, s_Range sErrorRange);
	void Start();
	void Stop();
	void Quit();

	QString GetInspectionName(EInspectionType eInsp, int indexWidget);


private:
	bool calculateMR(s_PLCAutoUpg *sPLCModelUpg, s_MotorCalibPara &sMotorCalibPara);
	bool calculateThickness(s_PLCAutoUpg *sPLCModelUpg, s_CheckModel &sCheckModel, s_MotorCalibPara *sMotorCalibPara);
	bool calculateCameraPo(s_PLCAutoUpg *sPLCModel, s_CheckModel *sCheckModel, s_CamCalibPara &sCamCalibPara1, s_CamCalibPara &sCamCalibPara2, QString *strErrMsg);
	s_ModelPosition getCameraPoInfo(s_CheckModel *sCheckModel, s_CamCalibPara &sCamCalibPara1, s_CamCalibPara &sCamCalibPara2, QString *strErrMsg);
	void getBodyParam(QVector<double> *vData, s_CheckModel &sCheckModel);
	void getFinishParam(QVector<double> *vData, int dSpan, s_CheckModel &sCheckModel);
	bool calculateInner(s_PLCAutoUpg *sPLCModelUpg, s_CheckModel *sCheckModel, s_InnerDiaCalibPara &sInnerDiaCalibPara);

	QVector<double> getBodyInspectionPositions(EInspectionType eInsp, s_CheckModel &sCheckModel);
	QString getInspectionName(EInspectionType eInsp, int indexWidget);

public slots:
	void slots_addLogText(QString str, EFileNameType eFileNameType);
	void slots_showMessageBox(QString strType, QString str);

private slots:
	void slots_cylinderArmGripper(bool bFlag);
	void slots_hasBottle(bool bFlag);
	void slots_outfeedConveyor(bool bFlag);
	void slots_trigCamera(bool bFlag, int nTrigCam);
	void slots_trigThickness(bool bFlag);
	void slots_trigInner(bool bFlag);
	void slots_trigPLCBusy(bool bFlag);
	void slots_motorCameraRotateRotating(bool bFlag);

signals:
	void signals_addLogText(QString str, EFileNameType eFileNameType = EFileNameType::Warning);
};
