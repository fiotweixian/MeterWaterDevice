#ifndef __HC32_BC35_H__
#define __HC32_BC35_H__
#include "gpio.h"
#include "uart.h"
#include "Common.h"
#include "rtc.h"
#include "hc32_debug.h"
#include "hc32_rtc.h"
#include "hc32_led.h"
#include "lpm.h"


#define HC32_BC28_SUPPORTS
//#define HC32_L161_SUPPORTS

#define Bc35UartRecvBufMaxSize 768

#define HC_BC35_OCEANCONNECT_SUPPORT
#define HC_BC35_NEIGHBOR_SUPPORT

#define  HC32_UART_NB_SUPPORTS
#define  HC_BC35_OCEANCONNECT_SERVER_IP_SUPPORT

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif	

#define USER_NBPWR_PORT        GpioPortC
#define USER_NBPWR_PIN         GpioPin11	
#define HC32_NBPWR_Low()       {Gpio_WriteOutputIO(USER_NBPWR_PORT,USER_NBPWR_PIN,0);}
#define HC32_NBPWR_Hight()     {Gpio_WriteOutputIO(USER_NBPWR_PORT,USER_NBPWR_PIN,1);}	
#define USER_NBRST_PORT        GpioPortC
#define USER_NBRST_PIN         GpioPin10	
#define HC32_NBRST_Low()       {Gpio_WriteOutputIO(USER_NBRST_PORT,USER_NBRST_PIN,0);}
#define HC32_NBRST_Hight()     {Gpio_WriteOutputIO(USER_NBRST_PORT,USER_NBRST_PIN,1);}
#define USER_NBWAKE_PORT       GpioPortD
#define USER_NBWAKE_PIN        GpioPin3	
#define HC32_NBWAKE_Low()      {Gpio_WriteOutputIO(USER_NBWAKE_PORT,USER_NBWAKE_PIN,0);}
#define HC32_NBWAKE_Hight()    {Gpio_WriteOutputIO(USER_NBWAKE_PORT,USER_NBWAKE_PIN,1);}

#define HC32_UART_NB_SUPPORTS	
	
#define OceanFlatSendMessId 	0
#define OceanFlatRecMessId  	1	

#define OceanFlatSendDataHeader 0x68
	
#define HcArraySize(Sz) (sizeof(Sz)/sizeof(Sz[0]))
	
#define BC35_OK_REPLY "OK"
	
#define BC35_ERROR_REPLY "ERROR"

#define BC35_AT "AT\r\n"

#define BC35_VERSION_READ "AT+CGMR\r\n"
#define BC35_VERSION_REPLY "+CGMR:"
	
#define BC35_IMEI_TEST "AT+CGSN=?\r\n"
#define BC35_IMEI_READ "AT+CGSN=1\r\n"
#define BC35_IMEI_REPLY "+CGSN:"	
	
#define BC35_SLEEP_WRITE(str) "AT+ECSLEEP="str"\r\n"

#define BC35_QCCID_READ "AT+ECICCID\r\n"
#define BC35_QCCID_REPLY "+ECICCID:"	

#define BC35_BCINFO_READ "AT+ECBCINFO\r\n"
#define BC35_BCINFO_REPLY0 "+ECBCINFOSC:"
#define BC35_BCINFO_REPLY1 "+ECBCINFONC:"

#define BC35_DATE_TIME_TEST "AT+CCLK=?\r\n"
#define BC35_DATE_TIME_READ "AT+CCLK?\r\n"
#define BC35_DATE_TIME_REPLY "+CCLK:"

#define BC35_IMSI_READ "AT+CIMI\r\n"

#define BC35_NNMI_READ "AT+CMGF?\r\n"
#define BC35_NNMI_WRITE(str) "AT+CMGF="str"\r\n"
#define BC35_NNMI_TWO			2
#define BC35_NNMI_ONE			1
#define BC35_NNMI_ZERO		0

#define BC35_CPSMS_TEST "AT+CPSMS=?\r\n"
#define BC35_CPSMS_WRITE(str) "AT+CPSMS="str"\r\n"
#define BC35_CPSMS_READ "AT+CPSMS?\r\n"
#define BC35_CPSMS_REPLY "+CPSMS:"
#define BC35_PSM_MODE_ENABLE 		1
#define BC35_PSM_MODE_DISABLE 	0

#define BC35_EDRX_READ "AT+CEDRXS?\r\n"
#define BC35_EDRX_WRITE(mod, adt, value) "AT+CEDRXS="mod","adt","value"\r\n"
#define BC35_EDRX_REPLY "+CEDRXS:"
#define BC35_EDRX_NBS1 				5
#define BC35_EDRX_DISABLE   	0
#define BC35_EDRX_ENABLE  		1
#define BC35_EDRX_20P48SEC  	1
#define BC35_EDRX_40P96SEC  	2
#define BC35_EDRX_81P92SEC  	3
#define BC35_EDRX_163P84SEC  	4
#define BC35_EDRX_327P68SEC  	5
#define BC35_EDRX_655P36SEC  	6
//....



#define BC35_CFUN_READ "AT+CFUN?\r\n"
#define BC35_CFUN_WRITE(str) "AT+CFUN="str"\r\n"
#define BC35_CFUN_REPLY "+CFUN:"

#define BC35_REBOOT_CRL "AT+ECRST\r\n"
#define BC35_REBOOT_REPLY "ECRDY"

#define BC35_CGATT_READ "AT+CGATT?\r\n"
#define BC35_CGATT_WRITE(str) "AT+CGATT="str"\r\n"
#define BC35_CGATT_REPLY "+CGATT:"

#define BC35_BAND_READ "AT+ECBAND?\r\n"
#define BC35_BAND_WRITE(str) "AT+ECBAND="str"\r\n"
#define BC35_BAND_REPLY "+ECBAND:"

#define BC35_MODIFY_IMEI(str) "AT+ECCGSN=\"IMEI\","str"\r\n"

#define BC35_CSQ_READ "AT+CSQ\r\n"
#define BC35_CSQ_REPLY "+CSQ:"

#define BC35_CEREG_READ "AT+CEREG?\r\n"
#define BC35_CEREG_REPLY "+CEREG:"


#define BC35_NUESTATS_READ "AT+ECSTATUS\r\n"
#define BC35_NUESTATS_REPLYPHY "+ECSTATUS: PHY, "
#define BC35_NUESTATS_REPLYL2 "+ECSTATUS: L2, "
#define BC35_NUESTATS_REPLYRRC "+ECSTATUS: RRC, "
#define BC35_NUESTATS_REPLYEMM "+ECSTATUS: EMM, "
#define BC35_NUESTATS_REPLYPLMN "+ECSTATUS: PLMN, "
#define BC35_NUESTATS_REPLYESM "+ECSTATUS: ESM, "
#define BC35_NUESTATS_REPLYCCM "+ECSTATUS: CCM, "

#define BC35_NNMI_REPLY "+CMGF:"
#define BC35_NMGR_READ "AT+NMGR\r\n"
#define BC35_NNMI_MESREPLY "+CTM2MRECV: DADA2BAC"
#define BC35_NNMI_RESPONSE "DADA2BAC"

#define BC35_NCDP_READ "AT+CTM2MSETPM?\r"
#define BC35_NCDP_REPLY "+CTM2MSETPM:"

#define BC35_CTM2MREG_READ "AT+CTM2MREG\r"
#define BC35_CTM2MEREG_READ "AT+CTM2MDEREG\r"
#define BC35_CTM2MUPDATE_READ "AT+CTM2MUPDATE\r"
#define BC35_CTM2MSEND_REPLY "+CTM2M: send,"
#define BC35_CTM2EREG_REPLY "+CTM2M: dereg,0\r\n"
#define BC35_CTM2MREG_REPLY "+CTM2M: reg,0\r\n\r\n+CTM2M: obsrv,0\r\n"


#ifdef HC32_BC28_SUPPORTS

#define BC28_OK_REPLY "OK"

#define BC28_REBOOT_CRL "AT+NRB\r\n"
#define BC28_REBOOT_REPLY "REBOOTING"

#define BC28_VERSION_READ "ATI\r\n"
#define BC28_VERSION_REPLY "Revision:"

#define BC28_QCCID_READ "AT+NCCID\r\n"
#define BC28_QCCID_REPLY "+NCCID:"	

#define BC28_BAND_READ "AT+NBAND?\r\n"
#define BC28_BAND_WRITE(str) "AT+NBAND="str"\r\n"
#define BC28_BAND_REPLY "+NBAND:"

#define BC28_NUESTATS_READ "AT+NUESTATS\r\n"

#define BC28_NCDP_READ "AT+NCDP?\r\n"
#define BC28_NCDP_REPLY "+NCDP:"

#define BC28_NNMI_READ "AT+NNMI?\r\n"
#define BC28_NNMI_WRITE(str) "AT+NNMI="str"\r\n"
#define BC28_NNMI_TWO		2
#define BC28_NNMI_ONE		1
#define BC28_NNMI_ZERO		0

#define BC28_NNMI_REPLY "+NNMI:"
#define BC28_NMGR_READ "AT+NMGR\r\n"
#define BC28_NNMI_MESREPLY "+NNMI"

#endif

//--------------------------------------------
// | 0x68 | 0x55 | Control | ... | CRC | 0x16
//--------------------------------------------
#define CMD_SYNC_HEADER    				68
#define CMD_SYNC_VERSION1  				1
#define CMD_SYNC_CONTROLCODE        	9
#define CMD_SYNC_NOTSECRECT         	0	
#define CMD_SYNC_PULSSECRECT        	1	
#define CMD_SYNC_END       				16

//阀门开关
#define CMD_CODE_VALVE					1000
//上报频率
#define CMD_CODE_UPDATAFRE				1001
//采集频率
#define CMD_CODE_GATHERFRE				1002
//过流告警
#define CMD_CODE_OVERCURRENTWARN		1003
//过流时间
#define CMD_CODE_OVERCURRENTTIME		1004
//反向过流告警
#define CMD_CODE_CYOVERCURRENTWARN		1005
//反向过流时间
#define CMD_CODE_CYOVERCURRENTTIME		1006
//修改服务器
#define CMD_CODE_MODIFYSERVER			1007
//修改系统时间
#define CMD_CODE_MODIFYSYSTIME			1008
//电池阈值
#define CMD_CODE_BATTERYTHRELD			1009
//修改APN
#define CMD_CODE_MODIFYAPN				1010
//修改水表编号
#define CMD_CODE_MODIFYWNUM				1011
//修改水表水量
#define CMD_CODE_WATERAMOUNT			1012
//设置立即告警
#define CMD_CODE_IMMEDWARN				1013
//设置密集周期上报
#define CMD_CODE_DENSEPERIOD			1014



//向终端发送抄表数据
#define DEV_CCODE_METERDATA 		1   
//向终端发送图片数据
#define DEV_CCODE_PHOTODATA 		2  
//向终端发送参数数据
#define DEV_CCODE_PARAMETERDATA 	3 
//向终端发送及时预警
#define DEV_CCODE_ALARMDATA 		4 
//向终端发送重发抄表数据
#define DEV_CCODE_AGAINMETERDATA 	5
//向终端发送重发图片数据
#define DEV_CCODE_AGAINPHOTODATA 	6 
//平台向终端发送命令
#define DEV_CCODE_FLATRECDATA 		9 
//...

//正常
#define DEV_ECODE_NORMAL 				1
//电池异常
#define DEV_ECODE_BATTERY 				2
//采集异常
#define DEV_ECODE_GAINDATA 				3
//破坏设备
#define DEV_ECODE_DEVDESTROY 			4
//阀门异常
#define DEV_ECODE_VALVA 				5	
//过流异常
#define DEV_ECODE_OVERCURRENT 			6
//反流异常
#define DEV_ECODE_CONCURRENT 			7

extern int gBc35UserQlwConnectStatus;

extern int gBc35UserWaitResponse;

extern char gBc35UserGetDataStatus;

extern char gBc35UserLastSec;

extern char gBc35UserGetDataError;

extern char u8Nb35SleepStatus;

extern char gBc35UserSendDataRepetition;

extern char gBc35UserSendDataStatus;

extern boolean_t gBc35UserImmediatelyWarn; 

extern boolean_t gNormalOverCurrentWarn;

extern uint32_t gBc35UserOverCurrentTime;

extern uint32_t gBc35UserConCurrentTime;

extern uint32_t gBc35UserSendDataFrequency;

extern char gBc28UserWaterDevNum[4];


typedef enum{
    BC35STATUS_NONE = -1,
    BC35STATUS_BUSY,
    BC35STATUS_AT,
    BC35STATUS_VERSION,
	BC35STATUS_REBOOT,
	BC35STATUS_BAND,
    BC35STATUS_DATATIME,
    BC35STATUS_NUESTATUS,
    BC35STATUS_QCCID,
    BC35STATUS_QIMEI,
    BC35STATUS_QIMSI,
    BC35STATUS_CGATT,
    BC35STATUS_CSQ,
    BC35STATUS_CEREG,
	BC35STATUS_SETCGATT,
	BC35STATUS_EDRX,
	BC35STATUS_SETEDRX,
	BC35STATUS_CPSMS,
	BC35STATUS_SETCPSMS,
	BC35STATUS_CFUN,
	BC35STATUS_SETCFUN,
	BC35STATUS_NNMI,
#ifdef HC_BC35_OCEANCONNECT_SUPPORT
    BC35STATUS_NRB,
    BC35STATUS_NCDP,
	BC35STATUS_CTM2MREG,
	BC35STATUS_CTM2MUPDATE,
    BC35STATUS_NMGS,
	BC35STATUS_CTM2MDEREG,
    BC35STATUS_NMGR,
#endif
    BC35STATUS_TOTAL_NUM
} Enum_BC35STATUS;


typedef struct _BC35_Qimei {
    char imei[16];
    char update;
} BC35_Qimei_T;

typedef struct _BC35_Qccid {
    char ccid[23];
    char update;
} BC35_Qccid_T;

typedef struct _BC35_Qimsi {
    char imsi[16];
    char update;
} BC35_Qimsi_T;

typedef struct _BC35_Version {
    char ver[20];
	char update;
} BC35_Version_T;

#ifdef HC_BC35_NEIGHBOR_SUPPORT
typedef struct _BC35_Neighbor{
    unsigned int Earfcn;
    unsigned short Pci;
    short Rsrp;
    short Rsrq;	
} BC35_Neighbor_T;
typedef struct _BC35_BCINFO{
    unsigned char NeighborNo;
    BC35_Neighbor_T Neighbor[10];
} BC35_BcInfo_T;
#endif
typedef struct _BC35_Band {
    char band[20];
	char update;
} BC35_Band_T;

typedef struct _BC35_Nngr {
    char len[3];
    char rec[128];
} BC35_Nngr_T;
typedef struct _BC35_Serv {
	short ippart1;
	short ippart2;
	short ippart3;
	short ippart4;
	short ipport;
	int lifetime;
	short instance;
} BC35_Serv_T;
typedef struct _BC35_DateTime {
    short year;
    char mouth;
    char day;
    char hour;
    char min;
    char sec;
	char zone;
    char update;
} BC35_DateTime_T;
typedef struct _BC35_NueStats
{
	char DlEarfcn[10];
	char UlEarfcn[10];
	char PCI[10];
	char Band[10];
	char RSRP[10];
	char RSRQ[10];
	char SNR[10];
	char CeLevel[10];
	char DlBler[10];
	char UlBler[10];
	char DataInactTimerS[10];
	char RetxBSRTimerP[10];
	char NBMode[16];

} BC35_NueStats_T;
//PARSER PROCEDURE
typedef enum
{
    CMD_PARSER_STATE_Header  = 0,
    CMD_PARSER_STATE_Version,
    CMD_PARSER_STATE_Control,
    CMD_PARSER_STATE_Secret,
    CMD_PARSER_STATE_Lenth,
    CMD_PARSER_STATE_Data,
    CMD_PARSER_STATE_CRC,
    CMD_PARSER_STATE_End,
	CMD_PARSER_STATE_Error,
	CMD_PARSER_STATE_Ok,
} CMD_READ_STATE_E;

typedef struct{
	short ServerPast1;
	short ServerPast2;
	short ServerPast3;
	short ServerPast4;
	short ServerPort;	
} rec_server_T;
typedef struct{
	short Year;
	char Month;
	char Day;
	char Hour;
	char Min;	
	char Sec;
} rec_time_T;
typedef struct{
	short Year;
	char Month;
	char Day;
	char Hour;
	char Min;	
	char Sec;
	char PriodStartFlag;	
} BC28_period_T;

extern BC28_period_T gBc28period;

typedef struct{
	short ControlNum;
	short Valve;
	short	SdataFrequency;
	short	GatherFrequency;
	short OvercurrentWarn;
	short	OvercurrentTime;
	short	ConOvercurrentWarn;
	short	ConOvercurrentTime;
	rec_server_T Server;
	rec_time_T Clock;
	short Battery;
} ctrl_cmd_rec_t;
	
typedef struct _BC35_Cmd_Parser_
{
	uint8_t        					Header;
	uint8_t 								AgreementVersion;
	uint8_t                 ControlCode;
	uint8_t                 SecretFlag;
	uint16_t                DataLenth;
	ctrl_cmd_rec_t					Data;
	uint16_t                CRCcheckH;
	uint16_t                CRCcheckL;
	uint8_t									End;
} BC35_Cmd_Parser_T;
typedef struct _BC35_Dat_Send_{
	char Header[2];
	char Imei[16];
	char VerCode[1];
	char ConCode[1];
	char Mid[11];
	char SNR[10];	
	char RSRP[10];
	char PCI[10];
	char ECL[10];
	char ReportTime[6];
	char DevVerCode[1];
	char DevId[2];
	char SecFlag[2];
	char Count[2];
	char Len[2];	
	char Ecode[1];
	char Valve[1];
	char WaterNum[8];
	char IntervalFlow[8];
	char InverseNum[8];
	char AcquisitionTime[6];
	char Vol[4];
	char DevDestroy[1];
	char Temperature[4];
	char Press[4];
	char Ph[4];
	char Chlorine[4];
	char Turbidity[4];
	char CRC[2];
	char End[2];
} BC35_Dat_Send;
#ifdef HC32_BC28_SUPPORTS
typedef struct _BC28_NueStats
{

	char Signal_power[10];
	char Total_power[10];
	char TX_power[10];
	char TX_time[10];
	char RX_time[10];
	char SCell_ID[10];
	char ECL[10];
	char SNR[10];
	char EARFCN[10];
	char PCI[10];
	char RSRQ[10];
} BC28_NueStats_T;
#endif

void HC32_BC35_GPIO(void);
void HC32_TXRX_Out(void);
void HC32_BC35_Init(void);
void HC32_BC35_Deinit(void);
void Bc35UartUserHandle(void);
void Bc35UserExceptionHandle(void);
void Bc35UserLogMsgParser(void);
static void Bc35UserUartMsgSend(void);
static void Bc35UserStatusInit(void);
static void Bc26UserQlwDataRecv(void);
extern int Bc35UserModifyIMEISend(char *str);
extern int Bc35UserEnterSleep(uint8_t mode_t);
int Bc35UserQlwSendDataSend(uint8_t mesg, uint8_t ctrlcode, uint8_t ecode, uint8_t secret);
#ifdef __cplusplus
}
#endif

#endif /* __HC32_BC35_H__ */
