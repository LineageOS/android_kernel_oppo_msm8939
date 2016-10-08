/*
 * yixue.ge add for oppo project
 */

#ifndef _OPPO_PROJECT_H_
#define _OPPO_PROJECT_H_

enum {
	HW_VERSION__UNKNOWN,
	HW_VERSION__10,		//1452mV
	HW_VERSION__11,		//1636 mV
	HW_VERSION__12,		//1224 mV
	HW_VERSION__13,		//900 mV
	HW_VERSION__14,		//720 mV
	HW_VERSION__15,
	HW_VERSION__16,
};

enum {
	RF_VERSION__UNKNOWN,
	RF_VERSION__11,
	RF_VERSION__12,
	RF_VERSION__13,
	RF_VERSION__21,
	RF_VERSION__22,
	RF_VERSION__23,
	RF_VERSION__31,
	RF_VERSION__32,
	RF_VERSION__33,
};

#define GET_PCB_VERSION() (get_PCB_Version())
#define GET_PCB_VERSION_STRING() (get_PCB_Version_String())

#define GET_MODEM_VERSION() (get_Modem_Version())
#define GET_OPERATOR_VERSION() (get_Operator_Version())

enum OPPO_PROJECT {
	OPPO_UNKNOWN = 0,
	OPPO_14005 = 14005,
	OPPO_15011 = 15011,
	OPPO_15018 = 15018,
	OPPO_15022 = 15022,
	OPPO_15109 = 15109,
};

enum OPPO_OPERATOR {
	OPERATOR_UNKNOWN		= 0,
	OPERATOR_OPEN_MARKET		= 1,
	OPERATOR_CHINA_MOBILE		= 2,
	OPERATOR_CHINA_UNICOM		= 3,
	OPERATOR_CHINA_TELECOM		= 4,
	OPERATOR_FOREIGN		= 5,
	OPERATOR_FOREIGN_WCDMA		= 6,
	OPERATOR_FOREIGN_RESERVED	= 7,
	OPERATOR_ALL_CHINA_CARRIER	= 8,
	OPERATOR_FOREIGN_4G 		= 101,
	OPERATOR_FOREIGN_3G 		= 100,
	OPERATOR_FOREIGN_TAIWAN		= 102,
	OPERATOR_FOREIGN_EUROPEAN	= 103,
	OPERATOR_FOREIGN_TELECOM	= 104,
	OPERATOR_FOREIGN_INDIA		= 105,
	OPERATOR_FOREIGN_ASIA		= 106,
};

typedef enum OPPO_PROJECT OPPO_PROJECT;

typedef struct {
	unsigned int	nProject;
	unsigned char	nModem;
	unsigned char	nOperator;
	unsigned char	nPCBVersion;
} ProjectInfoCDTType;

#ifdef CONFIG_MACH_OPPO
unsigned int init_project_version(void);
unsigned int get_project(void);
unsigned int is_project(OPPO_PROJECT project);
unsigned char get_PCB_Version(void);
unsigned char get_Modem_Version(void);
unsigned char get_Operator_Version(void);
#else
unsigned int init_project_version(void) { return 0; }
unsigned int get_project(void) { return 0; }
unsigned int is_project(OPPO_PROJECT project) { return 0; }
unsigned char get_PCB_Version(void) { return 0; }
unsigned char get_Modem_Version(void) { return 0; }
unsigned char get_Operator_Version(void) { return 0; }
#endif

#endif
