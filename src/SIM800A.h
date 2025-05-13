#ifndef __SIM900A_H__
#define __SIM900A_H__

#include <stdint.h> // 替换 STM32 的 "sys.h" 为标准类型头文件
//////////////////////////////////////////////////////////////////////////////////
#define SIM_OK 0
#define SIM_COMMUNTION_ERR 0xff
#define SIM_CPIN_ERR 0xfe
#define SIM_CREG_FAIL 0xfd
#define SIM_MAKE_CALL_ERR 0xfc
#define SIM_ATA_ERR 0xfb

#define SIM_CMGF_ERR 0xfa
#define SIM_CSCS_ERR 0xf9
#define SIM_CSCA_ERR 0xf8
#define SIM_CSMP_ERR 0xf7
#define SIM_CMGS_ERR 0xf6
#define SIM_CMGS_SEND_FAIL 0xf5

// 使用标准类型替换 STM32 的 u8
extern uint8_t Flag_Rec_Call;

extern uint8_t SIM900_CSQ[3];
extern uint8_t GSM_Dect(void);
extern uint8_t sim900a_send_chmessage_zc(const char *number, const char *content);
extern uint8_t sim900a_send_chmessage_zc_01(const char *number, const char *content);
extern uint8_t SIM_MAKE_CALL(const char *number) ;

#endif