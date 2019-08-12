#ifndef __BSP_H
#define __BSP_H

#include <wraperror.h>
//#include <sqlite3.h>
//#include <wrapsqlite.h>
#include <wrappthread.h>
#include "unp.h"

typedef int u16;
typedef unsigned char u8;




typedef struct member{
	int index;
        char name[32];
	char driver[16];
	int rem_para1;
	char rem_para2[16];
	char rem_para3[16];
        char loc_para1[16];
	char loc_para2[16];
	int com_index;
	int data_type;
	char rw_type[3];
	char unit_para1[3];
	char unit_para2[3];
	int data_para;
	int start_addr;
	int unit_len;
	int data_hold;
	int scan_cycle;
	int log; 
	int repeat;
} DEV_PARA;


typedef struct rtu{
	u8 slave_id;
	u16 start_addr;
	u16 unit_len;
	u16 cycle;
	uint16_t *rx_buf;
} _RTU_DRIVER;


typedef struct thread_para{
	u8 dev_num;
	_RTU_DRIVER **rtu_data;
} _THREAD_PARA;


typedef struct 
{
	int index;
    char name[32];
	char driver[16];
	int rem_para1;
	char rem_para2[16];
	char rem_para3[16];
    char loc_para1[16];
	char loc_para2[16];
	int com_index;
	int data_type;
	char rw_type[3];
	char unit_para1[3];
	char unit_para2[3];
	int data_para;
	int start_addr;
	int unit_len;
	int data_hold;
	int scan_cycle;
	int log; 
	int repeat;


	void *backend_data;
}Driver_TypeDef;


typedef struct 
{
	int slave_id;
	int data_type;
	int data_para;
	int start_addr;
	int unit_len;
	int data_hold;
	int scan_cycle;
	int log;
	int repeat;

	uint16_t *data;
}Port_modbus_ParaType;



typedef struct 
{
	int slave_id;
	int data_type;
	int data_para;
	int start_addr;
	int unit_len;
	int data_hold;
	int scan_cycle;
	int log;
	int repeat;
	uint16_t *rx_data;
} Modbus_ReadRegsTypeDef;

typedef struct 
{
	int slave_id;
	int data_type;
	int data_para;
	int start_addr;
	int unit_len;
	uint16_t valid_data[128];
} Modbus_WriteRegsTypeDef;




typedef struct 
{
	int port_id;
	int dev_num;
	void *read_data;
	void *write_data;
}DriverThreadPara_TypeDef;



#endif
