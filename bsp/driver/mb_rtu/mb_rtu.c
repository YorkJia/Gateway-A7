#include "mb_rtu.h"
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "modbus.h"
#include "portevent.h"
#include "unp.h"

/*
void *mb_rtu_master_thread(void *arg)
{

	DriverThreadPara_TypeDef *pdev;
	Modbus_ReadRegsTypeDef *pread_para;
	Modbus_WriteRegsTypeDef *pwrite_para;
	modbus_t *ctx = NULL;
	int rc, cnt = 0;
	int i;
	//int j;
	struct timeval xTimeCur,xTimeLast;
	pdev = (DriverThreadPara_TypeDef *)arg;
	pread_para = (Modbus_ReadRegsTypeDef *)pdev->read_data;
    pwrite_para = (Modbus_WriteRegsTypeDef *)pdev->write_data;

	Pthread_detach(pthread_self());
	
	//init the libmodbus
	ctx = modbus_new_rtu("/dev/ttySP0", 9600, 'N', 8, 1);
	if(ctx == NULL){
		err_quit("init the libmodbus error");
	}

	modbus_set_debug(ctx, 0);  //enable the debug info
	modbus_set_slave(ctx, 1);  //set slave id

	//connect the device
	if(modbus_connect(ctx) == -1){
		fprintf(stderr, "connect dev failed:%s\n", modbus_strerror(errno));
		modbus_close(ctx);
		modbus_free(ctx);
		err_quit("libmodbus connect error");
	}
	while(1){
		switch(eGetPort1ThreadState()){
		case STOPPED:
	
			break;
		case READ:
			for(i = 0; i < 5; i++){
				modbus_set_slave(ctx, pread_para[i].slave_id);
				rc = modbus_read_registers(ctx, pread_para[i].start_addr,
									pread_para[i].unit_len, pread_para[i].rx_data);
				if(rc == -1){
					fprintf(stderr, "%s\n", modbus_strerror(errno));
					modbus_close(ctx);
					modbus_free(ctx);
					err_quit("libmodbus read regs error");
				
				}
			}
			if(gettimeofday(&xTimeLast, NULL) == 0)
				SetPort1ThreadState( WAIT_CNT );
			break;
		case WRITE:
			modbus_write_registers(ctx, pwrite_para->start_addr, pwrite_para->unit_len, (const uint16_t *)pwrite_para->valid_data);
			cnt = 0;
			SetPort1ThreadState( READ );
			break;
		case WAIT_CNT:
			if(gettimeofday(&xTimeCur, NULL) == 0){
				if(xTimeCur.tv_sec > xTimeLast.tv_sec){
					if(xTimeCur.tv_sec - xTimeLast.tv_sec >= 2)
						SetPort1ThreadState( READ );
				}
			}
			break;
		case SHUTDOWN:
			printf("shutdown .\n");
			modbus_close(ctx);
			modbus_free(ctx);
			err_quit("port1 shutdown");
			break;
		}

		sleep(1);
	}

	modbus_close(ctx);
	modbus_free(ctx);

	err_quit("port1 thread quit.");
}
*/

void *mb_rtu_master_thread(void *arg)
{

	DriverThreadPara_TypeDef *pdev;
	Modbus_ReadRegsTypeDef *pread_para;
	Modbus_WriteRegsTypeDef *pwrite_para;
	modbus_t *ctx = NULL;
	char cSerialName[15];
	char msg[60];
	int rc, cnt = 0;
	int i;
	//int j;
	struct timeval xTimeCur,xTimeLast;

	pdev = (DriverThreadPara_TypeDef *)arg;
	pread_para = (Modbus_ReadRegsTypeDef *)pdev->read_data;
    pwrite_para = (Modbus_WriteRegsTypeDef *)pdev->write_data;

	Pthread_detach(pthread_self());
	/* print the arg */
	
	//init the libmodbus
	if((pdev->port_id == 0) || (pdev->port_id > 5))
		err_quit("port id error(port1~port5)");

	sprintf(cSerialName, "/dev/ttySP%d", pdev->port_id - 1);
	ctx = modbus_new_rtu(cSerialName, 9600, 'N', 8, 1);
	if(ctx == NULL){
		sprintf(msg, "port%d init the libmodbus error",pdev->port_id);
		err_quit(msg);
	}

	modbus_set_debug(ctx, 0);  //enable the debug info
	modbus_set_slave(ctx, 1);  //set slave id

	//connect the device
	if(modbus_connect(ctx) == -1){
		fprintf(stderr, "port%d connect dev failed:%s\n",pdev->port_id,modbus_strerror(errno));
		modbus_close(ctx);
		modbus_free(ctx);
		sprintf(msg, "port%d connect error.",pdev->port_id);
		err_quit(msg);
	}
	while(1){

		switch(eGetPortThreadState( pdev->port_id )){
		case STOPPED:
			break;
		case READ:
			for(i = 0; i < pdev->dev_num; i++){
				modbus_set_slave(ctx, pread_para[i].slave_id);
				rc = modbus_read_registers(ctx, pread_para[i].start_addr,
									pread_para[i].unit_len, pread_para[i].rx_data);
				if(rc == -1){
					fprintf(stderr, "port%d%s\n", pdev->port_id, modbus_strerror(errno));
					modbus_close(ctx);
					modbus_free(ctx);
					sprintf(msg, "port%d read regs error.", pdev->port_id);
					err_quit(msg);
				
				}	
			}
			if(gettimeofday(&xTimeLast, NULL) == 0)
				SetPortThreadState(pdev->port_id, WAIT_CNT);
			break;
		case WRITE:
			break;
		case WAIT_CNT:
			if(gettimeofday(&xTimeCur, NULL) == 0){
				if(xTimeCur.tv_sec > xTimeLast.tv_sec){
					if(xTimeCur.tv_sec - xTimeLast.tv_sec >= 2)
						SetPortThreadState(pdev->port_id, READ );
				}
			}
			break;
		case SHUTDOWN:
			break;
		}

	}

	modbus_close(ctx);
	modbus_free(ctx);

	sprintf(msg, "port%d thread quit.", pdev->port_id);
	err_quit(msg);
}

