#include "unp.h"


typedef enum 
{
	STOPPED,
	READ,
	WRITE,
	WAIT_CNT,
	SHUTDOWN
} ePortThreadState;


ePortThreadState eGetPortThreadState( int port_id );
void SetPortThreadState(int port_id, ePortThreadState eNewState);