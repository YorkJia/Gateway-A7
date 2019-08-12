#include "portevent.h"
#include "pthread.h"

static ePortThreadState ePort1ThreadState = STOPPED;
static ePortThreadState ePort2ThreadState = STOPPED;

static pthread_mutex_t xPort1Lock = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t xPort2Lock = PTHREAD_MUTEX_INITIALIZER;


ePortThreadState eGetPort1ThreadState( void )
{
	ePortThreadState eCurState;
	( void )pthread_mutex_lock( &xPort1Lock );
	eCurState = ePort1ThreadState;
	( void )pthread_mutex_unlock( &xPort1Lock );

	return eCurState;
}

void SetPort1ThreadState(ePortThreadState eNewState)
{
	( void )pthread_mutex_lock( &xPort1Lock );
	ePort1ThreadState = eNewState;
	( void )pthread_mutex_unlock( &xPort1Lock );
} 


ePortThreadState eGetPort2ThreadState( void )
{
	ePortThreadState eCurState;
	( void )pthread_mutex_lock( &xPort2Lock );
	eCurState = ePort2ThreadState;
	( void )pthread_mutex_unlock( &xPort2Lock );

	return eCurState;
}

void SetPort2ThreadState(ePortThreadState eNewState)
{
	( void )pthread_mutex_lock( &xPort2Lock );
	ePort2ThreadState = eNewState;
	( void )pthread_mutex_unlock( &xPort2Lock );
} 

ePortThreadState eGetPortThreadState( int port_id )
{
	switch( port_id ){
	case 1:
		return eGetPort1ThreadState();
		break;
	case 2:
		return eGetPort2ThreadState();
		break;
	default:
		return STOPPED;
		break;
	}
}

void SetPortThreadState(int port_id, ePortThreadState eNewState)
{
	switch( port_id ){
	case 1:
		return SetPort1ThreadState(eNewState);
		break;
	case 2:
		return SetPort2ThreadState(eNewState);
		break;
	}
} 


