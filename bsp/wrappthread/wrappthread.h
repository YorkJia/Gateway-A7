#ifndef __WRAP_PTHREAD_H
#define __WRAP_PTHREAD_H

#include <unp.h>

void
Pthread_create(pthread_t *tid, const pthread_attr_t *attr,
			void * (*func)(void *), void *arg);

void
Pthread_join(pthread_t tid, void **status);

void
Pthread_detach(pthread_t tid);


void
Pthread_mutex_init(pthread_mutex_t *mptr, pthread_mutexattr_t *attr);

void
Pthread_mutex_lock(pthread_mutex_t *mptr);

void
Pthread_mutex_unlock(pthread_mutex_t *mptr);


#endif
