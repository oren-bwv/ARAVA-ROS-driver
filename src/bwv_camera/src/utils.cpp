#include <unistd.h>		// usleep
#include <stdlib.h>		// malloc
#include <pthread.h>	// pthread_t, pthread_attr_init, pthread_setschedparam

#include "utils.h"		// implementation of utils.h functions

typedef void* thread_func_t(void*);

void* CreateThread(void* function, void* args, int* result){
	pthread_t* handle = (pthread_t*)malloc(sizeof(pthread_t));
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 10*1024*1024);

//	int ret = pthread_create(handle, &attr, (void*)function, args);
	int ret = pthread_create(handle, &attr,
			reinterpret_cast<thread_func_t*>(function), args);

	pthread_attr_destroy(&attr);

	if (result != NULL)
	{
		(*result) = ret;
	}

	struct sched_param params;
    params.sched_priority = 0;
	pthread_setschedparam((*handle),SCHED_OTHER, &params);

	return (handle);
}

void Sleep(int ms)
{
	usleep(ms * 1000);
}
