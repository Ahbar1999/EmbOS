#ifndef _KERNEL_H_
#define _KERNEL_H_

typedef struct {
	void* sp;	// stack pointer
	uint32_t timeout;	// delay countdown
	/* other info */
} OSThread;

// typedef for pointer to a function of type: void xyz(void)
typedef void (*OSThreadHandler)();

// stack pointer and size for idle thread
void OS_init(void* stack, uint32_t stack_size);

// for handling idle condition/thread
// this function will be defined in the main.c file
void OS_onIdle(void);

// schedule context switch 
void OS_sched();

void OS_delay(uint32_t ticks);
/*
 * stkSto: memory location of private stack 
 * stkSize: size of the stack
 * */
void OSThread_start(
	OSThread *this,
	OSThreadHandler threadHandler,
	void *stkSto, uint32_t stkSize);

#endif
