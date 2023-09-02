#include <stdint.h>
#include "kernel.h"

// system handler priority register 3 
#define SHPR3 0xE000ED20
// interrupt control and state register
#define ICSR 0xE000ED04
#define PENDSVSET 28

OSThread* volatile OS_curr;	/* current running thread	*/
OSThread* volatile OS_next;	/* scheduled to run thread */

// array of threads(including idle thread at index 0)
OSThread* OS_threads[32 + 1];
// count of total threads active 
uint8_t OS_thread_num; 
// index of the thread to run
uint8_t i_thread;

// bitset for thread's(except idle thread) ready flag 
uint32_t OS_ready_set;

// its stack will be declared in the main file
OSThread idle_thread;

void idle_main(void) {
	while(1) {
		// defined in main.c 
		OS_onIdle();
	}
}

void OS_init(void* stack, uint32_t stack_size) {
	// set PendSV to lowest priority so that it is the last interrupt to be serviced
	// Refer: Page No. 61: STM32F0 Cortex-M programming manual 
	*((uint32_t volatile *)SHPR3) = (0x00000000 | 0xF << 20);
	
	// start idle thread
	OSThread_start(&idle_thread,
					idle_main,
					stack,
					stack_size);			
}

void OS_sched(void) {
	if (OS_ready_set == 0U) {
		// no thread is ready
		// schedule idle thread	
		i_thread = 0;
	} else {
		// there is a thread thats ready!
		// find a ready thread
		do {
			i_thread++;
			// wrap around, starting at index 1  
			if (i_thread == OS_thread_num) i_thread = 1;
		//	ith thread has (i - 1)th bit in the ready bit set because there is no bit for idle thread	
		} while(((1U << (i_thread - 1U)) & OS_ready_set) == 0);
		
	}
	
	// const pointer 
	OSThread* const next = OS_threads[i_thread];
	// update counter
	// ROUND ROBIN: if (i_thread >= OS_thread_num) i_thread = 0;
	if (next != OS_curr) {
		OS_next = next;	
		// set PendSV pending for context switching
		*((volatile uint32_t*)ICSR) |= 1 << PENDSVSET;
	}
}

// for delaying a thread by 'ticks'
void OS_delay(uint32_t ticks) {
	if (i_thread == 0) return;	
	__asm volatile("CPSID 	i");
	
	OS_curr->timeout = ticks;	
	// clear ready bit for the current thread i.e. mark it busy
	OS_ready_set &= ~(1U << (i_thread - 1U));
	// switch context away from this thread since it just got a timeout
	OS_sched();
	
	__asm volatile("CPSIE 	i");
	// now the context switch will be performed 
}

// for updating timeout counter of the threads
// this function will be called from systick handler
// so disabling threads and calling OS_sched is not required
void OS_tick(uint32_t ticks) {
	uint8_t i;
	for (i = 1; i < OS_thread_num; i++) {
		if (OS_threads[i]->timeout != 0) {
			(OS_threads[i]->timeout)--;
			// mark the thread ready
			if (OS_threads[i]->timeout == 0) OS_ready_set |= (1U << (i - 1));
		}	
	} 
}

void OSThread_start (
	OSThread *this,
	OSThreadHandler threadHandler,
	void *stkSto, uint32_t stkSize)
{
	// stack grows from the top so we add the stack size to the stack start addr to get the addr of the top
	uint32_t *sp = (uint32_t*)((((uint32_t)stkSto + stkSize) / 8) * 8);
	uint32_t *stk_limit;	
	
	/* only PSR(thumb bit = 1) and PC(return addr) register have special value
	 * rest of the registers are set arbitrary values, we have chosen to set them
	 * to their(register) serial numbers
	 * 	xPSR
	 *	PC
	 *	LR
	 *	R12
	 *	R3
	 *	R2
	 *	R1
	 *	R0
	 */
	// pre decrement because sp always points to the last used address rather than the first unused addr 
	*(--sp) = (1U << 24);	/* xPSR */
	*(--sp) = (uint32_t)threadHandler;	/* PC */
	*(--sp) = 0x0000000EU;	/* LR(arbitrary value), LR is used by the CPU only to handle exception returns */
	*(--sp) = 0x0000000CU;	/* General purpose registers here onwards */
	*(--sp) = 0x00000003U;
	*(--sp) = 0x00000002U;
	*(--sp) = 0x00000001U;
	*(--sp) = 0x00000000U;
	// additionally save R7-R4
	*(--sp) = 0x00000007U;
	*(--sp) = 0x00000006U;
	*(--sp) = 0x00000005U;
	*(--sp) = 0x00000004U;	
	
	// save stack pointer(top) of this stack frame
	this->sp = sp;
	
	// round up the stack boundary to 8 byte
	stk_limit = (uint32_t*)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

	// pad unused part of the stack
	for (sp = sp - 1U; sp >= stk_limit; --sp) {
		*sp = 0x00ABCDEF;
	}

	// save the thread
	OS_threads[OS_thread_num] = this;
	// mark it ready	
	if (OS_thread_num > 0) {
		OS_ready_set |= (1U << (OS_thread_num - 1U));
	}
	
	OS_thread_num++;
}

// optimize attribute is used to particularly optimize marked piece of code
__attribute__ ((naked, optimize("-fno-stack-protector")))
void PendSV_Handler() {	
	__asm volatile(
		// disable IRQs
		"CPSID 	i					\n"
		// following 2 lines are equivalent of: r1 = OS_curr->sp
		// first load pointer: OS_curr, then load sp member of the struct pointed by OS_curr
		"LDR	R1,=OS_curr 		\n"		// R1 = &OS_curr
		"LDR	R1,[R1, #0x00] 		\n"		// R1 = OS_curr
			
		// branch if OS_curr == NULL, meaning no process is active, 
		// so restore a process	
		// "CBZ	r1,PendSV_restore 	\n"
		"CMP	R1,#0x00			\n"	
		"BEQ	PendSV_restore 		\n"
		// else : process is active
		// start saving current state to prep for context switch 	
		"PUSH	{R4-R7}				\n"
		// save current thread's stack pointer to the top of stack	
		// OS_curr->sp = sp
		"LDR	R1,=OS_curr			\n"		// R1 = &OS_curr 
		"LDR	R1,[R1,#0x00] 		\n"		// R1 = OS_curr
		
		"MRS	R2,MSP				\n"		// R2 = MSP(main stack pointer)
		"STR	R2,[R1,#0x00]		\n"		// OS_curr->sp = R2(MSP) 

		// load next thread	
		"PendSV_restore:			\n"
		// load sp from OS_next
		"LDR 	R1,=OS_next			\n"		// R1 = &OS_next
		"LDR	R1,[R1,#0x00]		\n"		// R1 = OS_next	
		"LDR	R1,[R1,#0x00]		\n"		// R1 = OS_next->sp
		"MSR	MSP, R1				\n"		// sp = OS_next->sp 

		// OS_curr = OS_next, update pointer
		// remember that OS_Thread struct is only characterized by the 
		// 'sp'
		"LDR	R1,=OS_next			\n"
		"LDR	R1,[R1,#0x00]		\n"
		"LDR	R2,=OS_curr			\n"
		"STR	R1,[R2,#0x00]		\n"
		
		// restore the extra registers
		// pop stack n times and load values in registers successively 
		// lowest numbered register(eg r4 here) uses the lowest memory addr 
		// and the highest register(eg. r7) using the highest memory addr
		"POP	{R4-R7}				\n"
		
		// renable interrupts 
		"CPSIE	i					\n"
		// LR is loaded with a special value that, when loaded in PC, tells the cpu that
		// the exception is completed and the reversal of stacking is performed by cpu 
		"BX		LR					\n"	
	);
}
