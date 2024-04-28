/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if defined(BOARD_CONFIG_QSD8250_LEO)

#include <mach/mach_types.h>

#include <IOKit/IOPlatformExpert.h>

#include <pexpert/pexpert.h>
#include <pexpert/arm/protos.h>
#include <pexpert/arm/boot.h>

#include <machine/machine_routines.h>

#include <vm/pmap.h>
#include <arm/pmap.h>

#include "pe_qsd8250.h"
#include "pe_leo.h"

#define KPRINTF_PREFIX  "PE_QSD8250: "

#define HwReg(x) *((volatile unsigned long*)(x))

extern void rtclock_intr(arm_saved_state_t * regs);
extern void rtc_configure(uint64_t hz);

uint64_t clock_decrementer = 0;
static boolean_t clock_initialized = FALSE;
static boolean_t clock_had_irq = FALSE;
static uint64_t clock_absolute_time = 0;




/* BLSP UART */
vm_offset_t         gMsmBlspUartBase;

/* VIC */
vm_offset_t gQSD8250Vic0Base;

/* Timer */
vm_offset_t gQSD8250TimerBase;

static void timer_configure(void)
{
    /*
     * DUMMY 
     */
    uint64_t hz = DGT_HZ;
    gPEClockFrequencyInfo.timebase_frequency_hz = hz;

    clock_decrementer = 10000;
    kprintf(KPRINTF_PREFIX "decrementer frequency = %llu\n", clock_decrementer);

    rtc_configure(hz);
    return;
}

uint64_t qsd8250_timer_value(void);
void qsd8250_timer_enabled(int enable);

void qsd8250_timebase_init(void)
{
    uint32_t value = HwReg(VIC_INT_EN0) | (1 << 5) | (1 << 6);
    gQSD8250TimerBase = ml_io_map(MSM_GPT_BASE, PAGE_SIZE);
    assert(gQSD8250TimerBase);
    kprintf("timebase init: gQSD8250TimerBase assert");
    /*
     * Set rtclock stuff 
     */
    timer_configure();
    kprintf("timebase init: called timer_configure");
    /*
     * Disable the timer. 
     */
    qsd8250_timer_enabled(FALSE);
    kprintf("timebase init: disabled timer");
    /*
     * Wait for it. 
     */
    kprintf(KPRINTF_PREFIX "waiting for system timer to come up...\n");
    qsd8250_timer_enabled(TRUE);
    kprintf("timebase init: timer enabled");
    clock_initialized = TRUE;
    kprintf("timebase init: clock init true");
    while (!clock_had_irq)
        barrier();
	kprintf("timebase init: clock didnt have irq");
    kprintf("timebase init: leaving.");
    return;
}

uint64_t qsd8250_get_timebase(void)
{
    uint32_t timestamp;

//    if (!clock_initialized)
//        return 0;

    timestamp = qsd8250_timer_value();

    if (timestamp) {
        uint64_t v = clock_absolute_time;
        v += (uint64_t) (((uint64_t) clock_decrementer) - (uint64_t) (timestamp));
        return v;
    } else {
        clock_absolute_time += clock_decrementer;
        return clock_absolute_time;
    }
}

void qsd8250_timer_settimerperiod(uint64_t TimerPeriod)
{
	uint64_t       TimerCount;
	uint64_t mTimerPeriod = 0;
	if (TimerPeriod == 0) 
  	{
    		/* Turn off the timer */
    		writel(0, gQSD8250TimerBase + 0x0018);
    		writel(0, gQSD8250TimerBase + 0x001C);
    		ml_set_interrupts_enabled(FALSE);
		kprintf("timerperiod: turned off timer");
	}

	else
  	{
		/* Disable the timer interrupt */
    		ml_set_interrupts_enabled(FALSE);
		kprintf("timerperiod: interrupt off");
    		/* The following code expects time in ms */
    		TimerCount = TimerPeriod / 10000;

    		writel(TimerCount * (DGT_HZ / 1000), gQSD8250TimerBase + 0x0010);
	  	writel(0, gQSD8250TimerBase + 0x001C);
	  	writel(3, gQSD8250TimerBase + 0x0018);

    		/* Enable the timer interrupt */
   		ml_set_interrupts_enabled(TRUE);
		kprintf("timerperiod: interrupt on");
  	}

  	/* Save the new timer period */
  	mTimerPeriod = TimerPeriod;
	kprintf("timerperiod: leaving.");
  	return;
}

uint64_t qsd8250_timer_value(void)
{
    uint64_t ret = (uint64_t) ((uint32_t) 0xFFFFFFFF - (uint32_t) HwReg(gQSD8250TimerBase + 0x0014));

    /*
     * HACK 
     */
    if (ret >= clock_decrementer)
        ret = 0;

    return ret;
}
/*
void qsd8250_timer_enabled(int enable)
{
	// do we ever disable this??
	if (!enable) 
	{
		// disable timer
		qsd8250_timer_settimerperiod(0);
		kprintf("disable timer");
	}
	else
	{
		// set up default timer (1ms period)
		qsd8250_timer_settimerperiod(1000);
		kprintf("set 1ms timer");
	}
	kprintf("enabled timer, leaving.");
	return;
}

/* TODO */
void qsd8250_timer_enabled(int enable)
{
    if(enable) {
        HwReg(gQSD8250TimerBase + DGT_MATCH_VAL) = clock_decrementer;
        HwReg(gQSD8250TimerBase + DGT_CLEAR) = 0;
        HwReg(gQSD8250TimerBase + DGT_ENABLE) = DGT_ENABLE_EN | DGT_ENABLE_CLR_ON_MATCH_EN;
    } else {
        HwReg(gQSD8250TimerBase + DGT_ENABLE) = 0;
        barrier();
        HwReg(gQSD8250TimerBase + DGT_CLEAR) = 0;
        barrier();
    }
    return;
}

void qsd8250_handle_interrupt(void *context)
{
    uint32_t current_irq = HwReg(gQSD8250Vic0Base + 0x00D0);
    // gQSD8250VICVECWR = ml_io_map(VIC_IRQ_VEC_WR, PAGE_SIZE);
    /*
     * Timer IRQs are handeled by us. 
     */
    
    if(current_irq > NR_IRQS) {
        kprintf(KPRINTF_PREFIX "Got a bogus IRQ?");
        return;
    }
    if (current_irq == 8) {
        /*
         * Disable timer 
         */
        qsd8250_timer_enabled(FALSE);
	kprintf("interrupt_handle: timer off");
        /*
         * Update absolute time 
         */
        clock_absolute_time += (clock_decrementer - (int64_t) qsd8250_timer_value());
	kprintf("interrupt_handle: updated absolute time");
        /*
         * Resynchronize deadlines. 
         */
        rtclock_intr((arm_saved_state_t *) context);
	kprintf("interrupt_handle: synced deadlines");
        /*
         * Enable timer. 
         */
        qsd8250_timer_enabled(TRUE);
	kprintf("interrupt_handle: timer on");

        /*
         * We had an IRQ. 
         */
        clock_had_irq = TRUE;
	kprintf("interrupt_handle: had an ir1");
    } else {
        irq_iokit_dispatch(current_irq);
    }
    /*
    * EOI. 
    */
    // HwReg(gQSD8250VICVECWR) = 0;
    writel(0, gQSD8250Vic0Base + 0x00D8);
    kprintf("interrupt_handle: EOI");

    kprintf("interrupt_handle: leaving.");
    return;
}


/* Functions */
void qsd8250_uart_dm_putc(int c);
int qsd8250_uart_dm_getc(void);

void udelay(unsigned usecs);

/*
 * Stub for printing out to framebuffer.
 */
void vcputc(__unused int l, __unused int u, int c);

/* UART */
/*
 * On Leo, there is no known accessible UART port, but maybe keep in mind to add support for passion (Nexus One)?
 *
 */

int qsd8250_VICInit(void)
{
	/* we have 2 VICs, lets just do 1 for now tho */
	assert(gQSD8250Vic0Base);
	kprintf("VICInit: asserted");

	/*
     	* Disable interrupts 
     	*/
    	ml_set_interrupts_enabled(FALSE);
	kprintf("VICInit: int off");
	/* do qcom voodoo magic */
	writel(0xffffffff, gQSD8250Vic0Base + 0x00B0);
	writel(0xffffffff, gQSD8250Vic0Base + 0x00B4);
	kprintf("VICInit: 1st writel");
	writel(0, gQSD8250Vic0Base + 0x0000);
	writel(0, gQSD8250Vic0Base + 0x0004);
	kprintf("VICInit: 2st writel");
	writel(0xffffffff, gQSD8250Vic0Base + 0x0040);
	writel(0xffffffff, gQSD8250Vic0Base + 0x00044);
	kprintf("VICInit: 3st writel");
	writel(0, gQSD8250Vic0Base + 0x006C);
	kprintf("VICInit: 4st writel");
	writel(1, gQSD8250Vic0Base + 0x0010);
	writel(1, gQSD8250Vic0Base + 0x0014);
	kprintf("VICInit: 5st writel");
	writel(1, gQSD8250Vic0Base + 0x0068);
	kprintf("VICInit: leaving.");
	return 0;
}

/* Framebuffer */
static void _fb_putc(int c)
{
	if (c == '\n') {
		vcputc(0, 0, '\r');
	}
	vcputc(0, 0, c);
}

/* Initialize a framebuffer */
void panel_init(void)
{
	char tempbuf[16]; 
	uint64_t panel_width = PANEL_WIDTH,
		 panel_height = PANEL_HEIGHT;


	/*
	* The hardware demands a framebuffer, but the framebuffer has to be given
	* in a hardware address.
	*/

	/* void *framebuffer = pmap_steal_memory(1280 * 720 * 4);
	 * void *framebuffer_phys = pmap_extract(kernel_pmap, framebuffer);
	 */

	PE_state.video.v_baseAddr = (unsigned long)0x2a00000;
	PE_state.video.v_rowBytes = panel_width * 4;
	PE_state.video.v_width = panel_width;
	PE_state.video.v_height = panel_height;
	PE_state.video.v_depth = 2 * (8);   /* Always 16bpp */

	kprintf(KPRINTF_PREFIX "Framebuffer initialized\n");

	/*
	 * Enable early framebuffer.
	 */
	if (PE_parse_boot_argn("-early-fb-debug", tempbuf, sizeof(tempbuf)))
		initialize_screen((void *) &PE_state.video, kPEAcquireScreen);
	else if (PE_parse_boot_argn("-graphics-mode", tempbuf, sizeof(tempbuf))) {
		memset(PE_state.video.v_baseAddr, 0xb9, PE_state.video.v_rowBytes * PE_state.video.v_height);

		initialize_screen((void *) &PE_state.video, kPEGraphicsMode);
	} else {
		initialize_screen((void *) &PE_state.video, kPETextMode);
	}

	return;
}
void qsd8250_mapping_init(void)
{
	gQSD8250Vic0Base = ml_io_map(MSM_VIC_BASE, PAGE_SIZE);
	gQSD8250TimerBase = ml_io_map(MSM_GPT_BASE, PAGE_SIZE);
	kprintf("mapping init: mapped all, leaving.");
	return;
}
void PE_board_init(void)
{
	qsd8250_mapping_init();

	gPESocDispatch.uart_getc = unimplemented;
	gPESocDispatch.uart_putc = unimplemented;
	gPESocDispatch.uart_init = unimplemented;

	gPESocDispatch.interrupt_init = qsd8250_VICInit;
	gPESocDispatch.handle_interrupt = qsd8250_handle_interrupt;

	gPESocDispatch.timebase_init = qsd8250_timebase_init;
	gPESocDispatch.get_timebase = qsd8250_get_timebase;

	gPESocDispatch.timer_value = qsd8250_timer_value;
	gPESocDispatch.timer_enabled = qsd8250_timer_enabled;

	gPESocDispatch.framebuffer_init = panel_init;

	panel_init();

	PE_halt_restart = unimplemented;
}

void PE_init_SocSupport_stub(void)
{
	PE_early_puts("PE_init_SocSupport: Initializing for HTC HD2 (leo)\n");
	PE_board_init();
}

#endif /* !BOARD_CONFIG_QSD8250_LEO */
