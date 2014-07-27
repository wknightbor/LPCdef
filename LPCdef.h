/************************************************************************

  Control bits definitions suitable for LPC1114, LPC1112 and similar CPU.
  Ver 0.0
   Information was taken from corresponding datasheets <www.nxp.com>.

  (C) Copyright 2013
  Andrey Karpenko  <andrey@delfa.net>
 
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation; either version 2 of
  the License, or (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston,
  MA 02111-1307 USA

************************************************************************/
// LPC1114 adc
// LPC_ADC->CR
// A/D Control Register

// Selects which of the AD7:0 pins is (are) to be sampled and converted.
#define AD0CR_SEL		(0)
#define AD0CR_SEL_Msk		(0b11111111 << AD0CR_SEL)
// The APB clock (PCLK) is divided by CLKDIV +1 to produce the clock for the ADC
#define AD0CR_CLKDIV	(8)
#define AD0CR_CLKDIV_Msk	(0b11111111 << AD0CR_CLKDIV)
// Burst mode Hardware scan
#define AD0CR_BURST		(16)
#define AD0CR_BURST_Msk		(0b1 << AD0CR_BURST)
// This field selects the number of clocks used for each conversion in Burst mode
// 		0x0 - 11 ..
//	..	0x7 - 4
#define AD0CR_CLKS		(17)
#define AD0CR_CLKS_Msk		(0b111 << AD0CR_CLKS)
// When the BURST bit is 0, these bits control whether and when an A/D conversion is started:
//		0x0 No start (this value should be used when clearing PDN to 0).
// 		0x1 Start conversion now.
// 		0x2 Start conversion when event occurs on PIO0_2/SSEL/CT16B0_CAP0.
// 		0x3 Start conversion when event occurs on PIO1_5/DIR/CT32B0_CAP0.
// 		0x4 Start conversion when event occurs on CT32B0_MAT0.
// 		0x5 Start conversion when event occurs on CT32B0_MAT1.
// 		0x6 Start conversion when event occurs on CT16B0_MAT0.
// 		0x7 Start conversion when event occurs on CT16B0_MAT1.
#define AD0CR_START		(24)
#define AD0CR_START_Msk		(0b111 << AD0CR_START)
// Start conversion on a rising/falling edge
#define AD0CR_EDGE		(27)
#define AD0CR_EDGE_Msk		(0b1 << AD0CR_EDGE)

// LPC_ADC->GDR
// A/D Global Data Register
#define AD0GDR_V_VREF	(6)
#define AD0GDR_V_VREF_Msk	(0b1111111111 << AD0GDR_V_VREF)

#define AD0GDR_CHN		(24)
#define AD0GDR_CHN_Msk		(0b111 << AD0GDR_CHN)

#define AD0GDR_OVERRUN	(30)
#define AD0GDR_OVERRUN_Msk	(0b1 << AD0GDR_OVERRUN)

#define AD0GDR_DONE		(31)
#define AD0GDR_DONE_Msk		(0b1 << AD0GDR_DONE)

// LPC_ADC->INTEN
// A/D Interrupt Enable Register
#define AD0INTEN_ADINTEN	(0)
#define AD0INTEN_ADINTEN_Msk	(0b11111111 << AD0INTEN_ADINTEN)

#define AD0INTEN_ADGINTEN	(8)
#define AD0INTEN_ADGINTEN_Msk	(0b1 << AD0INTEN_ADGINTEN)

// LPC_ADC->DR
// A/D Data Registers 0-7
#define AD0DRx_V_VREF	(6)
#define AD0DRx_V_VREF_Msk	(0b1111111111 << AD0DRx_V_VREF)

#define AD0DRx_OVERRUN	(30)
#define AD0DRx_OVERRUN_Msk	(0b1 << AD0DRx_OVERRUN)

#define AD0DRx_DONE		(31)
#define AD0DRx_DONE_Msk		(0b1 << AD0DRx_DONE)

// LPC_ADC->STAT
// A/D Status Register
#define AD0STAT_DONE	(0)
#define AD0STAT_DONE_Msk	(0b11111111 << AD0STAT_DONE)

#define AD0STAT_OVERRUN	(8)
#define AD0STAT_OVERRUN_Msk	(0b11111111 << AD0STAT_OVERRUN)

#define AD0STAT_ADINT	(16)
#define AD0STAT_ADINT_Msk	(0b1 << AD0STAT_ADINT)

#define CLKSEL_IRC		(0x0)
#define CLKSEL_SYS		(0x1)
#define CLKSEL_WDG		(0x2)
#define CLKSEL_MAIN		(0x3)


// LPC1114 uart
//
// LPC_UART->RBR
// UART Receiver Buffer Register
#define URBR_RBR		(0)
#define URBR_RBR_Msk		(0b11111111 << URBR_RBR)
// LPC_UART->THR
// UART Transmitter Holding Register
#define UTHR_THR		(0)
#define UTHR_THR_Msk		(0b11111111 << UTHR_THR)
// LPC_UART->DLL
// UART Divisor Latch LSB Register
#define UDLL_DLLSB		(0)
#define UDLL_DLLSB_Msk		(0b11111111 << UDLL_DLLSB)
// LPC_UART->DLM
// UART Divisor Latch MSB Register
#define UDLM_DLMSB		(0)
#define UDLM_DLMSB_Msk		(0b11111111 << UDLM_DLMSB)
// LPC_UART->IER
// UART Interrupt Enable Register
// RBR Interrupt Enable. Enables the Receive Data Available interrupt
//	for UART. It also controls the Character Receive Time-out interrupt.
#define UIER_RBRIE		(0)
#define UIER_RBRIE_Msk		(0b1 << UIER_RBRIE)
// THRE Interrupt Enable. Enables the THRE interrupt for UART. The status
//	of this interrupt can be read from U0LSR[5].
#define UIER_THREIE		(1)
#define UIER_THREIE_Msk		(0b1 << UIER_THREIE)
// RX Line Interrupt Enable. Enables the UART RX line status interrupts.
//	The status of this interrupt can be read from U0LSR[4:1].
#define UIER_RXLIE		(2)
#define UIER_RXLIE_Msk		(0b1 << UIER_RXLIE)
// Enables the end of auto-baud interrupt.
#define UIER_ABEOINTEN	(8)
#define UIER_ABEOINTEN_Msk	(0b1 << UIER_ABEOINTEN)
// Enables the auto-baud time-out interrupt.
#define UIER_ABTOINTEN	(9)
#define UIER_ABTOINTEN_Msk	(0b1 << UIER_ABTOINTEN)

// LPC_UART->IIR
// UART Interrupt Identification Register
// Interrupt status. Note that U0IIR[0] is active low.
#define UIIR_INTSTATUS	(0)
#define UIIR_INTSTATUS_Msk	(0b1 << UIIR_INTSTATUS)
// Interrupt identification.
//	0x3 1 - Receive Line Status (RLS).
//	0x2 2a - Receive Data Available (RDA).
//	0x6 2b - Character Time-out Indicator (CTI).
//	0x1 3 - THRE Interrupt.
//	0x0 4 - Modem interrupt.
#define UIIR_INTID		(1)
#define UIIR_INTID_Msk		(0b111 << UIIR_INTID)
// These bits are equivalent to U0FCR[0].
#define UIIR_FIFOENABLE	(6)
#define UIIR_FIFOENABLE_Msk	(0b11 << UIIR_FIFOENABLE)
// End of auto-baud interrupt. True if auto-baud has finished successfully and interrupt is enabled.
#define UIIR_ABEOINT	(8)
#define UIIR_ABEOINT_Msk	(0b1 << UIIR_ABEOINT)
// Auto-baud time-out interrupt. True if auto-baud has timed out and interrupt is enabled.
#define UIIR_ABTOINT	(9)
#define UIIR_ABTOINT_Msk	(0b1 << UIIR_ABTOINT)

// LPC_UART->FCR
// UART FIFO Control Register
// FIFO Enable. This bit must be set for proper UART operation.
#define UFCR_FIFOEN		(0)
#define UFCR_FIFOEN_Msk		(0b1 << UFCR_FIFOEN)
// RX FIFO Reset
#define UFCR_RXFIFORES	(1)
#define UFCR_RXFIFORES_Msk	(0b1 << UFCR_RXFIFORES)
// TX FIFO Reset
#define UFCR_TXFIFORES	(2)
#define UFCR_TXFIFORES_Msk	(0b1 << UFCR_TXFIFORES)
// RX Trigger Level. These two bits determine how many receiver UART FIFO characters must be written before an interrupt is activated.
#define UFCR_RXTL		(6)
#define UFCR_RXTL_Msk		(0b11 << UFCR_RXTL)

// LPC_UART->LCR
// UART Line Control Register
// Word Length Select
#define ULCR_WLS		(0)
#define ULCR_WLS_Msk		(0b11 << ULCR_WLS)
// Stop Bit Select
#define ULCR_SBS		(2)
#define ULCR_SBS_Msk		(0b1 << ULCR_SBS)
// Parity Enable
#define ULCR_PE			(3)
#define ULCR_PE_Msk			(0b1 << ULCR_PE)
// Parity Select
#define ULCR_PS			(4)
#define ULCR_PS_Msk			(0b11 << ULCR_PS)
// Break Control
#define ULCR_BC			(6)
#define ULCR_BC_Msk			(0b1 << ULCR_BC)
// Divisor Latch Access Bit
#define ULCR_DLAB		(7)
#define ULCR_DLAB_Msk		(0b1 << ULCR_DLAB)

// LPC_UART->MCR
// UART Modem Control Register
// DTR Control. Source for modem output pin, DTR. This bit reads
//		as 0 when modem loopback mode is active.
#define UMCR_DTRC		(0)
#define UMCR_DTRC_Msk		(0b1 << UMCR_DTRC)
// RTS Control. Source for modem output pin RTS. This bit reads as
//		0 when modem loopback mode is active.
#define UMCR_RTSC		(1)
#define UMCR_RTSC_Msk		(0b1 << UMCR_RTSC)
// Loopback Mode Select. The modem loopback mode provides a
//		mechanism to perform diagnostic loopback testing
#define UMCR_LMS		(4)
#define UMCR_LMS_Msk		(0b1 << UMCR_LMS)
// RTS flow control
#define UMCR_RTSEN		(6)
#define UMCR_RTSEN_Msk		(0b1 << UMCR_RTSEN)
// CTS flow control
#define UMCR_CTSEN		(7)
#define UMCR_CTSEN_Msk		(0b1 << UMCR_CTSEN)

// LPC_UART->LSR
// UART Line Status Register
// Receiver Data Ready.
#define ULSR_RDR		(0)
#define ULSR_RDR_Msk		(0b1 << ULSR_RDR)
// Overrun Error.
#define ULSR_OE		(1)
#define ULSR_OE_Msk		(0b1 << ULSR_OE)
// Parity Error.
#define ULSR_PE		(2)
#define ULSR_PE_Msk		(0b1 << ULSR_PE)
// Framing Error.
#define ULSR_FE		(3)
#define ULSR_FE_Msk		(0b1 << ULSR_FE)
// Break Interrupt.
#define ULSR_BI		(4)
#define ULSR_BI_Msk		(0b1 << ULSR_BI)
// Transmitter Holding Register Empty.
#define ULSR_THRE		(5)
#define ULSR_THRE_Msk		(0b1 << ULSR_THRE)
// Transmitter Empty.
#define ULSR_TEMT		(6)
#define ULSR_TEMT_Msk		(0b1 << ULSR_TEMT)
// Error in RX FIFO.
#define ULSR_RXFE		(7)
#define ULSR_RXFE_Msk		(0b1 << ULSR_RXFE)

// LPC_UART->MSR
// UART Modem Status Register
// Delta CTS. Set upon state change of input CTS. Cleared on a U0MSR read.
#define UMSR_DCTS		(0)
#define UMSR_DCTS_Msk		(0b1 << UMSR_DCTS)
// Delta DSR. Set upon state change of input DSR. Cleared on a U0MSR read.
#define UMSR_DDSR		(1)
#define UMSR_DDSR_Msk		(0b1 << UMSR_DDSR)
// Trailing Edge RI. Set upon low to high transition of input RI. Cleared on a U0MSR read.
#define UMSR_TERI		(2)
#define UMSR_TERI_Msk		(0b1 << UMSR_TERI)
// Delta DCD. Set upon state change of input DCD. Cleared on a U0MSR read.
#define UMSR_DDCD		(3)
#define UMSR_DDCD_Msk		(0b1 << UMSR_DDCD)
// Delta DCD. Set upon state change of input DCD. Cleared on a U0MSR read.
#define UMSR_CTS		(4)
#define UMSR_CTS_Msk		(0b1 << UMSR_CTS)
// Data Set Ready State. Complement of input signal DSR. This bit is connected to U0MCR[0] in modem loopback mode.
#define UMSR_DSR		(5)
#define UMSR_DSR_Msk		(0b1 << UMSR_DSR)
// Ring Indicator State. Complement of input RI. This bit is connected to U0MCR[2] in modem loopback mode.
#define UMSR_RI			(6)
#define UMSR_RI_Msk			(0b1 << UMSR_IR)
// Data Carrier Detect State. Complement of input DCD. This bit is connected to U0MCR[3] in modem loopback mode.
#define UMSR_DCD		(7)
#define UMSR_DCD_Msk		(0b1 << UMSR_DCD)

// LPC_UART->SCR
// UART Scratch Pad Register
// A readable, writable byte.
#define USCR_PAD				(0)
#define USCR_PAD_Msk				(0b11111111 << USCR_PAD)

// LPC_UART->ACR
// UART Auto-baud Control Register
// Start bit. This bit is automatically cleared after auto-baud completion.
#define UACR_START				(0)
#define UACR_START_Msk				(0b1 << UACR_START)
// Auto-baud mode select
#define UACR_MODE				(1)
#define UACR_MODE_Msk				(0b1 << UACR_MODE)
// Restart enable
#define UACR_AUTORESTART		(2)
#define UACR_AUTORESTART_Msk		(0b1 << UACR_AUTORESTART)
// End of auto-baud interrupt clear (write only accessible)
#define UACR_ABEOINTCLR			(8)
#define UACR_ABEOINTCLR_Msk			(0b1 << UACR_ABEOINTCLR)
// Auto-baud time-out interrupt clear (write only accessible)
#define UACR_ABTOINTCLR			(9)
#define UACR_ABTOINTCLR_Msk			(0b1 << UACR_ABTOINTCLR)

// LPC_UART->FCR
// UART Fractional Divider Register
// Baud rate generation pre-scaler divisor value. If this field is 0,
//	fractional baud rate generator will not impact the UART baud rate.
#define UFCR_DIVADDVAL			(0)
#define UFCR_DIVADDVAL_Msk			(0b1111 << UFACR_DIVADDVAL)
// Baud rate pre-scaler multiplier value. This field must be greater or
//	equal 1 for UART to operate properly, regardless of whether the
//	fractional baud rate generator is used or not.
#define UFCR_MULVAL				(4)
#define UFCR_MULVAL_Msk				(0b1111 << UFACR_MULVAL)

// LPC_UART->TER
// UART Transmit Enable Register
// When this bit is 1, as it is after a Reset, data written to the THR
//	is output on the TXD pin as soon as any preceding data has
//	been sent. If this bit cleared to 0 while a character is being sent,
//	the transmission of that character is completed, but no further
//	characters are sent until this bit is set again. In other words, a 0
//	in this bit blocks the transfer of characters from the THR or TX
//	FIFO into the transmit shift register. Software can clear this bit
//	when it detects that the a hardware-handshaking TX-permit
//	signal (CTS) has gone false, or with software handshaking,
//	when it receives an XOFF character (DC3). Software can set
//	this bit again when it detects that the TX-permit signal has gone
//	true, or when it receives an XON (DC1) character.
#define UTER_TXEN				(7)
#define UTER_TXEN_Msk				(0b1 << UTER_TXEN)

// LPC_UART->RS485CTRL
// UART RS485 Control register
// Normal Multidrop Mode Enable
#define URS485CTRL_NMMEN		(0)
#define URS485CTRL_NMMEN_Msk		(0b1 << URS485CTRL_NMMEN)
// Receiver enable.
#define URS485CTRL_RXDIS		(1)
#define URS485CTRL_RXDIS_Msk		(0b1 << URS485CTRL_RXDIS)
// Auto Address Detect enable.
#define URS485CTRL_AADEN		(2)
#define URS485CTRL_AADEN_Msk		(0b1 << URS485CTRL_AADEN)
// Select direction control pin
//	0 	If direction control is enabled (bit DCTRL = 1), pin
//			RTS is used for direction control.
//	1 	If direction control is enabled (bit DCTRL = 1), pin
//			DTR is used for direction control
#define URS485CTRL_SEL			(3)
#define URS485CTRL_SEL_Msk			(0b1 << URS485CTRL_SEL)
// Auto direction control enable.
#define URS485CTRL_DCTRL		(4)
#define URS485CTRL_DCTRL_Msk		(0b1 << URS485CTRL_DCTRL)
// Polarity control. This bit reverses the polarity of the direction control signal on the RTS (or DTR) pin.
//	0 	The direction control pin will be driven to logic 0
//			when the transmitter has data to be sent. It will be
//			driven to logic 1 after the last bit of data has been
//			transmitted.
//	1 	The direction control pin will be driven to logic 1
//			when the transmitter has data to be sent. It will be
//			driven to logic 0 after the last bit of data has been
//			transmitted.
#define URS485CTRL_OINV			(5)
#define URS485CTRL_OINV_Msk			(0b1 << URS485CTRL_OINV)

// LPC_UART->ADRMATCH
// UART RS485 Address Match register
#define URS485ADRMATCH_ADRMATCH (0)
#define URS485ADRMATCH_ADRMATCH_Msk (0b11111111 << URS485ADRMATCH_ADRMATCH)

// LPC_UART->RS485DLY
// UART1 RS485 Delay value register
#define URS485DLY_DLY 			(0)
#define URS485DLY_DLY_Msk 			(0b11111111 << URS485DLY_DLY)



//
// IOCON_PIOx_x
// Pin Modes
// Inactive (no pull-down/pull-up resistor enabled).
#define PIO_MODE_INACTIVE	(0x0<<3)
// Pull-down resistor enabled.
#define PIO_MODE_PULLDOWN	(0x1<<3)
// Pull-up resistor enabled.
#define PIO_MODE_PULLUP		(0x2<<3)
// Repeater mode.
#define PIO_MODE_REPEATOR	(0x3<<3)
// Hysteresis
#define PIO_HYS				(0x1<<5)
// Reserved
#define PIO_RES				(0x3<<6)
// Selects Analog/Digital mode 0 - analog 1 - digital
#define PIO_ADMODE			(0x1<<7)
// Selects pseudo open-drain mode
#define PIO_OD				(0x1<<10)

// IOCON_PIO0_4/5
// Pin Modes

// Selects function PIO0_x
#define PIO0_4_5_MODE_OD			(0x00)
// Selects I2C function
#define PIO0_4_5_MODE_I2C			(0x01)
// Standard mode/ Fast-mode I2C.
#define PIO0_4_5_I2CMODE_FAST		(0x00<8)
// Standard I/O functionality
#define PIO0_4_5_I2CMODE_IO			(0x01<8)
// Fast-mode Plus I2C
#define PIO0_4_5_I2CMODE_FASTPLUS	(0x01<8)




// LPC1114 Timers

// LPC_TMRxxBn->IR
// Interrupt Register
// Interrupt flag for match channel 0.
#define IR_MR0				(0x0)
#define IR_MR0_Msk			(1<<IR_MR0)
// Interrupt flag for match channel 1.
#define IR_MR1				(0x1)
#define IR_MR1_Msk			(1<<IR_MR1)
// Interrupt flag for match channel 2.
#define IR_MR2				(0x2)
#define IR_MR2_Msk			(1<<IR_MR2)
// Interrupt flag for match channel 3.
#define IR_MR3				(0x3)
#define IR_MR3_Msk			(1<<IR_MR3)
// Interrupt flag for capture channel 0 event.
#define IR_CR0				(0x4)
#define IR_CR0_Msk			(1<<IR_CR0)
// Interrupt flag for capture channel 1 event.(for LPC1100XL series)
#define IR_CR1				(0x5)
#define IR_CR1_Msk			(1<<IR_CR1)

// LPC_TMRxxBn->TCR
// Timer Control Register
// Counter Enable. When one, the Timer Counter and
//  Prescale Counter are enabled for counting. When zero,
//  the counters are disabled.
#define TCR_CEN				(0x0)
#define TCR_CEN_Msk			(1<<TCR_CEN)
// Counter Reset. When one, the Timer Counter and the
//  Prescale Counter are synchronously reset on the next
//  positive edge of PCLK. The counters remain reset until
//  TCR[1] is returned to zero.
#define TCR_CRST			(0x1)
#define TCR_CRST_Msk		(1<<TCR_CRST)

// LPC_TMRxxBn->MCR
// Match Control Register
// Interrupt on MR0: an interrupt is generated when MR0 matches the value in the TC.
#define MCR_MR0I			(0x0)
#define MCR_MR0I_Msk		(1<<MCR_MR0I)
// Reset on MR0: the TC will be reset if MR0 matches it.
#define MCR_MR0R			(0x1)
#define MCR_MR0R_Msk		(1<<MCR_MR0R)
// Stop on MR0: the TC and PC will be stopped and TCR[0] will be set to 0 if MR0 matches
//  the TC.
#define MCR_MR0S			(0x2)
#define MCR_MR0S_Msk		(1<<MCR_MR0S)

#define MCR_MR1I			(0x3)
#define MCR_MR1I_Msk		(1<<MCR_MR1I)
#define MCR_MR1R			(0x4)
#define MCR_MR1R_Msk		(1<<MCR_MR1R)
#define MCR_MR1S			(0x5)
#define MCR_MR1S_Msk		(1<<MCR_MR1S)
#define MCR_MR2I			(0x6)
#define MCR_MR2I_Msk		(1<<MCR_MR2I)
#define MCR_MR2R			(0x7)
#define MCR_MR2R_Msk		(1<<MCR_MR2R)
#define MCR_MR2S			(0x8)
#define MCR_MR2S_Msk		(1<<MCR_MR2S)

#define MCR_MR3I			(0x9)
#define MCR_MR3I_Msk		(1<<MCR_MR3I)
#define MCR_MR3R			(0xa)
#define MCR_MR3R_Msk		(1<<MCR_MR3R)
#define MCR_MR3S			(0xb)
#define MCR_MR3S_Msk		(1<<MCR_MR3S)

// LPC_TMRxxBn->CCR
// Capture Control Register
// Capture on CT16Bn_CAP0 rising edge: a sequence of 0 then 1 on CT16Bn_CAP0 will
//  cause CR0 to be loaded with the contents of TC.
#define CCR_CAP0RE			(0x0)
#define CCR_CAP0RE_Msk		(1<<CCR_CAP0RE)
// Capture on CT16Bn_CAP0 falling edge: a sequence of 1 then 0 on CT16Bn_CAP0 will
//  cause CR0 to be loaded with the contents of TC.
#define CCR_CAP0FE			(0x1)
#define CCR_CAP0FE_Msk		(1<<CCR_CAP0FE)
// Interrupt on CT16Bn_CAP0 event: a CR0 load due to a CT16Bn_CAP0 event will
//  generate an interrupt.
#define CCR_CAP0I			(0x2)
#define CCR_CAP0I_Msk		(1<<CCR_CAP0I)

// LPC_TMRxxBn->EMR
// External Match Register
// External Match 0. This bit reflects the state of output CTxxBn_MAT0,
//  whether or not this output is connected to its pin. When a match occurs between the TC
//  and MR0, this bit can either toggle, go LOW, go HIGH, or do nothing. Bits EMR[5:4]
//  control the functionality of this output. This bit is driven to the
//  CTxxBn_MAT0 pins if the match function is selected in the IOCON
//  registers (0 = LOW, 1 = HIGH).
#define EMR_EM0				(0x0)
#define EMR_EM0_Msk			(1<<EMR_EM0)
#define EMR_EM1				(0x1)
#define EMR_EM1_Msk			(1<<EMR_EM1)
#define EMR_EM2				(0x2)
#define EMR_EM2_Msk			(1<<EMR_EM2)
#define EMR_EM3				(0x3)
#define EMR_EM3_Msk			(1<<EMR_EM3)
// External Match Control 0. Determines the functionality of External Match 0.
//  0  Do Nothing.
//  1  Clear the corresponding External Match bit/output to 0 (CTxxBn_MATm pin is LOW if
//      pinned out).
//  2  Set the corresponding External Match bit/output to 1 (CTxxBn_MATm pin is HIGH if
//      pinned out).
//  3  Toggle the corresponding External Match bit/output.
#define EMR_EMC0			(0x4)
#define EMR_EMC0_Msk		(0b11<<EMR_EMC0)
#define EMR_EMC1			(0x6)
#define EMR_EMC1_Msk		(0b11<<EMR_EMC1)
#define EMR_EMC2			(0x8)
#define EMR_EMC2_Msk		(0b11<<EMR_EMC2)
#define EMR_EMC3			(0xa)
#define EMR_EMC3_Msk		(0b11<<EMR_EMC3)

// LPC_TMRxxBn->CTCR
// Count Control Register
// Counter/Timer Mode. This field selects which rising PCLK
//  edges can increment Timer’s Prescale Counter (PC), or clear
//  PC and increment Timer Counter (TC).
//   0x0 Timer Mode: every rising PCLK edge
//   0x1 Counter Mode: TC is incremented on rising edges on the
//        CAP input selected by bits 3:2.
//   0x2 Counter Mode: TC is incremented on falling edges on the
//        CAP input selected by bits 3:2.
//   0x3 Counter Mode: TC is incremented on both edges on the CAP
//        input selected by bits 3:2.
#define CTCR_CTM			(0x0)
#define CTCR_CTM_Msk		(0b11<<CTCR_CTM)
// Count Input Select. In counter mode (when bits 1:0 in this
//  register are not 00), these bits select which CAP pin is
//  sampled for clocking. Note: If Counter mode is selected in
//  the CTCR register, bits 2:0 in the Capture Control Register
//  (CCR) must be programmed as 000.
//   0 CTxxBn_CAP0
//   1 CTxxBn_CAP1 (16 bit timers only)
#define CTCR_CIS			(0x2)
#define CTCR_CIS_Msk		(0b11<<CTCR_CIS)
// Setting this bit to one enables clearing of the timer and the
//  prescaler when the capture-edge event specified in bits 7:5
//  occurs. (LPC1100XL series)
#define CTCR_ENCC			(0x4)
#define CTCR_ENCC_Msk		(1<<CTCR_ENCC)
// When bit 4 is one, these bits select which capture input edge
//  will cause the timer and prescaler to be cleared. These bits
//  have no effect when bit 4 is zero. (LPC1100XL series)
//   0x0 Rising Edge of CAP0 clears the timer (if bit 4 is set).
//   0x1 Falling Edge of CAP0 clears the timer (if bit 4 is set).
//   0x2 Rising Edge of CAP1 clears the timer (if bit 4 is set).
//   0x3 Falling Edge of CAP1 clears the timer (if bit 4 is set).
#define CTCR_SELCC			(0x5)
#define CTCR_SELCC_Msk		(0b111<<CTCR_SELCC)

// LPC_TMRxxBn->PWMC
// PWM Control register
// PWM channel0 enable
#define PWMC_PWMEN0			(0x0)
#define PWMC_PWMEN0_Msk		(1<<PWMC_PWMEN0)
#define PWMC_PWMEN1			(0x1)
#define PWMC_PWMEN1_Msk		(1<<PWMC_PWMEN1)
#define PWMC_PWMEN2			(0x2)
#define PWMC_PWMEN2_Msk		(1<<PWMC_PWMEN2)
#define PWMC_PWMEN3			(0x3)
#define PWMC_PWMEN3_Msk		(1<<PWMC_PWMEN3)

// LPC_SYSCON->SYSAHBCLKCTRL
// System AHB clock control register
// Enables clock for AHB to APB bridge, to the AHB
//  matrix, to the Cortex-M0 FCLK and HCLK, to the
//  SysCon, and to the PMU. This bit is read only.
#define SYSAHBCLKCTRL_SYS			(0x0)
#define SYSAHBCLKCTRL_SYS_Msk		(1<<SYSAHBCLKCTRL_SYS)
// Enables clock for ROM.
#define SYSAHBCLKCTRL_ROM			(0x1)
#define SYSAHBCLKCTRL_ROM_Msk		(1<<SYSAHBCLKCTRL_ROM)
// Enables clock for RAM.
#define SYSAHBCLKCTRL_RAM			(0x2)
#define SYSAHBCLKCTRL_RAM_Msk		(1<<SYSAHBCLKCTRL_RAM)
// Enables clock for flash register interface.
#define SYSAHBCLKCTRL_FLASHREG		(0x3)
#define SYSAHBCLKCTRL_FLASHREG_Msk	(1<<SYSAHBCLKCTRL_FLASHREG)
// Enables clock for flash array access.
#define SYSAHBCLKCTRL_FLASHARRAY	(0x4)
#define SYSAHBCLKCTRL_FLASHARRAY_Msk (1<<SYSAHBCLKCTRL_FLASHARRAY)
// Enables clock for I2C.
#define SYSAHBCLKCTRL_I2C			(0x5)
#define SYSAHBCLKCTRL_I2C_Msk		(1<<SYSAHBCLKCTRL_I2C)
// Enables clock for GPIO.
#define SYSAHBCLKCTRL_GPIO			(0x6)
#define SYSAHBCLKCTRL_GPIO_Msk		(1<<SYSAHBCLKCTRL_GPIO)
// Enables clock for 16-bit counter/timer 0.
#define SYSAHBCLKCTRL_CT16B0		(0x7)
#define SYSAHBCLKCTRL_CT16B0_Msk	(1<<SYSAHBCLKCTRL_CT16B0)
// Enables clock for 16-bit counter/timer 1.
#define SYSAHBCLKCTRL_CT16B1		(0x8)
#define SYSAHBCLKCTRL_CT16B1_Msk	(1<<SYSAHBCLKCTRL_CT16B1)
// Enables clock for 32-bit counter/timer 0.
#define SYSAHBCLKCTRL_CT32B0		(0x9)
#define SYSAHBCLKCTRL_CT32B0_Msk	(1<<SYSAHBCLKCTRL_CT32B0)
// Enables clock for 32-bit counter/timer 1.
#define SYSAHBCLKCTRL_CT32B1		(0xa)
#define SYSAHBCLKCTRL_CT32B1_Msk	(1<<SYSAHBCLKCTRL_CT32B1)
// Enables clock for SPI0.
#define SYSAHBCLKCTRL_SSP0			(0xb)
#define SYSAHBCLKCTRL_SSP0_Msk		(1<<SYSAHBCLKCTRL_SSP0)
// Enables clock for UART.
#define SYSAHBCLKCTRL_UART			(0xc)
#define SYSAHBCLKCTRL_UART_Msk		(1<<SYSAHBCLKCTRL_UART)
// Enables clock for ADC.
#define SYSAHBCLKCTRL_ADC			(0xd)
#define SYSAHBCLKCTRL_ADC_Msk		(1<<SYSAHBCLKCTRL_ADC)
// Enables clock for WDT.
#define SYSAHBCLKCTRL_WDT			(0xf)
#define SYSAHBCLKCTRL_WDT_Msk		(1<<SYSAHBCLKCTRL_WDT)
// Enables clock for I/O configuration block.
#define SYSAHBCLKCTRL_IOCON			(0x10)
#define SYSAHBCLKCTRL_IOCON_Msk		(1<<SYSAHBCLKCTRL_IOCON)
// Enables clock for C_CAN.
#define SYSAHBCLKCTRL_CAN			(0x11)
#define SYSAHBCLKCTRL_CAN_Msk		(1<<SYSAHBCLKCTRL_CAN)
// Enables clock for SPI1.
#define SYSAHBCLKCTRL_SSP1			(0x12)
#define SYSAHBCLKCTRL_SSP1_Msk		(1<<SYSAHBCLKCTRL_SSP1)

// LPC_SYSCON->PDSLEEPCFG
// Deep-sleep configuration register

// Reserved always write these bits
#define PDSLEEPCFG_RES_Msk			(0b1100010110111)

// BOD power-down control in Deep-sleep mode
#define PDSLEEPCFG_BOD_PD 			(0x03)
#define PDSLEEPCFG_BOD_PD_Msk		(1<<PDSLEEPCFG_BOD_PD)

// Watchdog oscillator power control in Deep-sleep mode
#define PDSLEEPCFG_WDTOSC_PD		(0x06)
#define PDSLEEPCFG_WDTOSC_PD_Msk	(1<<PDSLEEPCFG_WDTOSC_PD)


// LPC_SYSCON->PDAWAKECFG
// Wake-up configuration register
// LPC_SYSCON->PDRUNCFG
// ???

// IRC oscillator output wake-up configuration
#define PDCFG_IRCOUT_PD			(0x00)
#define PDCFG_IRCOUT_PD_Msk		(1<<PDCFG_IRCOUT_PD)

// IRC oscillator power-down wake-up configuration
#define PDCFG_IRC_PD			(0x01)
#define PDCFG_IRC_PD_Msk		(1<<PDCFG_IRC_PD)

// Flash wake-up configuration
#define PDCFG_FLASH_PD			(0x02)
#define PDCFG_FLASH_PD_Msk		(1<<PDCFG_FLASH_PD)

// BOD wake-up configuration
#define PDCFG_BOD_PD			(0x03)
#define PDCFG_BOD_PD_Msk		(1<<PDCFG_BOD_PD)

// ADC wake-up configuration
#define PDCFG_ADC_PD			(0x04)
#define PDCFG_ADC_PD_Msk		(1<<PDCFG_ADC_PD)

// System oscillator wake-up configuration
#define PDCFG_SYSOSC_PD			(0x05)
#define PDCFG_SYSOSC_PD_Msk		(1<<PDCFG_SYSOSC_PD)

// Watchdog oscillator wake-up configuration
#define PDCFG_WDTOSC_PD			(0x06)
#define PDCFG_WDTOSC_PD_Msk		(1<<PDCFG_WDTOSC_PD)

// System PLL wake-up configuration
#define PDCFG_SYSPLL_PD			(0x07)
#define PDCFG_SYSPLL_PD_Msk		(1<<PDCFG_SYSPLL_PD)

// Reserved always write these bits
#define PDCFG_RES_Msk			(0b1110110100000000)


// SCB->SCR
// System Control Register

// Send Event on Pending bit
#define SCR_SEVONPEND			(0x04)
#define SCR_SEVONPEND_Msk		(1<<SCR_SEVONPEND)

// Controls whether the processor uses sleep or deep sleep as its low power mode
#define SCR_SLEEPDEEP			(0x02)
#define SCR_SLEEPDEEP_Msk		(1<<SCR_SLEEPDEEP)

// Indicates sleep-on-exit when returning from Handler mode to Thread mode
#define SCR_SLEEPONEXIT			(0x01)
#define SCR_SLEEPONEXIT_Msk		(1<<SCR_SLEEPONEXIT)


// LPC_CAN->CNTL
// CAN control register
// 	0	Normal operation.
// 	1	Initialization is started. On reset, software
//			needs to initialize the CAN controller.
#define CANCNTL_INIT				(0x00)
#define CANCNTL_INIT_Msk			(1<<CANCNTL_INIT)
//	0 Disable CAN interrupts.
//	1 Enable CAN interrupts.
#define CANCNTL_IE					(0x01)
#define CANCNTL_IE_Msk				(1<<CANCNTL_IE)

#define CANCNTL_SIE					(0x02)
#define CANCNTL_SIE_Msk				(1<<CANCNTL_SIE)

#define CANCNTL_EIE					(0x03)
#define CANCNTL_EIE_Msk				(1<<CANCNTL_EIE)

#define CANCNTL_DAR					(0x05)
#define CANCNTL_DAR_Msk				(1<<CANCNTL_DAR)

#define CANCNTL_CCE					(0x06)
#define CANCNTL_CCE_Msk				(1<<CANCNTL_CCE)

#define CANCNTL_TEST				(0x07)
#define CANCNTL_TEST_Msk			(1<<CANCNTL_TEST)



// LPC_CAN->STAT
// CAN status register
// Last error code
//	0x0 No error
//	0x1 Stuff error
//	0x2 Form error
//	0x3 AckError
//	0x4 Bit1Error
//	0x5 Bit0Error
//	0x6 CRCError
//	0x7 Unused
#define CANSTAT_LEC					(0x00)
#define CANSTAT_LEC_Msk				(0b111 << CANSTAT_LEC)
// Transmitted a message successfully
#define CANSTAT_TXOK				(0x03)
#define CANSTAT_TXOK_Msk			(1<<CANSTAT_TXOK)
// Received a message successfully
#define CANSTAT_RXOK				(0x04)
#define CANSTAT_RXOK_Msk			(1<<CANSTAT_RXOK)
// Error passive
#define CANSTAT_EPASS				(0x05)
#define CANSTAT_EPASS_Msk			(1<<CANSTAT_EPASS)
// Warning status
#define CANSTAT_EWARN				(0x06)
#define CANSTAT_EWARN_Msk			(1<<CANSTAT_EWARN)
// Busoff status
#define CANSTAT_BOFF				(0x07)
#define CANSTAT_BOFF_Msk			(1<<CANSTAT_BOFF)


// LPC_CAN->EC
// CAN error counter
// Transmit error counter
#define CANEC_TEC					(0x00)
#define CANEC_TEC_Msk				(0xff<<CANEC_TEC)
// Receive error counter
#define CANEC_REC					(0x08)
#define CANEC_REC_Msk				(0xff<<CANEC_REC)
// Receive error passive
#define CANEC_RP					(0x0f)
#define CANEC_RP_Msk				(1<<CANEC_RP)


// LPC_CAN->BT
// CAN bit timing register
// Baud rate prescaler
#define CANBT_BRP					(0x00)
#define CANBT_BRP_Msk				(0b111111<<CANBT_BRP)
// (Re)synchronization jump width
#define CANBT_SJW					(0x06)
#define CANBT_SJW_Msk				(0b11<<CANBT_SJW)
// Time segment before the sample point
#define CANBT_TSEG1					(0x08)
#define CANBT_TSEG1_Msk				(0b1111<<CANBT_TSEG1)
// Time segment after the sample point
#define CANBT_TSEG2					(0x0c)
#define CANBT_TSEG2_Msk				(0b111<<CANBT_TSEG2)



// LPC_CAN->TEST
// CAN test register
// Basic mode
#define CANTEST_BASIC				(0x02)
#define CANTEST_BASIC_Msk			(1<<CANTEST_BASIC)
// Silent mode
#define CANTEST_SILENT				(0x03)
#define CANTEST_SILENT_Msk			(1<<CANTEST_SILENT)
// Loop back mode
#define CANTEST_LBACK				(0x04)
#define CANTEST_LBACK_Msk			(1<<CANTEST_LBACK)
// Control of CAN_TXD pins
#define CANTEST_TX					(0x05)
#define CANTEST_TX_Msk				(0b11<<CANTEST_TX)
// Monitors the actual value of the CAN_RXD pin.
#define CANTEST_RX					(0x07)
#define CANTEST_RX_Msk				(1<<CANTEST_RX)


// LPC_CAN->BRPE
// CAN baud rate prescaler extension register
// Baud rate prescaler extension
#define CANBRPE_BRPE				(0x00)
#define CANBRPE_BRPE_Msk			(0b1111<<CANBRPE_BRPE)


// LPC_CAN->IF1_CMDREQ
// LPC_CAN->IF2_CMDREQ
// CAN message interface command request registers
#define CANIF_CMDREQ_MN				(0x00)
#define CANIF_CMDREQ_MN_Msk			(0b111111<<CANIF_CMDREQ)
// BUSY flag
#define CANIF_CMDREQ_BUSY			(0x0a)
#define CANIF_CMDREQ_BUSY_Msk		(1<<CANIF_BUSY)


// LPC_CAN->IF1_CMDMSK
// LPC_CAN->IF2_CMDMSK
// CAN message interface command mask registers
// Access data bytes 4-7
#define CANIF_CMDMSK_DATA_B			(0x00)
#define CANIF_CMDMSK_DATA_B_Msk		(1<<CANIF_CMDMSK_DATA_B)
// Access data bytes 0-3
#define CANIF_CMDMSK_DATA_A			(0x01)
#define CANIF_CMDMSK_DATA_A_Msk		(1<<CANIF_CMDMSK_DATA_A)
// Access transmission request bit
#define CANIF_CMDMSK_TXRQST			(0x02)
#define CANIF_CMDMSK_TXRQST_Msk		(1<<CANIF_CMDMSK_TXRQST)
// This bit is ignored in the write direction
#define CANIF_CMDMSK_CLRINTPND		(0x03)
#define CANIF_CMDMSK_CLRINTPND_Msk	(1<<CANIF_CMDMSK_CLRINTPND)
// Access new data bit
#define CANIF_CMDMSK_NEWDAT			(0x02)
#define CANIF_CMDMSK_NEWDAT_Msk		(1<<CANIF_CMDMSK_NEWDAT)
// Access control bits
#define CANIF_CMDMSK_CTRL			(0x04)
#define CANIF_CMDMSK_CTRL_Msk		(1<<CANIF_CMDMSK_CTRL)
// Access arbitration bits
#define CANIF_CMDMSK_ARB			(0x05)
#define CANIF_CMDMSK_ARB_Msk		(1<<CANIF_CMDMSK_ARB)
// Access mask bits
#define CANIF_CMDMSK_MASK			(0x06)
#define CANIF_CMDMSK_MASK_Msk		(1<<CANIF_CMDMSK_MASK)
// Write transfer
#define CANIF_CMDMSK_WR_RD			(0x07)
#define CANIF_CMDMSK_WR_RD_Msk		(1<<CANIF_CMDMSK_WR_RD)


// LPC_CAN->IF1_MSK1
// LPC_CAN->IF2_MSK1
// CAN message interface command mask 1 registers
// Identifier mask 0-15 bit
#define CANIF_MSK1_MSK				(0x00)
#define CANIF_MSK1_MSK_Msk			(0xffff<<CANIF_MSK1_MSK)

// LPC_CAN->IF1_MSK2
// LPC_CAN->IF2_MSK2
// Identifier mask 16-28 bit
#define CANIF_MSK2_MSK				(0x00)
#define CANIF_MSK2_MSK_Msk			(0b1111111111111<<CANIF_MSK2_MSK)
// Mask message direction
#define CANIF_MSK2_MDIR				(0x0e)
#define CANIF_MSK2_MDIR_Msk			(1<<ANIF_MSK2_MDIR)
// Mask extend identifier
#define CANIF_MSK2_MXTD				(0x0f)
#define CANIF_MSK2_MXTD_Msk			(1<<CANIF_MSK2_MXTD)

// LPC_CAN->IF1_ARB1
// LPC_CAN->IF2_ARB1
// CAN message interface command arbitration 1 registers
// Message identifier 0-15 bit
#define CANIF_ARB1_ID				(0x00)
#define CANIF_ARB1_ID_Msk			(0xffff<<CANIF_ARB1_ID)

// LPC_CAN->IF1_ARB2
// LPC_CAN->IF2_ARB2
// Message identifier 16-28 bit
#define CANIF_ARB2_ID				(0x00)
#define CANIF_ARB2_ID_Msk			(0b1111111111111<<CANIF_ARB2_ID)
// Message direction
#define CANIF_ARB2_DIR				(0x0d)
#define CANIF_ARB2_DIR_Msk			(1<<CANIF_ARB2_DIR)
// Extend identifier
#define CANIF_ARB2_XTD				(0x0e)
#define CANIF_ARB2_XTD_Msk			(1<<CANIF_ARB2_XTD)
// Message valid
#define CANIF_ARB2_MSGVAL			(0x0f)
#define CANIF_ARB2_MSGVAL_Msk		(1<<CANIF_ARB2_MSGVAL)

// LPC_CAN->IF1_MCTRL
// LPC_CAN->IF2_MCTRL
// CAN message interface message control registers
// Data length code
#define CANIF_MCTRL_DLC				(0x00)
#define CANIF_MCTRL_DLC_Msk			(0b1111<<CANIF_MCTRL_DLC)
// End of buffer
#define CANIF_MCTRL_EOB				(0x07)
#define CANIF_MCTRL_EOB_Msk			(1<<CANIF_MCTRL_EOB)
// Transmit request
#define CANIF_MCTRL_TXRQST			(0x08)
#define CANIF_MCTRL_TXRQST_Msk		(1<<CANIF_MCTRL_TXRQST)
// Remote enable
#define CANIF_MCTRL_RMTEN			(0x09)
#define CANIF_MCTRL_RMTEN_Msk		(1<<CANIF_MCTRL_RMTEN)
// Receive interrupt enable
#define CANIF_MCTRL_RXIE			(0x0a)
#define CANIF_MCTRL_RXIE_Msk		(1<<CANIF_MCTRL_RXIE)
// Transmit interrupt enable
#define CANIF_MCTRL_TXIE			(0x0b)
#define CANIF_MCTRL_TXIE_Msk		(1<<CANIF_MCTRL_TXIE)
// Use acceptance mask
#define CANIF_MCTRL_UMASK			(0x0c)
#define CANIF_MCTRL_UMASK_Msk		(1<<CANIF_MCTRL_UMASK)
// Interrupt pending
#define CANIF_MCTRL_INTPND			(0x0d)
#define CANIF_MCTRL_INTPND_Msk		(1<<CANIF_MCTRL_INTPND)
// Message lost (only valid for message objects in the direction receive)
#define CANIF_MCTRL_MSGLST			(0x0e)
#define CANIF_MCTRL_MSGLST_Msk		(1<<CANIF_MCTRL_MSGLST)
// New data
#define CANIF_MCTRL_NEWDAT			(0x0f)
#define CANIF_MCTRL_NEWDAT_Msk		(1<<CANIF_MCTRL_NEWDAT)


// LPC_WDT->MOD
// Watchdog Mode register

// WDEN Watchdog enable bit (Set Only).
#define WDTMOD_WDEN					(0x00)
#define WDTMOD_WDEN_Msk				(1<<WDTMOD_WDEN)
// WDRESET Watchdog reset enable bit (Set Only).
#define WDTMOD_WDRESET				(0x01)
#define WDTMOD_WDRESET_Msk			(1<<WDTMOD_WDRESET)
// WDTOF Watchdog time-out flag.
#define WDTMOD_WDTOF				(0x02)
#define WDTMOD_WDTOF_Msk			(1<<WDTMOD_WDTOF)
// WDINT Watchdog interrupt flag (read only)
#define WDTMOD_WDINT				(0x03)
#define WDTMOD_WDINT_Msk			(1<<WDTMOD_WDINT)

// LPC_SYSCON->WDTOSCCTRL
// Watchdog oscillator control register

// Select divider for Fclkana.
//  wdt_osc_clk = Fclkana/ (2 x (1 + DIVSEL))
#define WDTOSCCTRL_DIVSEL			(0x00)
#define WDTOSCCTRL_DIVSEL_Msk		(0b11111<<WDTOSCCTRL_DIVSEL)
// Select watchdog oscillator analog output frequency (Fclkana)
//  0x1 0.6 MHz   0x2 1.05 MHz   0x3 1.4 MHz   0x4 1.75 MHz
//  0x5 2.1 MHz   0x6 2.4 MHz    0x7 2.7 MHz   0x8 3.0 MHz
//  0x9 3.25 MHz  0xA 3.5 MHz    0xB 3.75 MHz  0xC 4.0 MHz
//  0xD 4.2 MHz   0xE 4.4 MHz    0xF 4.6 MHz
#define WDTOSCCTRL_FREQSEL			(0x05)
#define WDTOSCCTRL_FREQSEL_Msk		(0b1111<<WDTOSCCTRL_FREQSEL)





// LPC_SYSCON->SYSRSTSTAT
// System reset status register

// POR reset status (Writing a one clears this reset)
#define SYSRSTSTAT_POR				(0x00)
#define SYSRSTSTAT_POR_Msk			(1<<SYSRSTSTAT_POR)
// Status of the external RESET pin
#define SYSRSTSTAT_EXTRST			(0x01)
#define SYSRSTSTAT_EXTRST_Msk		(1<<SYSRSTSTAT_EXTRST)
// Status of the Watchdog reset
#define SYSRSTSTAT_WDT				(0x02)
#define SYSRSTSTAT_WDT_Msk			(1<<SYSRSTSTAT_WDT)
// Status of the Brown-out detect reset
#define SYSRSTSTAT_BOD				(0x03)
#define SYSRSTSTAT_BOD_Msk			(1<<SYSRSTSTAT_BOD)
// Status of the software system reset
#define SYSRSTSTAT_SYSRST			(0x04)
#define SYSRSTSTAT_SYSRST_Msk		(1<<SYSRSTSTAT_SYSRST)


// LPC_SYSCON->BODCTRL
// BOD control register

// BOD reset level 00
//  0x0 Level 0: The reset voltage is 1.46 V/1.63 V.
//  0x1 Level 1: The reset voltage is 2.06 V/2.15 V.
//  0x2 Level 2: The reset voltage is 2.35 V/2.43 V.
//  0x3 Level 3: The reset voltage is 2.63 V/2.71 V.
#define BODCTRL_BODRSTLEV			(0x00)
#define BODCTRL_BODRSTLEV_Msk		(0b11<<BODCTRL_BODRSTLE)

// BOD interrupt level
//  0x0 Level 0: Reserved.
//  0x1 Level 1: The interrupt voltage is 2.22 V/2.35 V.
//  0x2 Level 2: The interrupt voltage is 2.52 V/2.66 V.
//  0x3 Level 3: The interrupt voltage is 2.80 V/2.90 V.
#define BODCTRL_BODINTVAL			(0x02)
#define BODCTRL_BODINTVAL_Msk		(0b11<<BODCTRL_BODINTVAL)

// BOD reset enable
#define BODCTRL_BODRSTENA			(0x04)
#define BODCTRL_BODRSTENA_Msk		(1<<BODCTRL_BODRSTENA)

// LPC_SYSCON->PRESETCTRL
// Peripheral reset control register

// SPI0 reset control
#define PRESETCTRL_SSP0_RST_N		(0x00)
#define PRESETCTRL_SSP0_RST_N_Msk	(1<<PRESETCTRL_SSP0_RST_N)

// I2C reset control
#define PRESETCTRL_I2C_RST_N		(0x01)
#define PRESETCTRL_I2C_RST_N_Msk	(1<<PRESETCTRL_I2C_RST_N)

// SPI1 reset control
#define PRESETCTRL_SSP1_RST_N		(0x02)
#define PRESETCTRL_SSP1_RST_N_Msk	(1<<PRESETCTRL_SSP1_RST_N)

// C_CAN reset control
#define PRESETCTRL_CAN_RST_N		(0x03)
#define PRESETCTRL_CAN_RST_N_Msk	(1<<PRESETCTRL_CAN_RST_N)

// LPC_PMU->PCON
// Power control register

// Deep power-down mode enable
#define PCON_DPDEN				(0x01)
#define PCON_DPDEN_Msk			(1<<PCON_DPDEN)

// Sleep mode flag
#define PCON_SLEEPFLAG			(0x08)
#define PCON_SLEEPFLAG_Msk		(1<<PCON_SLEEPFLAG)

// Deep power-down flag
#define PCON_DPDFLAG			(0x0b)
#define PCON_DPDFLAG_Msk		(1<<PCON_DPDFLAG)

// LPC_PMU->GPREG4
// General purpose register 4
#define GPREG4_WAKEUPHYS		(0x0a)
#define GPREG4_WAKEUPHYS_Msk	(1<<GPREG4_WAKEUPHYS)


// LPC_I2C->CONSET (I2C0CONSET)
// I2C Control Set register
// LPC_I2C->CONCLR (I2C0CONCLR)
// I2C Control Clear register
// 	Writing a one to a bit of this register causes the corresponding bit in the
//		I2C control register to be set

// Assert acknowledge flag.
#define I2CCON_AA				(0x02)
#define I2CCON_AA_Msk			(1<<I2CCON_AA)

// I2C interrupt flag.
#define I2CCON_SI				(0x03)
#define I2CCON_SI_Msk			(1<<I2CCON_SI)

// STOP flag. (do not use for clear)
#define I2CCON_STO				(0x04)
#define I2CCON_STO_Msk			(1<<I2CCON_STO)

// START flag.
#define I2CCON_STA				(0x05)
#define I2CCON_STA_Msk			(1<<I2CCON_STA)

// I2C interface enable
#define I2CCON_I2EN				(0x06)
#define I2CCON_I2EN_Msk			(1<<I2CCON_I2EN)


// LPC_I2C->STAT (I2C0STAT)
// I2C Status register

#define I2CSTAT_STATUS			(0x03)
#define I2CSTAT_STATUS_Msk		(0b111111<<I2CSTAT_STATUS)


// LPC_I2C->DAT
// I2C Data register

#define I2CDAT_DATA				(0x00)
#define I2CDAT_DATA_Msk			(0xff<<I2CDAT_DATA)


// LPC_I2C->ADR0/1/2/3
// I2C Slave Address register

// General Call enable bit.
#define I2CADR_GC				(0x00)
#define I2CADR_GC_Msk			(1<<I2CADR_GC)

// The I2C device address for slave mode.
#define I2CADR_ADDRESS			(0x01)
#define I2CADR_ADDRESS_Msk		(0b1111111<<I2CADR_ADDRESS)


// LPC_I2C->MMCTRL
// I2C Monitor mode control register

// Monitor mode enable.
#define I2CMMCTRL_MM_ENA		(0x00)
#define I2CMMCTRL_MM_ENA_Msk	(1<<I2CMMCTRL_MM_ENA)

// SCL output enable.
#define I2CMMCTRL_ENA_SCL		(0x01)
#define I2CMMCTRL_ENA_SCL_Msk	(1<<I2CMMCTRL_ENA_SCL)

// Select interrupt register match.
#define I2CMMCTRL_MATCH_ALL		(0x02)
#define I2CMMCTRL_MATCH_ALL_Msk	(1<<I2CMMCTRL_MATCH_ALL)
