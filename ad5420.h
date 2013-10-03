//***************************************************************************//
//
// COPYRIGHT NOTICE
//
// (c) Denis Vasilkovsky - d.wsky@yandex.ru
//
//  This software is provided 'as-is', without any express or implied
//  warranty.  In no event will the authors be held liable for any damages
//  arising from the use of this software.
//
//  Permission is granted to anyone to use this software for any purpose,
//  including commercial applications, and to alter it and redistribute it
//  freely, subject to the following restrictions:
//
//  1. The origin of this software must not be misrepresented; you must not
//     claim that you wrote the original software. If you use this software
//     in a product, an acknowledgment in the product documentation would be
//     appreciated but is not required.
//  2. Altered source versions must be plainly marked as such, and must not be
//     misrepresented as being the original software.
//  3. This notice may not be removed or altered from any source distribution.
//
//***************************************************************************//

/*! \file ad5420.h

\brief Header file of AD5420 driver.

\version 1.0

\date 10/03/13

The AD5410/AD5420 are low cost, precision, fully integrated 12-/16-bit
converters offering a programmable current source output designed to meet
the requirements of industrial process control applications. The output
current range is programmable at 4 mA to 20 mA, 0 mA to 20 mA, or an
overrange function of 0 mA to 24 mA. The output is open-circuit protected.
The device operates with a power supply (AV DD ) range from 10.8 V to 60 V.
Output loop compliance is 0 V to AV DD  ? 2.5 V. The flexible serial interface
is SPI, MICROWIRE™, QSPI™, and DSP compatible and can be operated in 3-wire
mode to minimize the digital isolation required in isolated applications. The
device also includes a power-on reset function, ensuring that the device powers
up in a known state, and an asynchronous CLEAR pin that sets the output to the
low end of the selected current range. The total unadjusted error is typically
±0.01% FSR.

The AD5410/AD5420 are controlled over a versatile 3-wire 
serial interface that operates at clock rates of up to 30 MHz. Data is loaded
into the device MSB first as a 24-bit word under the control of a serial clock
input, SCLK. Data is clocked in on the rising edge of SCLK. The input shift
register consists of eight address bits and 16 data bits, as shown in Table 6.
The 24-bit word is unconditionally latched on the rising edge of LATCH. Data
continues to be clocked in irrespective of the state of LATCH. On the rising
edge of LATCH, the data that is present in the input shift register is latched;
that is, the last 24 bits to be clocked in before the rising edge of LATCH is
the data that is latched. For systems that contain several devices, the SDO pin
can be used to daisy-chain several devices together.

From SPI point of view DAC has 3 internal registers which can be accessed
through read and write commands. CONTROL register set output parameters,
DATA register stores output code, STATUS register is read-only and is used to
check whether there some faults on DAC operation.

Using this library may ease programming process to end user. Before start
diving into embedding this code in your application first take a look in
ad5420_port.h and ad5420_port.c file to make them consistent with your uC ports
and registers as well as with your scheme. This is a simple AVR Mega example
that works pretty fine in next configuration:

           Mega               AD5420
        PB0        <--------> FAULT
		PB1 (SCK)  <--------> SCK
		PB2 (MOSI) <--------> SDIN
		PB3 (MISO) <--------> SDO
		PB4        <--------> CLEAR
		PB5        <--------> LATCH
		
In case you are using Mega or other AVR-classic devices try editing
AD5420_xxx_BIT,AD5420_xxx_OUT and other macros to make them compatible with
your scheme. If you are using uC of other manufacturer try editing
AD5420_xxx_HIGH and AD5420_xxx_LOW macros as well. And you must make
#AD5420_LATCH_CONTROL_WRITE_DELAY consistent with your code for delay for
5 to 10 usec.

Once you've done it, you can now make some coode to start working with DAC.
This part would be pretty easy:

     <code>
	 ad5420_init();
	 ad5420_reset();
	 ad5420_write_ctrl(AD5420_OUTEN | AD5420_REXT | AD5420_4_20_RANGE);
	 ad5420_write_data(0x8000);
	 </code>
	 
This code configures GPIO pins to work with AD5420, sends a reset command
to it and configures AD5420 to work with external reference R in 4 to 20 mA
range. You can use here some other macros here, check out description of
ad5420_write_ctrl(). Last statement sets output current to the middle of the
output range.

This DAC also has an automatic slew rate control so you can vary rate at
which it changes output current. In very wide range between 2 msec and 20
sec (sic!). This thing might be done either with ad5420_write_ctrl() function
or with specific ad5420_set_slewrate() function. E.x.:

     <code>
	 ad5420_set_slewrate(AD5420_SR_CLK_3300, AD5420_SR_STEPSIZE_1, 1);
	 </code>
	 
sets the lowest slewrate of output current. 

If you've forgot your current DAC configuration and output code you can get
them through ad5420_read_reg() function which returns DAC CONTROL, STATUS or
DATA register value.
*/



#ifndef AD5420_H_
#define AD5420_H_

#include "ad5420_port.h"

/*! \brief Internal address of DAC STATUS register.*/
#define AD5420_STATUS_REG        0x00
/*! \brief Internal address of DAC DATA register.*/
#define AD5420_DATA_REG          0x01
/*! \brief Internal address of DAC CONTROL register.*/
#define AD5420_CONTROL_REG       0x02

/*! \brief Internal flag of using REXT as reference resistance. Please set this
bit before or during changing #AD5420_OUTEN bit.*/
#define AD5420_REXT              0x2000
/*! \brief Internal flag of switching DAC on.*/
#define AD5420_OUTEN             0x1000
/*! \brief Internal flag of switching slew rate control on.*/
#define AD5420_SREN              0x0010
/*! \brief Internal flag of setting daisy-chain mode of SPI bus.*/
#define AD5420_DCEN              0x0008
/*! \brief Internal flag of setting 4-20 mA output range.*/
#define AD5420_4_20_RANGE        0x0005
/*! \brief Internal flag of setting 0-20 mA output range.*/
#define AD5420_0_20_RANGE        0x0006
/*! \brief Internal flag of setting 0-24 mA output range.*/
#define AD5420_0_24_RANGE        0x0007

/*! \brief Internal flag of output current fault in status register.*/
#define AD5420_IOUT_FAULT        0x0004
/*! \brief Internal flag of slew rate being active in status register.*/
#define AD5420_SR_ACTIVE         0x0002
/*! \brief Internal flag of overheat fault of DAC in status register.*/
#define AD5420_OVRHEAT_FAULT     0x0001


/******************************************************************************
**                                                                           **
**      Set of slew rate control defines                                     **
**                                                                           **
******************************************************************************/

/*! \brief Bit value that sets slewrate to 257,730 Hz.*/
#define AD5420_SR_CLK_257730     0x0000
/*! \brief Bit value that sets slewrate to 198,410 Hz.*/
#define AD5420_SR_CLK_198410     0x0100
/*! \brief Bit value that sets slewrate to 152,440 Hz.*/
#define AD5420_SR_CLK_152440     0x0200
/*! \brief Bit value that sets slewrate to 131,580 Hz.*/
#define AD5420_SR_CLK_131580     0x0300
/*! \brief Bit value that sets slewrate to 115,740 Hz.*/
#define AD5420_SR_CLK_115740     0x0400
/*! \brief Bit value that sets slewrate to 69,440 Hz.*/
#define AD5420_SR_CLK_69440      0x0500
/*! \brief Bit value that sets slewrate to 37,590 Hz.*/
#define AD5420_SR_CLK_37590      0x0600
/*! \brief Bit value that sets slewrate to 25,770 Hz.*/
#define AD5420_SR_CLK_25770      0x0700
/*! \brief Bit value that sets slewrate to 20,160 Hz.*/
#define AD5420_SR_CLK_20160      0x0800
/*! \brief Bit value that sets slewrate to 16,030 Hz.*/
#define AD5420_SR_CLK_16030      0x0900
/*! \brief Bit value that sets slewrate to 10,290 Hz.*/
#define AD5420_SR_CLK_10290      0x0A00
/*! \brief Bit value that sets slewrate to 8,280 Hz.*/
#define AD5420_SR_CLK_8280       0x0B00
 /*! \brief Bit value that sets slewrate to 6,900 Hz.*/
#define AD5420_SR_CLK_6900       0x0C00
/*! \brief Bit value that sets slewrate to 5,530 Hz.*/
#define AD5420_SR_CLK_5530       0x0D00
/*! \brief Bit value that sets slewrate to 4,240 Hz.*/
#define AD5420_SR_CLK_4240       0x0E00
/*! \brief Bit value that sets slewrate to 3,300 Hz.*/
#define AD5420_SR_CLK_3300       0x0F00

/*! \brief Bit value that sets slewrate step to 1 LSB.*/
#define AD5420_SR_STEPSIZE_1     0x0000
/*! \brief Bit value that sets slewrate step to 2 LSB.*/
#define AD5420_SR_STEPSIZE_2     0x0020
/*! \brief Bit value that sets slewrate step to 4 LSB.*/
#define AD5420_SR_STEPSIZE_4     0x0040
/*! \brief Bit value that sets slewrate step to 8 LSB.*/
#define AD5420_SR_STEPSIZE_8     0x0060
/*! \brief Bit value that sets slewrate step to 16 LSB.*/
#define AD5420_SR_STEPSIZE_16    0x0080
/*! \brief Bit value that sets slewrate step to 32 LSB.*/
#define AD5420_SR_STEPSIZE_32    0x00A0
/*! \brief Bit value that sets slewrate step to 64 LSB.*/
#define AD5420_SR_STEPSIZE_64    0x00C0
/*! \brief Bit value that sets slewrate step to 128 LSB.*/
#define AD5420_SR_STEPSIZE_128   0x00E0


/*! \brief This function resets DAC to it's initial state by sending a specific
command to it.*/
extern void ad5420_reset();

/*! \brief This function sets output of the DAC to it's minimum value using
CLEAR pin.*/
extern void ad5420_clear();

/*! \brief Writes data to DAC. This function changes sets output current.

\param data - 16-bit data to be written to DAC.*/
extern void ad5420_write_data(uint16_t data);

/*! \brief Writes control bits to DAC.

\param data - data to be written to DAC. Possible value is a logical OR
of one or more of next values:

    - #AD5420_REXT determines whether external resistor is used to make
	               a zero offset;
	- #AD5420_OUTEN determines whether output current is enabled;
	- #AD5420_SREN determines whether output slew rate limits are enabled;
	- one of #AD5420_0_20_RANGE, #AD5420_4_20_RANGE, #AD5420_0_24_RANGE
	  to set the output current range.

This function defines current state of the DAC CONTROL register. Slew rate
value is determined by ad5420_set_slewrate() function. But you can do it
in one piece if you add #AD5420_SREN flag and some of these:
AD5420_SR_CLK_xxx and AD5420_SR_STEPSIZE_xxx.*/
extern void ad5420_write_ctrl(uint16_t data);

/*! \brief This function sets parameters of DAC slewrate control.

\param sr_clock Sets clock rate of update and must be one of the
       AD5420_SR_CLK_xxx macros.
\param sr_step Sets update step of DAC output value. Must be one of the
       AD5420_SR_STEPSIZE_xxx macros.
\param sr_enabled This param determines whether the slewrate function is
       enabled.
	   
\warning Setting input parameters to lowest value makes output to change very
slowly. Time that is need to switch between max and min codes is varied from
about 1.5 ms (without slewrate control) to about 15 s (with slowest slew
rate).*/
extern void ad5420_set_slewrate(uint16_t sr_clock, uint16_t sr_step,
                                uint8_t sr_enabled);

/*! \brief This function reads DAC internal register.

\param reg_name - Name of the reading register. Must be one of the follows:

    - AD5420_STATUS_REG to read STATUS register;
	- AD5420_CONTROL_REG to read CONTROL register;
	- AD5420_DATA_REG to read DATA register.
	
\return Returns 16-bit data that had been read.*/
extern uint16_t ad5420_read_reg(uint8_t reg_name);


/*! \brief This function gets status bits.

\return Returns 8-bit int, last 3 bits of which is used as status flags. To
work with them #AD5420_IOUT_FAULT, #AD5420_SR_ACTIVE, #AD5420_OVRHEAT_FAULT
macros are provided.*/
extern uint8_t ad5420_get_status();



#endif /* AD5420_H_ */