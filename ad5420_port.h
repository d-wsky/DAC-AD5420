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

#ifndef AD5420_PORT_H_
#define AD5420_PORT_H_

#include <avr/io.h>
#include <util/delay.h>

/*! \brief Pin of the CLEAR signal.*/
#define AD5420_CLEAR_BIT  PB4
/*! \brief Pin of the LATCH signal.*/
#define AD5420_LATCH_BIT  PB5
/*! \brief Pin of the FAULT signal.*/
#define AD5420_FAULT_BIT  PB0
/*! \brief Input register name of the CLEAR signal.*/
#define AD5420_CLEAR_IN   PINB
/*! \brief Input register name of the LATCH signal.*/
#define AD5420_LATCH_IN   PINB
/*! \brief Input register name of the FAULT signal.*/
#define AD5420_FAULT_IN   PINB
/*! \brief Direction register name of the CLEAR signal.*/
#define AD5420_CLEAR_DIR  DDRB
/*! \brief Direction register name of the LATCH signal.*/
#define AD5420_LATCH_DIR  DDRB
/*! \brief Direction register name of the FAULT signal.*/
#define AD5420_FAULT_DIR  DDRB
/*! \brief Output register name of the CLEAR signal.*/
#define AD5420_CLEAR_OUT  PORTB
/*! \brief Output register name of the LATCH signal.*/
#define AD5420_LATCH_OUT  PORTB
/*! \brief Output register name of the FAULT signal.*/
#define AD5420_FAULT_OUT  PORTB

/*! \brief Shorthand macro for setting LATCH pin HIGH.*/
#define AD5420_LATCH_HIGH  AD5420_LATCH_OUT |= (1 << AD5420_LATCH_BIT)
/*! \brief Shorthand macro for setting LATCH pin LOW.*/
#define AD5420_LATCH_LOW   AD5420_LATCH_OUT &=~(1 << AD5420_LATCH_BIT)
/*! \brief Shorthand macro for a 10 us delay required by write to control
register of DAC.*/
#define AD5420_LATCH_CONTROL_WRITE_DELAY   _delay_us(10)
/*! \brief Shorthand macro for setting CLEAR pin HIGH.*/
#define AD5420_CLEAR_HIGH  AD5420_CLEAR_OUT |= (1 << AD5420_CLEAR_BIT)
/*! \brief Shorthand macro for setting CLEAR pin LOW.*/
#define AD5420_CLEAR_LOW   AD5420_CLEAR_OUT &=~(1 << AD5420_CLEAR_BIT)

/*! \brief Totally portable function that initialize discrete in-out pins
of uC.*/
extern void ad5420_init();

/*! \brief This is a wrap function to user's realization of a read-write cycle
of 1 byte via SPI interface. Bit order is from MSB to LSB.

\param data Byte that would be sent to SPI bus.

\return Byte that was read back from SPI bus while sending data.*/
extern uint8_t ad5420_spi_wrap(uint8_t data);

#endif /* AD5420_PORT_H_ */