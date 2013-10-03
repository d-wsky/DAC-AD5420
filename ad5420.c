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

#include "ad5420.h"
#include "spi.h"

/*! \brief Internal address of DAC CONTROL register.*/
#define AD5420_CONTROL_REG_ADDR  0x55UL
/*! \brief Internal address of DAC DATA register.*/
#define AD5420_DATA_REG_ADDR     0x01UL
/*! \brief Internal address of DAC RESET register.*/
#define AD5420_RESET_REG_ADDR    0x56UL

/*! \brief This type is specifically defined because almost
all transmissions between DAC and uC are 24-bit long.*/
typedef uint32_t _uint24_t;

_uint24_t ad5420_write(_uint24_t data) {
	_uint24_t result = 0;
	// write highest byte
	result |=  ad5420_spi_wrap((uint8_t)(data >> 16));
	result <<= 8;
	// write middle byte
	result |=  ad5420_spi_wrap((uint8_t)(data >> 8));
	result <<= 8;
	// write lowest byte
	result |= ad5420_spi_wrap((uint8_t)(data));
	
	return result;
}

void ad5420_write_ctrl(uint16_t data) {
	// clear unused bits
	uint16_t d = (data & 0x3FFF);
	// write data
	ad5420_write((AD5420_CONTROL_REG_ADDR << 16)+d);
	AD5420_LATCH_HIGH;
	AD5420_LATCH_CONTROL_WRITE_DELAY;
	AD5420_LATCH_LOW;
}

void ad5420_write_data(uint16_t data) {
	ad5420_write((AD5420_DATA_REG_ADDR << 16) + data);
	AD5420_LATCH_HIGH;
	AD5420_LATCH_LOW;
}

void ad5420_set_slewrate(uint16_t sr_clock, uint16_t sr_step, uint8_t sr_enabled) {
	_uint24_t ctrl_reg = ad5420_read_reg(AD5420_CONTROL_REG);
	if (sr_enabled) {
		ctrl_reg &=~0x000FE0;
		ctrl_reg |= (sr_clock | sr_step | AD5420_SREN);
	}	
	else
		ctrl_reg &=~(AD5420_SREN);
	ad5420_write_ctrl((uint16_t)ctrl_reg);
}

uint16_t ad5420_read_reg(uint8_t reg_name) {
	// write read register command with reg name at the end of the 3rd byte
	ad5420_write(0x020000+reg_name);
	AD5420_LATCH_HIGH;
	AD5420_LATCH_LOW;
	// now register data would appear in next
	// three bytes
	_uint24_t r = ad5420_write(0x000000);
	AD5420_LATCH_HIGH;
	AD5420_LATCH_LOW;
	return (uint16_t)r;
}

uint8_t ad5420_get_status() {
	uint8_t r = ad5420_read_reg(AD5420_STATUS_REG);
	return r;
}

void ad5420_clear() {
	AD5420_CLEAR_HIGH;
	AD5420_CLEAR_LOW;
}

void ad5420_reset() {
	ad5420_write((AD5420_RESET_REG_ADDR << 16) + 1);
	AD5420_LATCH_HIGH;
	AD5420_LATCH_CONTROL_WRITE_DELAY;
	AD5420_LATCH_LOW;
}