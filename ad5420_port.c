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

#include "ad5420_port.h"
#include "spi.h"   // <-- your spi header

/* init function */
void ad5420_init() {
	AD5420_CLEAR_DIR |= (1 << AD5420_CLEAR_BIT);
	AD5420_LATCH_DIR |= (1 << AD5420_LATCH_BIT);
	AD5420_FAULT_DIR &=~(1 << AD5420_FAULT_BIT);
	AD5420_LATCH_OUT &=~(1 << AD5420_LATCH_BIT);
	AD5420_CLEAR_OUT &=~(1 << AD5420_CLEAR_BIT);
}

uint8_t ad5420_spi_wrap(uint8_t data) {
	return spi_read(data);
}