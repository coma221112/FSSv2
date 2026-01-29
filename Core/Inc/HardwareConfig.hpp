/*
 * HardwareConfig.hpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_HARDWARECONFIG_HPP_
#define INC_HARDWARECONFIG_HPP_

#include "CS1237.hpp"

volatile CS1237 sg0(PA1,PA0);
volatile CS1237 sg1(PA3,PA2);
volatile CS1237 sg2(PA5,PA4);



#endif /* INC_HARDWARECONFIG_HPP_ */
