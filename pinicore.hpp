/**
* @file		pinicore.h
* @brief	Easy include PiniCore library.
* @author	PiniponSelvagem
*
* Copyright(C) PiniponSelvagem
*
***********************************************************************
* Software that is described here, is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
**********************************************************************/

#pragma once

/**
 * TODO:
 * - Add support for Arduino boards, currently only compiles on ESP-IDF based ones.
 */

/**
 * Useful website to better organize a C struct:
 * @see https://padding-split.vercel.app/
 */

/**
  * Dependencies:
  * lib_deps = 
  *     vshymanskyy/TinyGSM@^0.12
  *     https://github.com/alkonosst/SSLClientESP32.git#v2.0.3
  *     arduino-libraries/ArduinoHttpClient@^0.6.1
  */
#ifndef _PINICORE_H_
#define _PINICORE_H_

#include "utils/log.hpp"
#include "utils/time.hpp"
#include "utils/watchdog.hpp"
#include "utils/memory.hpp"
#include "utils/calculation.hpp"

#include "system/board.hpp"
#include "system/interrupt.hpp"

#include "drivers/io/button/button.hpp"
#include "drivers/io/led/led.hpp"
#include "drivers/io/rotaryencoder/rotaryencoder.hpp"

#include "drivers/io/relays/irelays.hpp"
#include "drivers/io/relays/rl_virtual.hpp"
#include "drivers/io/relays/rl_ts.hpp"
#include "drivers/io/relays/rl_x16blue.hpp"

#include "drivers/sensors/lm35.hpp"
#include "drivers/sensors/dht.hpp"

#include "drivers/communication/lora.hpp"

#include "drivers/energy/battery.hpp"

#include "storage/storage.hpp"

#include "communication/network/inetwork.hpp"
#include "communication/network/wifi.hpp"
#include "communication/network/mobile.hpp"

#include "communication/network/request/ota/iota.hpp"
#include "communication/network/request/ota/ota_ts.hpp"

#include "communication/network/request/mqtt/mqtt.hpp"

#include "communication/radio/loracomm.hpp"

#endif /* _PINICORE_H_ */
