#ifndef POSES
#define POSES

#include <avr/pgmspace.h>

const PROGMEM uint16_t Center[] = {2048, 2048, 2048, 2048, 512, 512};
const PROGMEM uint16_t Home[] = {2033, 1698, 1448, 2336, 512, 512};
const PROGMEM uint16_t Rest[] = {2048, 1020, 1030, 2048, 512, 512};
const PROGMEM uint16_t PickNDropHome[] = {2048, 2048, 2048, 1024, 512, 512};
const PROGMEM uint16_t PickNDropTrash[] = {3072, 2048, 2048, 1024, 512, 512};

#endif