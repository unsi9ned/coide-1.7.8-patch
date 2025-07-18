/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x20000000, LENGTH = 0x08000
  RAM (rwx) :  ORIGIN = 0x20008000, LENGTH = 0x08000
}


INCLUDE "nrf52_common.ls"
