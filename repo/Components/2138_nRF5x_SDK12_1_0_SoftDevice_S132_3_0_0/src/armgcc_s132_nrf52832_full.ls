/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  S132 (rx)  : ORIGIN = 0x00000,    LENGTH = 0x1f000
  FLASH (rx) : ORIGIN = 0x1f000,    LENGTH = 0x61000
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x10000
}

SECTIONS
{
  .soft_device : {
      __soft_device_start__ = .;
      KEEP (*(.soft_device))
      __soft_device_end__ = .;
  } > S132
    
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf52_common.ls"
