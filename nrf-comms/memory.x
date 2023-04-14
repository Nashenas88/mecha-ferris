MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* These values correspond to the NRF52840 with Softdevices S140 7.3.0 */
  FLASH : ORIGIN = 0x00000000 + 156K, LENGTH = 1024K - 156K
  RAM : ORIGIN = 0x20000000 + 63K, LENGTH = 256K - 63K
}
