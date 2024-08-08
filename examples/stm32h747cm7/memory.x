MEMORY
{
    FLASH    : ORIGIN = 0x08040000, LENGTH = 768K  /* BANK_1 - 0x08000000 to 0x0803FFFF reserved for bootloader */
    RAM      : ORIGIN = 0x24000000, LENGTH = 512K  /* AXIRAM */
    RAM_D3   : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 */
}

SECTIONS
{
    .ram_d3 :
    {
        *(.ram_d3)
    } > RAM_D3
}
