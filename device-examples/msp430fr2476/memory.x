/* DEVICE SELECTION:
   To use MSP430FR2475: 
   - Change RAM LENGTH to 0x1800
   - Change ROM LENGTH to 0x8000
   (SLASEO7C Table 9-31, p. 73: the MSP430FR2475 has 6KB of RAM, 2000h to 37FFh, and 32KB of
   FRAM, 8000h to FFFFh)
*/

/* INTERRUPT VECTORS IN RAM:
   For examples/ram_vectors.rs:
   - Change RAM LENGTH to 0x1F80 (0x1780 on the MSP430FR2475)
   The interrupt vector table in RAM takes the top 128 bytes of RAM, 3F80h to 3FFFh on the
   MSP430FR2476 (SLAU445I 1.3.6.1, p. 36), and the stack starts at the end of RAM, so RAM must leave
   them out. The other examples work either way.
*/

MEMORY
{
  /* Current Values for MSP430FR2476 (SLASEO7C Table 9-31, p. 73): 8KB of RAM, 2000h to 3FFFh, and
     64KB of FRAM, 8000h to 17FFFh, which holds the interrupt vectors and signatures at FF80h to
     FFFFh (also SLASEO7C 9.4, p. 46: "The interrupt vectors and the power-up start address are in
     the address range 0FFFFh to 0FF80h") */
  RAM     : ORIGIN = 0x2000, LENGTH = 0x2000
  ROM     : ORIGIN = 0x8000, LENGTH = 0x10000  
  VECTORS : ORIGIN = 0xFF80, LENGTH = 0x80
}