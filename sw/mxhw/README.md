# mxhw -- Read enabled hardware from custom CSR

This program reads the synthesized hardware from a
custom CS register `mxhw`. The register is located at address
0xcf0. This register is read-only.

| Bit  | Description             |
|-----:|:------------------------|
| 0    | GPIOA enabled           |
| 1    | Reserved                |
| 2    | Reserved                |
| 3    | Reserved                |
| 4    | UART1 enabled           |
| 5    | Reserved                |
| 6    | I2C1 enabled            |
| 7    | Reserved                |
| 8    | SPI1 enabled            |
| 9    | SPI2 enabled            |
| 10   | TIMER1 enabled          |
| 11   | TIMER2 enabled          |
| 12   | Reserved                |
| 13   | Reserved                |
| 14   | Reserved                |
| 15   | Multiply/divide enabled |
| 16   | Fast divide enabled     |
| 17   | Bootloader enabled      |
| 18   | Registers in RAM        |
| rest | Reserved                |

## Status

Works on the DE0-CV board.
