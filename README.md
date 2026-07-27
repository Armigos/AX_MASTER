# AX_MASTER

STM32F411RE master board firmware for reading four AX-12A positions and
transmitting them to a slave board through an HC-05 Bluetooth UART link.

## Interfaces

- USART1, PA9/D8: AX-12A half-duplex bus at 1 Mbps
- USART2, PA2/PA3: TelePlot console at 115200 baud
- USART6, PC6/PC7: HC-05 link at 115200 baud

## Master motors

| Logical motor | AX-12 ID |
| --- | ---: |
| Master 1 | 10 |
| Master 2 | 11 |
| Master 3 | 12 |
| Master 4 | 14 |

Master motor torque is disabled during initialization so the motors can be
moved by hand. Present positions are sampled and sent over the HC-05 link at
100 Hz.

## Bluetooth frame

Each 18-byte frame has the following layout:

```text
A5 5A 01 SEQ 04
ID10 POS_L POS_H
ID11 POS_L POS_H
ID12 POS_L POS_H
ID14 POS_L POS_H
XOR_CHECKSUM
```

The checksum is the XOR of all bytes starting at the message type and ending
at the final position byte.

## TelePlot

USART2 publishes four independent position series at 10 Hz:

```text
>master_1_pos:512
>master_2_pos:512
>master_3_pos:512
>master_4_pos:512
```

## Console command

`bus1m` probes IDs 10, 11, 12, and 14 at 115200 baud, changes responding
motors to 1 Mbps, and verifies all four IDs at 1 Mbps. Do not remove motor
power while this command is updating EEPROM.
