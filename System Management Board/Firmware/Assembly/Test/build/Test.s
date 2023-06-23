.thumb_func
.global main
.equ PIN_Write, 0xd000001c
.equ GPIO_BASE, 0x40014000
.equ SIO_MODE, 5
.equ GPIO_25, 0x02000000
.equ GPIO_25_CTL, 0x0cc
.equ POS_100_THSND, 100000

main:   LDR R0, =GPIO_BASE
        LDR R2, =GPIO_25_CTL
        ADD R0, R0, R2
        LDR R1, =SIO_MODE
        STR R1, [R0, #0]
        MOV R2, #1
        LDR R3, =PIN_Write
        STR R2, [R3, #0]
        LDR R1, =GPIO_25
load:   LDR R0, =POS_100_THSND
loop:   ADD R0, R0, #-1
        Bge loop
        LDR R2, =GPIO_25
        STR R2, [R3, #0]
        ADD R1, R1, #1
        Beq load
        B main
