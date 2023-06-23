.thumb_func
.global state_enforce

.align 4
.equ RETURN_ADDR 0
.equ MAX_PIN, -29

@R0 has pointer to the pins to activate
@the two addresses following that pointer
@have the output pins and then a blank return address
state_enforce:  LDR R1, [R0], #1
                LDR R2, [R0], #1
                STR R7, [R0]
                STR R0, =RETURN_ADDR
                MOV R6, R0
                MOV R3, #1
                MOV R4, #0
loop1:          AND R5, R3, R1
                
                
                LDR R7, [R0]
                B R7
