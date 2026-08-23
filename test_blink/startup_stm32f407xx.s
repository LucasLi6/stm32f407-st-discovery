; Minimal startup for bare-metal blink test @ 0x0800C000
; Same layout as APP — bootloader reads SP/PC from vector table here.

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

                PRESERVE8
                THUMB

; Vector table — placed at 0x0800C000 by scatter file (+First)
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End

__Vectors
                DCD     __initial_sp            ; SP
                DCD     Reset_Handler           ; PC ← bootloader jumps here
                DCD     Default_Handler         ; NMI
                DCD     Default_Handler         ; HardFault
                DCD     Default_Handler         ; MemManage
                DCD     Default_Handler         ; BusFault
                DCD     Default_Handler         ; UsageFault
                DCD     0
                DCD     0
                DCD     0
                DCD     0
                DCD     Default_Handler         ; SVCall
                DCD     Default_Handler         ; DebugMon
                DCD     0
                DCD     Default_Handler         ; PendSV
                DCD     Default_Handler         ; SysTick
                ; IRQ 0..81
                SPACE   (82 * 4)
__Vectors_End

                AREA    |.text|, CODE, READONLY

Reset_Handler   PROC
                EXPORT  Reset_Handler
                IMPORT  main

                ; Minimal SystemInit — enable FPU, set VTOR
                LDR     R0, =0x0800C000
                LDR     R1, =0xE000ED08          ; SCB->VTOR
                STR     R0, [R1]

                ; Enable FPU (CP10, CP11 full access)
                LDR     R1, =0xE000ED88          ; SCB->CPACR
                LDR     R0, [R1]
                ORR     R0, R0, #(0xF << 20)
                STR     R0, [R1]
                DSB
                ISB

                ; Call main() — no __main needed for bare-metal
                BL      main
                ENDP

Default_Handler PROC
                EXPORT  Default_Handler
                B       .
                ENDP

                ALIGN   4
                END
