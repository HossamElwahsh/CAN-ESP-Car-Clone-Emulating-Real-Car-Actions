# RC Car Control

----
# Issues
## 1. UART ISR Not working
- ### Environment
  - FreeRTOS Kernel V10.0.1 
  - UART uses DMA
  - DMA Interrupt Priority: `0`
  - `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` = `5`
  - `configLIBRARY_LOWEST_INTERRUPT_PRIORITY` = `15`
  


- ISR fires but when calling any API `..FromISR()` the whole system halts
- When DMA set priority set: `6`
  - ISR doesn't fire at all, although it's in the range `5 - 15`, 
  - _with 5 being the highest priority_ 

   
- ### Solution:
- Set DMA ISR priority to exactly `5` so that the ISR will fire and you will be able to call `...FromISR()` APIs safely without getting stuck in the `port.c@677` 👇

  		configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
---
## 2. DC Motors doesn't run on PWM duty less than 70%
- At duty cycle less than 70% the average DC voltage is less than the required to kickstart and run the motors, therefore you'll have to map all the PWM duties to more than or equal 70% for the motors to run.
----
