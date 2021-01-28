# STM32F103_DUAL_VCP

<font color="red">RadioOperator modified:  
</font>
<font color="red">- changed new USB lib from STM32CubeMX to improve PC Enum.  
</font>
<font color="red">- added LED (PC13) flashing when UART transfering data.  
</font>




An example to show how to use dual CDC VCP USB interfaces. 

## Hardware configuration

MCU: STM32F103C6Tx(72MHz, LQFP48, 32KB Flash, 10KB RAM)  
Hardware board: BluePill  
LED: PC13, low-active  

## Software Development Environment

- MDK/Keil v5.xx  
- ARMCC v6.xx -O1  

## Firmware Configureation

- Memory configuration:
    - Heap Size: 0x1000
    - Stack Size: 0x400

- Perpherials
    - RCC 
        - High Speed Clock (HSE): Crystal/Ceramic Resonator
        - Low Speed Clock (LSE) : Crystal/Ceramic Resonator
    - USB
        - Device (FS)

    - USART1
        - Mode: Asynchronous

    - USART2
        - Mode: Asynchronous

- MiddleWares
    - USB_DEVICE
        - Class for FS IP: Communication Device Class (Virtual Port Com)

- Pin configuration
    - USB_DM: PA11
    - USB_PM: PA12
    - USART1_TX: PA9
    - USART1_RX: PA10
    - USART2_TX: PA2
    - USART2_RX: PA3


```

