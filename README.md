# stm32f10x_uart_dma
stm32f1系列的usart3 DMA方式接收和发送例程，发送采用write方式，附加，重写printf函数（程序配置勾选MicroLIB），在低速芯片如stm32上，慎用printf，容易出现丢帧现象。
