/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "aq_serial.h"
#include "util.h"
#include <stdio.h>
#include <stdlib.h>

#ifdef SERIAL_UART1
serialPort_t *serialPort1;
#endif
#ifdef SERIAL_UART2
serialPort_t *serialPort2;
#endif
#ifdef SERIAL_UART3
serialPort_t *serialPort3;
#endif
#ifdef SERIAL_UART4
serialPort_t *serialPort4;
#endif
#ifdef SERIAL_UART5
serialPort_t *serialPort5;
#endif
#ifdef SERIAL_UART6
serialPort_t *serialPort6;
#endif

serialPort_t *serialSTDIO;


void serialOpenUART(serialPort_t *s) {
	struct serial_configure cfg;
	
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(s->serial_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("can not find device: %s\n", s->serial_name);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX ) == RT_EOK)
    {
		cfg.baud_rate = s->baudRate;
//		cfg.stop_bits = s->stopBits;
//		cfg.parity = s->parity;
		
        rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    }	
}

#ifdef SERIAL_UART1
serialPort_t *serialUSART1(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {

    serialPort_t *s;
	
    s = serialPort1 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));
	s->serial_name = "uart1";
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);
    return s;
}
#endif

#ifdef SERIAL_UART2
serialPort_t *serialUSART2(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    serialPort_t *s;
	
    s = serialPort2 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));
	s->serial_name = "uart2";
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);
    return s;
}
#endif

#ifdef SERIAL_UART3
serialPort_t *serialUSART3(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    serialPort_t *s;
	
    s = serialPort3 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));
	s->serial_name = "uart3";
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);
    return s;
}
#endif

#ifdef SERIAL_UART4
serialPort_t *serialUSART4(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    serialPort_t *s;
	
    s = serialPort4 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));
	s->serial_name = "uart4";
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);
    return s;
}
#endif

#ifdef SERIAL_UART5
serialPort_t *serialUSART5(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    serialPort_t *s;
	
    s = serialPort5 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));
	s->serial_name = "uart5";
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);
    return s;
}
#endif

#ifdef SERIAL_UART6
serialPort_t *serialUSART6(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    serialPort_t *s;
	
    s = serialPort6 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));
	s->serial_name = "uart6";
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);
    return s;
}
#endif

serialPort_t *serialOpen(char *serial_name, unsigned int baud, uint16_t flowControl, unsigned int rxBufSize, unsigned int txBufSize)
{
    serialPort_t *s = 0;

    // Enable USART clocks/ports
#ifdef SERIAL_UART1
    if (serial_name == "uart1") {
	s = serialUSART1(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART2
    if (serial_name == "uart2") {
	s = serialUSART2(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART3
    if (serial_name == "uart3") {
	s = serialUSART3(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART4
    if (serial_name == "uart4") {
	s = serialUSART4(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART5
    if (serial_name == "uart5") {
	s = serialUSART5(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART6
    if (serial_name == "uart6") {
	s = serialUSART6(flowControl, rxBufSize, txBufSize);
    }
#endif

    rt_sem_init(&(s->waitFlag), "ssem", 0, RT_IPC_FLAG_FIFO);
	
  
    s->rxHead = s->rxTail = 0;
    s->txHead = s->txTail = 0;
    s->baudRate = baud;
    s->flowControl = flowControl;
    //s->parity = USART_Parity_No;
    //s->stopBits = USART_StopBits_1;

    serialOpenUART(s);


    // use this port for STDIO if not already defined
    if (serialSTDIO == 0)
	serialSTDIO = s;

    return s;
}

void serialWrite(serialPort_t *s, unsigned char ch) {
    s->txBuf[s->txHead] = ch;
    s->txHead = (s->txHead + 1) % s->txBufSize;

//    if (s->txDMAStream)
//	serialStartTxDMA(s);
//    else
//	USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
}

unsigned char serialAvailable(serialPort_t *s) {
//    if (s->rxDMAStream)
//	return (s->rxDMAStream->NDTR != s->rxPos);
//    else
//	return (s->rxTail != s->rxHead);
}

int serialRead(serialPort_t *s) {
    int ch;

//    if (s->rxDMAStream) {
//	ch = s->rxBuf[s->rxBufSize - s->rxPos];
//	if (--s->rxPos == 0)
//	    s->rxPos = s->rxBufSize;
//    }
//    else {
//	ch = s->rxBuf[s->rxTail];
//	s->rxTail = (s->rxTail + 1) % s->rxBufSize;
//    }

    return ch;
}

int serialReadBlock(serialPort_t *s) {
    while (!serialAvailable(s))
	yield(1);

    return serialRead(s);
}

void serialPrint(serialPort_t *s, const char *str) {
    while (*str)
	serialWrite(s, *(str++));
}

void serialChangeBaud(serialPort_t *s, unsigned int baud) {
	struct serial_configure cfg;
	
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(s->serial_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("can not find device: %s\n", s->serial_name);
        return;
    }
	
    s->baudRate = baud;
	cfg.baud_rate = s->baudRate;
//	cfg.stop_bits = s->stopBits;
//	cfg.parity = s->parity;
	
	rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &cfg);

    serialOpenUART(s);
}

void serialChangeParity(serialPort_t *s, uint16_t parity) {

	struct serial_configure cfg;
	
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(s->serial_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("can not find device: %s\n", s->serial_name);
        return;
    }
	
    s->parity = parity;
	//cfg.baud_rate = s->baudRate;
//	cfg.stop_bits = s->stopBits;
	cfg.parity = s->parity;
	
	rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &cfg);

    serialOpenUART(s);
}

void serialChangeStopBits(serialPort_t *s, uint16_t stopBits) {
	struct serial_configure cfg;
	
    rt_device_t dev = RT_NULL;

    dev = rt_device_find(s->serial_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("can not find device: %s\n", s->serial_name);
        return;
    }
	
    s->stopBits = stopBits;
	//cfg.baud_rate = s->baudRate;
	cfg.stop_bits = s->stopBits;
	//cfg.parity = s->parity;
	
	rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &cfg);

    serialOpenUART(s);
}

//void serialWatch(void) {
//#ifdef SERIAL_UART1_PORT
//    if (serialPort1 && serialAvailable(serialPort1))
//	CoSetFlag(serialPort1->waitFlag);
//#endif
//#ifdef SERIAL_UART2_PORT
//    if (serialPort2 && serialAvailable(serialPort2))
//	CoSetFlag(serialPort2->waitFlag);
//#endif
//#ifdef SERIAL_UART3_PORT
//    if (serialPort3 && serialAvailable(serialPort3))
//	CoSetFlag(serialPort3->waitFlag);
//#endif
//#ifdef SERIAL_UART4_PORT
//    if (serialPort4 && serialAvailable(serialPort4))
//	CoSetFlag(serialPort4->waitFlag);
//#endif
//#ifdef SERIAL_UART5_PORT
//    if (serialPort5 && serialAvailable(serialPort5))
//	CoSetFlag(serialPort5->waitFlag);
//#endif
//#ifdef SERIAL_UART6_PORT
//    if (serialPort6 && serialAvailable(serialPort6))
//	CoSetFlag(serialPort6->waitFlag);
//#endif
//}

void serialSetSTDIO(serialPort_t *s) {
    serialSTDIO = s;
}

int __putchar(int ch) {
    if (serialSTDIO)
	serialWrite(serialSTDIO, ch);

    return ch;
}

int __getchar(void) {
    int ch = 0;

    if (serialSTDIO)
	ch = serialRead(serialSTDIO);

    return ch;
}

//
// Interrupt handlers
//

// UART1 TX DMA
#ifdef SERIAL_UART1_PORT
#ifdef SERIAL_UART1_TX_DMA_ST
void SERIAL_UART1_TX_DMA_IT(void) {
    serialPort_t *s = serialPort1;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART1 global IRQ handler (might not be used)
void USART1_IRQHandler(void) {
    serialPort_t *s = serialPort1;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART1_PORT

// UART2 TX DMA
#ifdef SERIAL_UART2_PORT
#ifdef SERIAL_UART2_TX_DMA_ST
void SERIAL_UART2_TX_DMA_IT(void) {
    serialPort_t *s = serialPort2;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART2 global IRQ handler (might not be used)
void USART2_IRQHandler(void) {
    serialPort_t *s = serialPort2;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART2_PORT

// UART3 TX DMA
#ifdef SERIAL_UART3_PORT
#ifdef SERIAL_UART3_TX_DMA_ST
void SERIAL_UART3_TX_DMA_IT(void) {
    serialPort_t *s = serialPort3;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART3 global IRQ handler (might not be used)
void USART3_IRQHandler(void) {
    serialPort_t *s = serialPort3;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART3_PORT

// UART4 TX DMA
#ifdef SERIAL_UART4_PORT
#ifdef SERIAL_UART4_TX_DMA_ST
void SERIAL_UART4_TX_DMA_IT(void) {
    serialPort_t *s = serialPort4;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART4 global IRQ handler (might not be used)
void UART4_IRQHandler(void) {
    serialPort_t *s = serialPort4;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART4_PORT

// UART5 TX DMA
#ifdef SERIAL_UART5_PORT
#ifdef SERIAL_UART5_TX_DMA_ST
void SERIAL_UART5_TX_DMA_IT(void) {
    serialPort_t *s = serialPort5;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART5 global IRQ handler (might not be used)
void UART5_IRQHandler(void) {
    serialPort_t *s = serialPort5;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART5_PORT

// UART6 TX DMA
#ifdef SERIAL_UART6_PORT
#ifdef SERIAL_UART6_TX_DMA_ST
void SERIAL_UART6_TX_DMA_IT(void) {
    serialPort_t *s = serialPort6;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART6 global IRQ handler (might not be used)
void USART6_IRQHandler(void) {
    serialPort_t *s = serialPort6;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART6_PORT
