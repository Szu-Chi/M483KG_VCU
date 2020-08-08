#include "UART0.h"

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead = 0;
volatile uint32_t g_u32comRtail = 0;
volatile char mode;
volatile bool ModeWaitFlag = true;
volatile uint8_t g_u8RecData[RXBUFSIZE] = {0};

void UART0_IRQHandler(void){
	char u8InChar;
    uint8_t Check;
    uint32_t u32IntSts = UART0->INTSTS;
    if(u32IntSts & UART_INTSTS_RDAINT_Msk){
        /* Get all the input characters */
        while(UART_GET_RX_EMPTY(UART0) == 0){
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);
            Check = (int)u8InChar;
            if (ModeWaitFlag){
                mode = u8InChar;
                ModeWaitFlag = false;
            }
        }
    }
    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk)){
    	printf("Problem\n");
    	printf("rxovif:%d bif:%d\n",(UART0->FIFOSTS & UART_FIFOSTS_RXOVIF_Msk),(UART0->FIFOSTS & UART_FIFOSTS_BIF_Msk));
        UART0->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    }
}

void UART_Init(void){
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    // Set RX FIFO Interrupt Trigger Level
    UART0->FIFO &= ~ UART_FIFO_RFITL_Msk;
    UART0->FIFO |= UART_FIFO_RFITL_1BYTE;

    /* Enable UART RDA/THRE/Time-out interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
}

