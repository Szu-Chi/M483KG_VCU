#include "SYS_Init.h"

void SYS_Init(void){

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk; // XTAL12M (HXT) Enabled

    /* Waiting clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(192000000);
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2); // PCLK divider set 2
    printf("Core Clock Frequency: 192MHz\n");

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    
    /* Lock protected registers */
	SYS_LockReg();
	printf("System Initialization Completed\n");
}

void TIMER0_Init(uint8_t freq){
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;

	/* Select TMR clock source from high speed crystal */
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, freq);

    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer */
    TIMER_Start(TIMER0);
    printf("Timer0 Initialization Completed\n");
    printf("Timer0 Interruption Frequency: %d\n",freq);
}
void TIMER1_Init(uint8_t freq){
	/* Enable IP clock */
	CLK->APBCLK0 |= CLK_APBCLK0_TMR1CKEN_Msk;
	
	/* Select TMR clock source from high speed crystal */
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);

    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, freq);

    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);

    /* Start Timer */
    TIMER_Start(TIMER1);
    printf("Timer1 Initialization Completed\n");
    printf("Timer1 Interruption Frequency: %d\n",freq);
} 
