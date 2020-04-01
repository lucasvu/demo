#include "pac5xxx.h"
#include "string.h"
#include "test_config.h"

#define TRIM_CONFIG_ADDR			(TrimParams *)0x20007000
#define TEST_CONFIG_ADDR			(TestConfig *)0x2000702C
#define INFO1_ADDR						(uint32_t *)0x00100000
#define INFO2_ADDR						(uint32_t *)0x00100400

TestConfig *pTestConfig;
TrimParams *pTrimConfig;

void delay(uint32_t time)
{
	while(time--);
}

PAC5XXX_RAMFUNC void pac55xx_spia_write_register(uint8_t address, uint8_t value)
{
	uint8_t dummy_data;
	PAC55XX_SSPA->SSCR.SWSS = 0;	// assert SS
	PAC55XX_SSPA->DAT.DATA = ((address << 1) | 1);
	while(! PAC55XX_SSPA->STAT.RNE);
	dummy_data = PAC55XX_SSPA->DAT.DATA;
	while(PAC55XX_SSPA->STAT.RNE);
	
	PAC55XX_SSPA->DAT.DATA = value;
	while(! PAC55XX_SSPA->STAT.RNE);
	dummy_data = PAC55XX_SSPA->DAT.DATA;
	while(PAC55XX_SSPA->STAT.RNE);
	PAC55XX_SSPA->SSCR.SWSS = 1;	// de-assert SS
}
uint16_t crc16(uint16_t sum, uint32_t *p_mem, uint32_t len)
{
    PAC55XX_CRC->SEED.CRCSEED = sum;

    // Compute CRC using 32-bit input on memory that is 32-bit aligned 
    while (len)    
    {
        PAC55XX_CRC->DATAIN = *p_mem++;             // Input a 32-bit word
        len = len - 4;                              // Decrement Length by 4 bytes
    }
    
    // Small delay to allow CRC to finish final output for reading
    __asm{NOP};
    __asm{NOP};

    // Return final CRC result
    return PAC55XX_CRC->OUT.CRCOUT;
}

void configure_crc_test(void)
{
	/* Configure clock system */
    PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_CLKREF;
    PAC55XX_SCC->CCSPLLCTL.w = 0x00012C45 & 0xFFFFFFFE;
    PAC55XX_SCC->CCSPLLCTL.PLLEN = 1;
    // Wait for Lock
    while(PAC55XX_SCC->CCSPLLCTL.PLLLOCK == 0);
    PAC55XX_SCC->CCSCTL.ACLKDIV = 0;
    PAC55XX_SCC->CCSCTL.PCLKDIV = 0;
    PAC55XX_SCC->CCSCTL.HCLKDIV = 0;
    PAC55XX_SCC->CCSCTL.SCLKMUXSEL = CCSCTL_SCLK_PLLCLK;
	
	/* Configure CRC module */
	PAC55XX_CRC->CTL.INREF = 0;
    PAC55XX_CRC->CTL.OUTREF = 0;
    PAC55XX_CRC->CTL.POLYSEL = CRC16_CCITT;
    PAC55XX_CRC->CTL.DATAWIDTH = CRC_DATA_WIDTH_32BITS;
	
	/* Configure MEMCTL */
	PAC55XX_MEMCTL->MEMCTL.w = 0x00520048;
}

PAC5XXX_RAMFUNC void write_info2(uint16_t info1crc, uint16_t info2crc)
{
	PAC55XX_MEMCTL->FLASHLOCK = 0x2B73D8E2;
	*(uint32_t *)0x00100460 = ((uint32_t)info1crc) | ((uint32_t)info1crc << 16);
	*(uint32_t *)0x00100464 = ((uint32_t)info2crc) | ((uint32_t)info2crc << 16);
	*(uint32_t *)0x00100468 = 0xFFFFFFFF;
	*(uint32_t *)0x0010046c = 0xFFFFFFFF;
	
	while(PAC55XX_MEMCTL->MEMSTATUS.WBUSY);
	while(1);
}

void configure_spi_bypass(void)
{
	/* Configure clock system */
    PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_CLKREF;
    PAC55XX_SCC->CCSPLLCTL.w = 0x00012C45 & 0xFFFFFFFE;
    PAC55XX_SCC->CCSPLLCTL.PLLEN = 1;
    // Wait for Lock
    while(PAC55XX_SCC->CCSPLLCTL.PLLLOCK == 0);
    PAC55XX_SCC->CCSCTL.ACLKDIV = 0;
    PAC55XX_SCC->CCSCTL.PCLKDIV = 0;
    PAC55XX_SCC->CCSCTL.HCLKDIV = 1;
    PAC55XX_SCC->CCSCTL.SCLKMUXSEL = CCSCTL_SCLK_PLLCLK;
	
	/* Configure GPIO */
	PAC55XX_SCC->PFMUXSEL.P3=0; //CLK -> PA3
	PAC55XX_SCC->PFMUXSEL.P4=0; //SS  -> PA6
	PAC55XX_SCC->PFMUXSEL.P5=0; //MISO-> PA5
	PAC55XX_SCC->PFMUXSEL.P6=0;	//MOSI-> PA4
	
	PAC55XX_GPIOF->MODE.P3=3;
	PAC55XX_GPIOF->MODE.P4=3;
	PAC55XX_GPIOF->MODE.P5=1;
	PAC55XX_GPIOF->MODE.P6=3;	

	PAC55XX_SCC->PAMUXSEL.P3=0; //CLK
	PAC55XX_SCC->PAMUXSEL.P4=0; //MOSI
	PAC55XX_SCC->PAMUXSEL.P5=0; //MISO
	PAC55XX_SCC->PAMUXSEL.P6=0;	//SS
	
	PAC55XX_GPIOA->MODE.P3=1;
	PAC55XX_GPIOA->MODE.P4=1;
	PAC55XX_GPIOA->MODE.P5=3;
	PAC55XX_GPIOA->MODE.P6=1;		
}

PAC5XXX_RAMFUNC void spi_update(void)
{
	while(1)
	{
		PAC55XX_GPIOA->OUT.P6=PAC55XX_GPIOF->IN.P4;//SS
		PAC55XX_GPIOF->OUT.P5=PAC55XX_GPIOA->IN.P5;//MISO
		PAC55XX_GPIOA->OUT.P4=PAC55XX_GPIOF->IN.P6;//MOSI
		PAC55XX_GPIOA->OUT.P3=PAC55XX_GPIOF->IN.P3;//CLK
	}
}

PAC5XXX_RAMFUNC void enter_sleep()
{
    // Prepare for Sleep
    // Set HCLK = SCLK = FRCLK = CLKREF = 4 MHz and MCLK = ROSCCLK = 16 MHz
    PAC55XX_MEMCTL->MEMCTL.MCLKSEL = MEMCTL_MCLK_ROSCCLK;           // Flash off ROSCCLK
    PAC55XX_SCC->CCSCTL.SCLKMUXSEL = CCSCTL_SCLK_FRCLK;             // SCLK = FRCLK 
    PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_CLKREF;          // FRCLK = CLKREF
    PAC55XX_SCC->CCSCTL.HCLKDIV = CCSCTL_HCLKDIV_DIV1;              // HCLK = SCLK / 1
    PAC55XX_SCC->CCSCTL.XTALEN = 0;
    PAC55XX_ADC->ADCCTL.ENABLE = 0;                                 // Must disable ADC before clock gating ADCCLK
    PAC55XX_SCC->CCSCTL.ADCCLKEN = 0;
    PAC55XX_SCC->CCSCTL.ACLKEN = 0;
    PAC55XX_SCC->CCSPLLCTL.PLLEN = 0;
    PAC55XX_WWDT->WWDTCTL.CLKSEL = WWDTCTL_CLKSEL_FRCLK;            // Use slower FRCLK for WWDT - might get reduced power by dividing this down WWDT clk
    PAC55XX_SCC->CCSCTL.STCLKSLPEN = 1;                             // Set Systick Clock to stop running in Deep Sleep
    
    // Disable PCLK after all other peripherals are disabled
    PAC55XX_SCC->CCSCTL.PCLKEN = 0;

    // Put Flash in standby and turn of LDO
    PAC55XX_MEMCTL->FLASHLOCK = FLASH_LOCK_ALLOW_WRITE_MEMCTL;
    PAC55XX_MEMCTL->MEMCTL.STDBY = 1;
    PAC55XX_SCC->CCSCTL.LDOEN = 0;
    
    // With Flash not running, can turn of ROSC
    PAC55XX_SCC->CCSCTL.ROSCEN = 0;
	PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_EXTCLK;

    // Put ARM in Sleep mode
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
}

void configure_normal_clock(void)
{
	/* Configure clock system */
    PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_CLKREF;
    PAC55XX_SCC->CCSPLLCTL.w = 0x00012C41 & 0xFFFFFFFE;
    PAC55XX_SCC->CCSPLLCTL.PLLEN = 1;
    // Wait for Lock
    while(PAC55XX_SCC->CCSPLLCTL.PLLLOCK == 0);
    PAC55XX_SCC->CCSCTL.ACLKDIV = 0;
    PAC55XX_SCC->CCSCTL.PCLKDIV = 0;
    PAC55XX_SCC->CCSCTL.HCLKDIV = 1;
    PAC55XX_SCC->CCSCTL.SCLKMUXSEL = CCSCTL_SCLK_PLLCLK;
	PAC55XX_SCC->CCSCTL.ROSCEN = 0;
}

int main(void)
{
	PAC55XX_MEMCTL->FLASHLOCK = FLASH_LOCK_ALLOW_WRITE_MEMCTL; // D513B490
	PAC55XX_MEMCTL->MEMCTL.CACHEDIS = 0;
	PAC55XX_SCC->CCSCTL.FRCLKMUXSEL = CCSCTL_CLKIN_ROSC;      // FRCLK = ROSC
	
	memset(TRIM_CONFIG_ADDR, 0, 11*sizeof(TrimParams)+sizeof(TestConfig)); 
	pTrimConfig = TRIM_CONFIG_ADDR;
	pTestConfig = TEST_CONFIG_ADDR;
	pTestConfig->unlock = 0xDEAD;
		
	while(pTestConfig->unlock != 0xCAFE)
	{
			delay(500);
	}
	
	if(pTestConfig->nOftrim > 0)
	{
			// Configure SPI
			PAC55XX_SCC->PAMUXSEL.w = 0x01111110;
			PAC55XX_GPIOA->MODE.w = 0x0000DD7F;
			PAC55XX_SCC->CCSCTL.USAMODE = 0;
			PAC55XX_SSPA->CLK.w = 0x00000408;
			PAC55XX_SSPA->SSCR.w = 0x0000000C;
			PAC55XX_SSPA->CON.w = 0x00000807;
		
			uint8_t i;
			for(i = 0; i < pTestConfig->nOftrim; i++)
			{
					pac55xx_spia_write_register((pTrimConfig+i)->address, (pTrimConfig+i)->value);
			}
			PAC55XX_SSPA->CON.SSPEN = 0;
			PAC55XX_GPIOA->MODE.w = 0x0000FFFF;
	}

	switch(pTestConfig->mode)
	{
		case MODE_SPI_BYPASS:
			configure_spi_bypass();
			spi_update();
		case MODE_NORMAL:
			configure_normal_clock();
			while(1);
		case MODE_SLEEP:
			enter_sleep();
			while(1);
		case MODE_CRC:
			configure_crc_test();
			uint16_t info1crc, info2crc;
			info1crc = crc16(0,INFO1_ADDR,1024);
			info2crc = crc16(0,INFO2_ADDR,0x60);
			write_info2(info1crc, info2crc);
		default:
			break;
	}		
}

void exit(int value)
{
    while(1);
}
