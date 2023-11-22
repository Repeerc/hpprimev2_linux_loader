
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include "regs.h"

#include "atag.h"

#include "lcd.h"


#define Fin_MHZ  12

/*


Start
B
Run>
Init 320x240
Init 320x240 rVIDCON0=0x5270
320x240 rVIDTCON0=0x110300 rVIDTCON1=0x401100 rVIDTCON2=0x7793f rVIDCON1=0x8080
ARMCLK:400000000
HCLK  : 133333333
PCLK  : 66666666
nandid: ad da 90 95 44 
InitBfsHeader...
nandid: ad da 90 95 44 
block size:0x20000 page size :0x800 Attr:1c03110b NandSize:256(MB)
read header...ok
has BFS header
[00][01]
BFS End.

1ram size :32MB
rBANKCFG:4890d
GetHWVersion:3 31
CodeEntry:0x30000020
CodeLoadeAddress:0x30000000
CodeLoadSize:0x100000


*/

uint8_t params_dat[1024];

struct tag *params = (struct tag *)params_dat;
//linuxrc rootfstype=jffs2 rw rootwait   
//ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs rw  rootwait 
//ubi.mtd=3,2048 root=ubi0:rootfs rootfstype=ubifs rw 
// root=/dev/mtdblock3 rootfstype=jffs2 rw rootwait 
//init=/linuxrc  ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs rw  rootwait
//init=/sbin/init root=/dev/sda1 rootfstype=ext4 rw rootwait
//ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs rw 
//console=ttyS0 console=ttySAC0,115200 

char cmdline[] = "noinitrd init=/linuxrc root=/dev/mtdblock3 rootfstype=jffs2 rw rootwait rootwait  earlyprintk ignore_loglevel";

//------------------------------------------------------------------------------


volatile UART_REG *uart0 = (volatile UART_REG *)UART0_REG_PA_BASE;
volatile UART_REG *uart1 = (volatile UART_REG *)UART1_REG_PA_BASE;
volatile GPIO_REG *gpio = (volatile GPIO_REG *)GPIO_REG_PA_BASE;

volatile void _init() __attribute__((naked)) __attribute__((section(".init"))) __attribute__((naked));
volatile void _init()
{
	__asm volatile(".word  0x30000020");  //entry
	__asm volatile(".word  0");
	__asm volatile(".word  0x100000");    //load size
	__asm volatile(".word  0x30000000");  //load addr
	__asm volatile(".word  0x100000");   //size
	__asm volatile(".word  0x4A3556");  // "V5J"
	__asm volatile(".word  0x36313432"); //"2416"
	__asm volatile(".word  0"); 


	__asm volatile("ldr pc,.Lreset");
	__asm volatile(".Lreset: 		.long 	 _boot");

}


extern unsigned int __bss_start;
extern unsigned int __bss_end; 
extern unsigned int __HEAP_START;
static void *heap = NULL;

#define HEAP_END (0x30000000 + 512 * 1024)

void putch(char c)
{
    while(uart0->rUFSTAT>>14);
    uart0->rUTXH = c;
}

caddr_t _sbrk(int incr) {
    void *prev_heap;

    if (heap == NULL) {
        heap = &__HEAP_START;
    }
    prev_heap = heap;
    if (((uint32_t)heap + incr) > HEAP_END) {
        //printf("heap:%p, incr:%d\r\n", heap, incr);
        putch('M');
        putch('E');
        putch('M');
        putch('O');
        putch('F');
        for(;;);
//        return 0;
    }
    heap += incr;

    return (caddr_t)prev_heap;
}

char prbuf[128];
int uart0_printf(char *format, ...)
{
    
    va_list ap;
    int n, i;
    va_start(ap, format);
    n = vsprintf(prbuf, format, ap);
    i = 0;
    while(prbuf[i])putch(prbuf[i++]);
    va_end(ap);
    return n;
}


uint32_t powu32(int x, int y)
{
    while(--y)x = x*x;
    return x;
}

uint32_t CPU_GetFclkHZ(void)
{
	int 	fclk,mdiv,pdiv,sdiv;
	volatile SYSCON_REG *clk= (SYSCON_REG*)SYSCON_REG_PA_BASE;
	
	sdiv=clk->rMPLLCON;
	pdiv=(clk->rMPLLCON>>5);
	mdiv=(clk->rMPLLCON>>14);
	
	sdiv&=0x03;
	pdiv&=0x3f;
	mdiv&=0x3ff;
	
	fclk	=(mdiv*Fin_MHZ)/(pdiv*powu32(2,sdiv));
	return	(fclk*1000000);

}

uint32_t	CPU_GetHclkHZ(void)
{

	uint32_t	pre_div,hclk_div;
	uint32_t hclk_ratio;
	volatile	SYSCON_REG	*sys=(SYSCON_REG*)SYSCON_REG_PA_BASE;
	////

	pre_div		=(sys->rCLKDIV0>>4)&0x03;
	hclk_div	=(sys->rCLKDIV0)&0x03;
	hclk_ratio	=(pre_div+1)*(hclk_div+1);
	////
	
//	Uart_Printf(0,"HclkRatio =%d\r\n",hclk_ratio);
	
	return	CPU_GetFclkHZ()/hclk_ratio;
}

uint32_t	CPU_GetPclkHZ(void)
{ 
	volatile SYSCON_REG *sys=(SYSCON_REG*)SYSCON_REG_PA_BASE;
	//return	66000000; 
	if(sys->rCLKDIV0&(1<<2))
	{
		return CPU_GetHclkHZ()>>1;
	}
	else
	{
		return CPU_GetHclkHZ();
	}
}

#define NF8_ReadPage_Adv(block,page,buf) (((int(*)(uint32_t, uint32_t, uint8_t*))(0xAF0 ))(block,page,buf))

#define CopyMovitoMem(a,b,c,d) (((bool(*)(uint32_t,uint16_t,uint32_t*, bool))(0x1B68))(a,b,c,d))
#define globalBlockSizeHide *((volatile unsigned int*)0x40003FFC)
#define globalSDHCInfoBit *((volatile unsigned int*)0x40003FF8)
 
void clock_init()
{
    volatile SYSCON_REG *sys=(SYSCON_REG*)SYSCON_REG_PA_BASE;

/*
    Clock setting(External Crystal12M):
    MPLLCLK = 800M, EPLLCLK = 96M
    ARMCLK = 400M, HCLK = 133M
    DDRCLK = 266M, SSMCCLK = 66M,PCLK = 66M
    HSMMC1 = 24M
*/
    LOCKCON0_OFS = 3600;
    LOCKCON1_OFS = 3600;

    //ARMCLK Ratio=(ARMDIV+1),
    //HCLKRatio=(PREDIV+1)*(HCLKDIV+1)
    CLKDIV0_OFS =   (1<<0)|   // HCLKDIV = 1 
                    (1<<2)|   // PCLK = HCLK/2
                    (1<<3)|   // HCLKx1_2(SSMC) =  HCLK/2
                    (2<<4)|   // PREDIV HCLK = 2
                    (1<<9);   // ARMDIV = 1 

    //EPLL
    CLKDIV1_OFS =   (0x1<<4)| // USBHOSTDIV = 0+1
                    (0x3<<6)| // HSMMCDIV_1 = 3+1
                    (1<<0)| // UART clock divider ratio, ratio = (UARTDIV + 1) 4bit
                    (0x0<<12)| // I2S0 clock divider ratio, ratio = (I2SDIV_0 + 1)
                    (0x0<<16)| // Display controller clock divider ratio,ratio = (DISPDIV + 1)
                    (0x0<<24); // HS-SPI clock divider ratio, ratio = (SPIDIV +1)
    
    CLKDIV2_OFS =   (0x0<<0)| // HS-SPI0 clock divider ratio(MPLL), ratio = (SPIDIV_1 +1)
                    (0x3<<6); // HSMMC_0 clock divider ratio(EPLL), ratio = (HSMMCDIV_1 + 1)

    sys->rEPLLCON_K = 0;

    //MDIV=32,PDIV=1,SDIV=2,Fout=((MDIV+(KDIV/2^16))*Fin)/(PDIV*2^SDIV),KDIV=0
    EPLLCON_OFS =   (2<<0)|   //S
                    (1<<8)|   //P
                    (32<<16)| //M
                    (0x0<<24)|(0x0<<25);  // 96MHz


    //MDIV=400,PDIV=3,SDIV=1,Fout=(MDIV*Fin)/(PDIV*2^SDIV)
    MPLLCON_OFS =   (1<<0)|    //S
                    (3<<5)|    //P
                    (400<<14)| //M
                    (0x0<<24)|(0x0<<25); // 800MHz

    HCLKCON_OFS =   (1 << 20) | //2D
                    (1 << 19) | //DRAM
                    (1 << 18) | //SSMC
                    (0 << 16) | //HSMMC1
                    (0 << 15) | //HSMMC0
                    (1 << 13) | //IROM
                    (1 << 12) | //USBDEV
                    (1 << 11) | //USBHOST
                    (1 << 9) | //DISPCON   
                    (0x3 << 6) | //RESERVED
                    (0x3F << 0) ;//DMA0~5   

    

    CLKSRC_OFS = (1<<4)|(1<<6);

     


}
 
void PrepareATAGS()
{

    params->hdr.tag = ATAG_CORE;
	params->hdr.size = tag_size (tag_core);
 
	params->u.core.flags = 0;
	params->u.core.pagesize = 0;
	params->u.core.rootdev = 0;
	params = tag_next (params);

	params->hdr.tag = ATAG_MEM; 
	params->hdr.size = tag_size (tag_mem32);
	params->u.mem.start = 0x30000000; //memory base
	params->u.mem.size = 32*1024*1024 - 512*1024; //512K Reserved for fb
	params = tag_next (params);

	//params->hdr.tag = ATAG_MEM;  /*0x54410002*/
	//params->hdr.size = tag_size (tag_mem32);
	//params->u.mem.start = 0x38000000;
	//params->u.mem.size = 64*1024*1024;
	//params = tag_next (params);

    
	char *p;
	/* eat leading white space */
	for (p = cmdline; *p == ' '; p++);
	params->hdr.tag = ATAG_CMDLINE;  /*0x54410009*/
	params->hdr.size =
		(sizeof(struct tag_header) + strlen(p));
	strcpy(params->u.cmdline.cmdline, p);
    uart0_printf("cmdline len:%d\r\n",params->hdr.size);
	params = tag_next (params);

	params->hdr.tag = ATAG_NONE;
	params->hdr.size = 0;

    uart0_printf("atagsz:%d\r\n", (uint32_t)params - (uint32_t)params_dat);
}
 
#define	LCD_XSIZE	320
#define	LCD_YSIZE	240
#define	LCD_BPP		24 
 

#if	(LCD_BPP==16)
#define	LCD_COLOR	uint16_t
#endif

#if	(LCD_BPP==24)
#define	LCD_COLOR	uint32_t
#endif

#define	LCD_BUF_SIZE	(LCD_XSIZE*LCD_YSIZE*sizeof(LCD_COLOR))

#define LCD_BUF_PA_BASE  ((LCD_COLOR *)(0x30000000 + 32 * 1048576 - 512 * 1024))


LCD_REG *lcd = (LCD_REG *)LCD_REG_PA_BASE;
void lcd_init()
{
 
	if ((lcd->rVIDCON0 & 0x3) == 0x3) {
		while ((lcd->rVIDCON1 & 0x6000) == 0x4000);
	}
	lcd->rWINCON0 = (lcd->rWINCON0 & 0xFF7FFFFF) | (0 << 23);


	lcd->rVIDW00ADD0B0 = (uint32_t)LCD_BUF_PA_BASE;		
	lcd->rVIDW00ADD1B0 = (uint32_t)LCD_BUF_PA_BASE + LCD_BUF_SIZE;


    #if(LCD_BPP==16)
	lcd->rWINCON0 = (lcd->rWINCON0 & 0xFFFFFFC3) | 0x14;
    #endif
    #if(LCD_BPP==24)
	lcd->rWINCON0 = (lcd->rWINCON0 & 0xFFFFFFC3) | 0x2C;
    #endif

    lcd->rVIDCON0 = (lcd->rVIDCON0 & 0xFFFFFFFC) | 0x3;

    lcd->rDITHMODE = 0;

    for(int y = 0; y < 240; y++)
    for(int x = 0; x < 320; x++)
    {
        {
            //m = x/100;
            int m = x/20;
            #if(LCD_BPP==16) 
                if(m < 16)
                {
                    LCD_BUF_PA_BASE[x+y*320] = 1 << m;
                } 
            #endif
            #if(LCD_BPP==24)
                LCD_BUF_PA_BASE[x+y*320] = (((m&1)*0xFF)<<0) | (((m&2)*0xFF)<<8) | (((m&4)*0xFF)<<16);
            #endif
            //LCD_BUF_PA_BASE[x+y*800] = x % 0xff;
        }
    }
    uart0_printf( "rVIDCON0 = %08X\r\n",  lcd->rVIDCON0);
    uart0_printf( "rVIDCON1 = %08X\r\n",  lcd->rVIDCON1);

    uart0_printf( "rVIDTCON0 = %08X\r\n",  lcd->rVIDTCON0);
    uart0_printf( "rVIDTCON1 = %08X\r\n",  lcd->rVIDTCON1);
    uart0_printf( "rVIDTCON2 = %08X\r\n",  lcd->rVIDTCON2); 
}

void __attribute__((target("arm"))) mmu_invalidate_dcache_all()
{
    register uint32_t value asm("r0");

    value = 0;

    __asm volatile (" mcr p15, 0, %0, c7, c6, 0 " :: "r"(value) );
}


void delay()
{ 

    mmu_invalidate_dcache_all();
    uint32_t volatile *BCDSEC = (uint32_t *)0x57000070;
    uint32_t trd = *BCDSEC;
    while (trd == *BCDSEC)
    {
        mmu_invalidate_dcache_all();
    }
}
 
#define GPIO_COL  gpio->rGPKDAT
#define GPIO_ROW  gpio->rGPGDAT

uint32_t scan_keyboard()
{
    //GPIO_COL = ~1;

    for(int col=0;col<16;col++)
    {
        GPIO_COL = ~(1<<col);
        delay();

        for(int row = 0; row <8;row++)
        {
            if(!((GPIO_ROW >> row) & 1))
            {
                uart0_printf("K:row:%d  col:%d\r\n",   row, col);
            }
        }
    }
    return 0;
}




#include "nand.h"


int Nand_ReadID(Nand_ID_Info *pInfo)
{
	if (pInfo == (Nand_ID_Info *)0) {
		return 1; // 参数错误
	}
	NF_CE_ENABLE();
	NF_CLEAR_RB();
	NF_CMD(NAND_CMD_READID); // 发送读ID命令
	NF_ADDR(0x0); // 写0x0地址
	
	pInfo->Maker = NF_READ_BYTE(); // Maker:0xEC
	pInfo->Device = NF_READ_BYTE(); // Device:0xDA
	pInfo->ID_Data3 = NF_READ_BYTE(); //0x10
	pInfo->ID_Data4 = NF_READ_BYTE(); //0x95
	pInfo->ID_Data5 = NF_READ_BYTE();  //0x44	
	
	NF_CE_DISABLE();
	return 0;
}

static int Nand_ReadPage(uint32_t Page,
							uint8_t *Buffer)
{
	unsigned int i;
	unsigned int MECC, SECC;
	if (Buffer == (unsigned char *)0) {
		return 1; // 缓冲区为空，参数错误
	}
	//Page &= (64-1); // 64 page in one block
	//Page += (Block << 6); // Block转换为页数
	NF_INIT_MECC(); // main区ECC清空
	NF_INIT_SECC(); // spare区ECC清空
	NF_MECC_UNLOCK(); // main区ECC解锁，开始ECC计算
	NF_CE_ENABLE(); // 使能片选
	NF_CLEAR_RB(); // 清数据传输标志
	
	NF_CMD(NAND_CMD_READ0); // page read cycle 1
	NF_ADDR(0); // column address
	NF_ADDR(0); // columu address
	NF_ADDR(Page & 0xff); // 写入3字节的页地址
	NF_ADDR((Page>>8) & 0xff);
	NF_ADDR((Page>>16) & 0xff);	
	NF_CMD(NAND_CMD_READSTART); // page read cycle 2

	NF_WAIT_READY(); // 等待命令完成	
	for (i=0; i<2048; i++) { // 读取main区数据
		Buffer[i] = NF_READ_BYTE();
	}
	NF_MECC_LOCK(); // 锁定main ECC
	NF_SECC_UNLOCK(); // 解锁spare ECC

	MECC = NF_READ_WORD(); // spare区前4字节为main区ECC
	// main区的ECC放入到NFMECCD0/1中相应的位中
	rNFMECCD0=((MECC&0xff00)<<8) | (MECC&0xff);	
	rNFMECCD1=((MECC&0xff000000)>>8) | ((MECC&0xff0000)>>16);
	NF_SECC_LOCK(); // 锁定spare ECC
	// spare区第5,6这两字节为spare区ECC,剩下部分未使用
	SECC = NF_READ_WORD();
	// spare区的ECC放入到NFMECCD0/1中相应的位中	
	rNFSECCD=((SECC&0xff00)<<8)|(SECC&0xff);	
	NF_CE_DISABLE();
	
	// check whether spare/main area bit fail error occurred
	if ((rNFECCERR0 & 0xf) == 0) {
		return 0; // 数据读取正确
	} else {
		return 2; // ECC检验不一致，数据读取有误
	}
}

void set_flash_lock_boundary(uint32_t sb, uint32_t eb)
{

    *NFSBLK = ((sb<<6) & 0xff) | (((sb>>2) & 0xff) << 8)| (((sb >> 10) & 0xff) << 16);
    *NFEBLK = ((eb<<6) & 0xff) | (((eb>>2) & 0xff) << 8)| (((eb >> 10) & 0xff) << 16);

    *NFCONT |= 1<<17;  
    uart0_printf("Lock blks, NFCONT:%08x, row addr:%d~%d\r\n", *NFCONT, *NFSBLK, *NFEBLK);
    uart0_printf("Lock blks, NFCONT:%08x, BLK:%d~%d\r\n", *NFCONT, sb, eb);
}


uint8_t *pgbuf = (uint8_t *)(0x30000000 + 6 * 1048576);

volatile void _boot() __attribute__((naked));
volatile void _boot(){
	

    //WT_BASE = 0; //turn off watch dog
 

	for(char *i = (char *)&__bss_start; i < (char *)&__bss_end; i++){
		*i = 0;		//clear bss
	}

    __asm volatile("mrs r1, cpsr_all");
    __asm volatile("orr r1, r1, #0xc0");
    __asm volatile("msr cpsr_all, r1");	//Disable interrupt


    clock_init();

    gpio->rGPHCON &= ~((3<<(0*2))|(3<<(1*2))); //set gpio for uart0
    gpio->rGPHCON |= (2<<(1*2))|(2<<(0*2));

    uart0->rULCON = 0x3;
    uart0->rUCON = 0x5 ;
    uart0->rUFCON = 0x7;
    uart0->rUMCON = 0;
    uart0->rUBRDIV = (CPU_GetPclkHZ()/(16*115200))-1;
    uart0->rUDIVSLOT = 0x0888;


    uart0_printf("NFSBLK:%d\r\n", *NFSBLK);
    uart0_printf("NFEBLK:%d\r\n", *NFEBLK);
    uart0_printf("NFCONT:%08x\r\n", *NFCONT);
    uart0_printf("NFCONF:%08x\r\n", *NFCONF);


    set_flash_lock_boundary(10,2040); //lock FLASH blk 0~9 and 2041~2047 in case.

    *RTCCON = 0x11;
     

    gpio->rGPBDAT = (gpio->rGPBDAT  & 0xFFFFFFFD) | (1 << 1); //enable backlight
    lcd_init();

    //gpio->rGPHCON &= ~((3<<(2*2))|(3<<(3*2))); //set gpio for uart1
    //gpio->rGPHCON |= (2<<(2*2))|(2<<(3*2));

    // uart0->rULCON = 0x3;
    // uart0->rUCON = 0x5 ;
    // uart0->rUFCON = 0x7;
    // uart0->rUMCON = 0;
    // uart0->rUBRDIV = (CPU_GetPclkHZ()/(16*115200))-1;
    // uart0->rUDIVSLOT = 0x0888;

    //uart0_printf("Test ERASE:%d\r\n", Nand_EraseBlock(127));
    //uart0_printf("Test ERASE:%d\r\n", Nand_EraseBlock(128));


    uart0_printf("Starting Tiny Linux loader...\r\n");
    uart0_printf("PCLK:%d MHz, HCLK:%d MHz, FCLK:%d MHz\r\n", 
                CPU_GetPclkHZ()/1000000, CPU_GetHclkHZ()/1000000, CPU_GetFclkHZ()/1000000);
  
     
    Nand_ID_Info nfid;

    Nand_ReadID(&nfid);
    uart0_printf("FlashID: %02x %02x %02x %02x %02x\r\n", nfid.Maker, nfid.Device, nfid.ID_Data3, nfid.ID_Data4,nfid.ID_Data5);
  
    // gpio->rGPHCON &= 3; gpio->rGPKCON = 0;  gpio->rGPLCON = 0; gpio->rGPFCON = 0x00;
    // gpio->rGPBCON = 0x00;gpio->rGPCCON = 0x00;gpio->rGPMCON = 0x00;
    // //gpio->rGPFUDP &= ~(0b11 << (2*2));
    // ////gpio->rGPFUDP |= (0b10 << (2*2)); 
    // 
    // //gpio->rGPFDAT = 0xF;
    //  //gpio->rGPKCON = 0; 
    // while (1)
    // {
    //     uart0_printf("rGPBCON:%08x  rGPBDAT:%08X\r\n",gpio->rGPBCON, gpio->rGPBDAT);
    //     uart0_printf("rGPCCON:%08x  rGPCDAT:%08X\r\n",gpio->rGPCCON, gpio->rGPCDAT);
    //     uart0_printf("rGPDCON:%08x  rGPDDAT:%08X\r\n",gpio->rGPDCON, gpio->rGPDDAT);
    //     uart0_printf("rGPECON:%08x  rGPEDAT:%08X\r\n",gpio->rGPECON, gpio->rGPEDAT);
    //     uart0_printf("rGPFCON:%08x  rGPFDAT:%08X\r\n",gpio->rGPFCON, gpio->rGPFDAT);
    //     uart0_printf("rGPGCON:%08x  rGPGDAT:%08X\r\n",gpio->rGPGCON, gpio->rGPGDAT);
    //     uart0_printf("rGPHCON:%08x  rGPHDAT:%08X\r\n",gpio->rGPHCON, gpio->rGPHDAT); 
    //     uart0_printf("rGPJCON:%08x  rGPJDAT:%08X\r\n",gpio->rGPJCON, gpio->rGPJDAT);
    //     uart0_printf("rGPKCON:%08x  rGPKDAT:%08X\r\n",gpio->rGPKCON, gpio->rGPKDAT);
    //     uart0_printf("rGPLCON:%08x  rGPLDAT:%08X\r\n",gpio->rGPLCON, gpio->rGPLDAT);
    //     uart0_printf("rGPMCON:%08x  rGPMDAT:%08X\r\n",gpio->rGPMCON, gpio->rGPMDAT);
    //     uart0_printf("\r\n\r\n\r\n");
    //     mmu_invalidate_dcache_all();
    //     delay();
    // }


    int res = Nand_ReadPage(8*1048576 / 2048, pgbuf);
    uart0_printf("test rdblk:%d,%d\r\n", 8*1048576 / 2048, res);
    for(int i = 0; i < 32; i++)
        uart0_printf("%02X ", pgbuf[i]);
    uart0_printf("\r\n");

    int rdres = 0;
    uint8_t *kernel_load_addr = (uint8_t *) 0x30008000;
    for(int page = 8 * 64; page < (8 + 8*1048576 / (64*2048)) * 64 ; page++)
    {
        rdres = Nand_ReadPage(page, kernel_load_addr);
        kernel_load_addr += 2048;
        uart0_printf("%d", rdres);
    }
    PrepareATAGS();
    ((void(*)(uint32_t,uint32_t,uint32_t*))(0x30008000))(0, 0x696, (uint32_t *)params_dat);


/*
      for(int blk = 0; blk < 2048; blk++)
      {
          int res = Nand_ReadPage(blk*64, pgbuf);
          uart0_printf("rdblk:%d,%d\r\n", blk, res);
          for(int i = 0; i < 16; i++)
              uart0_printf("%02X ", pgbuf[i]);
          uart0_printf("\r\n");
      }
    
        int res = Nand_ReadPage(2041*64, pgbuf);
        uart0_printf("rdblk:%d,%d\r\n", 2041, res);
        for(int i = 0; i < 2048; i++)
            uart0_printf("%02X ", pgbuf[i]);
        uart0_printf("\r\n");
    
        res = Nand_ReadPage(2047*64, pgbuf);
        uart0_printf("rdblk:%d,%d\r\n", 2047, res);
        for(int i = 0; i < 2048; i++)
            uart0_printf("%02X ", pgbuf[i]);
        uart0_printf("\r\n");  
*/


    //res = Nand_ReadPage(127*64, pgbuf);
    //uart0_printf("rdpg:%d\r\n", res);
    //for(int i = 0; i < 2048; i++)
    //{
    //    uart0_printf("%02X ", pgbuf[i]);
    //}
    //uart0_printf("\r\n");

    for(;;)
    {
        //uart0_printf("GPC:%08x\r\n", gpio->rGPCDAT);
        //uart0_printf("GPD:%08x\r\n", gpio->rGPDDAT);
        //uart0_printf("GPIOK 0-16:");
        /*
        for(int i = 0; i <8;i++)
        {
            //if(!((gpio->rGPGDAT >> i) & 1))
            //    uart0_printf("G:%d\n", i);
            uart0_printf("%d ",(gpio->rGPGDAT >> i) & 1);
        }
        uart0_printf("\r\n");*/

        /*
        for(int i = 0; i <16;i++)
        {
            //if(!((gpio->rGPKDAT >> i) & 1))
            //    uart0_printf("K:%d\n", i);
            uart0_printf("%d ",(gpio->rGPKDAT >> i) & 1);
        }
        uart0_printf("\r\n");*/
        
        //uart0_printf("GPF:%x\r\n",gpio->rGPFDAT);
        //uart0_printf("GPM:%x\r\n",gpio->rGPMDAT);

        //delay();
        //scan_keyboard();
        //uart1->rUTXH = 0x34;
    }

}



