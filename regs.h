
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>


#define	GPIO_REG_PA_BASE	0x56000000

#define UINT32 uint32_t

typedef struct {
	
	UINT32 rGPACON;                  // Port A - offset 0
	UINT32 rGPADAT;                  // Data
	
    UINT32 rPAD1[2];

    UINT32 rGPBCON;                  // Port B - offset 0x10
    UINT32 rGPBDAT;                  // Data
    UINT32 rGPBUDP;                   // Pull-up disable
    UINT32 rPAD2;

    UINT32 rGPCCON;                  // Port C - offset 0x20
    UINT32 rGPCDAT;                  // Data
    UINT32 rGPCUDP;                   // Pull-up disable
    UINT32 rPAD3;
    
    UINT32 rGPDCON;                  // Port D - offset 0x30
    UINT32 rGPDDAT;                  // Data
    UINT32 rGPDUDP;                   // Pull-up disable
    UINT32 rPAD4;
    
    UINT32 rGPECON;                  // Port E - offset 0x40
    UINT32 rGPEDAT;                  // Data
    UINT32 rGPEUDP;                   // Pull-up disable
    UINT32 rPAD5;                 
    
    UINT32 rGPFCON;                  // Port F - offset 0x50
    UINT32 rGPFDAT;
    UINT32 rGPFUDP; 
    UINT32 rPAD6;
    
    UINT32 rGPGCON;                  // Port G - offset 0x60
    UINT32 rGPGDAT;
    UINT32 rGPGUDP; 
    UINT32 rPAD7;
    
    UINT32 rGPHCON;                  // Port H - offset 0x70
    UINT32 rGPHDAT;
    UINT32 rGPHUDP; 
    UINT32 rPAD8;

    UINT32 rMISCCR;                  // misc control reg - offset 0x80
    UINT32 rDCLKCON;                 // DCLK0/1 control reg
    
    UINT32 rEXTINT0;                 // external interrupt control reg 0
    UINT32 rEXTINT1;                 // external interrupt control reg 1
    UINT32 rEXTINT2;                 // external interrupt control reg 2
    
    UINT32 rEINTFLT0;                // reserved
    UINT32 rEINTFLT1;                // reserved
    UINT32 rEINTFLT2;                // external interrupt filter reg 2
    UINT32 rEINTFLT3;                // external interrupt filter reg 3

    UINT32 rEINTMASK;                // external interrupt mask reg
    UINT32 rEINTPEND;                // external interrupt pending reg

    UINT32 rGSTATUS0;                // external pin status
    UINT32 rGSTATUS1;                // chip ID
    UINT32 rGSTATUS2;                // reset status
    UINT32 rGSTATUS3;                // inform register
    UINT32 rGSTATUS4;                // inform register

	UINT32 rDSC0;					// C0 - added by simon
	UINT32 rDSC1;
	UINT32 rDSC2;
	UINT32 rMSLCON;

	UINT32 rGPJCON;					// D0
	UINT32 rGPJDAT;
	UINT32 rGPJUDP;
	UINT32 rPDA9;

	UINT32 rGPKCON;					// E0
	UINT32 rGPKDAT;
	
	UINT32 rGPKUDP;
	
	UINT32 rPDA10;
    
	UINT32 rGPLCON;					// F0
	UINT32 rGPLDAT;
	UINT32 rGPLUDP;
	UINT32 rPDA11;

	UINT32 rGPMCON;					// 100
	UINT32 rGPMDAT;
	UINT32 rGPMUDP;
	UINT32 rPDA12;
} GPIO_REG;  

#define UART0_REG_PA_BASE                  0x50000000            
#define UART1_REG_PA_BASE                  0x50004000            
#define UART2_REG_PA_BASE                  0x50008000    
#define UART3_REG_PA_BASE                  0x5000C000    

typedef struct {
    UINT32 rULCON;                   // line control reg
    UINT32 rUCON;                    // control reg
    UINT32 rUFCON;                   // FIFO control reg
    UINT32 rUMCON;                   // modem control reg

    UINT32 rUTRSTAT;                 // tx/rx status reg
    UINT32 rUERSTAT;                 // rx error status reg
    UINT32 rUFSTAT;                  // FIFO status reg
    UINT32 rUMSTAT;                  // modem status reg

    UINT32 rUTXH;                    // tx buffer reg
    UINT32 rURXH;                    // rx buffer reg
    UINT32 rUBRDIV;                  // baud rate divisor
    UINT32 rUDIVSLOT;                // baud rate divisor
} UART_REG;


#define	 SYSCON_REG_PA_BASE	(0x4C000000)

typedef struct 
{

    UINT32   rLOCKCON0;           // 0x00    // MPLL lock time count register
    UINT32   rLOCKCON1;           // 0x04    // EPLL lock time count register
    UINT32   rOSCSET;             // 0x08
    UINT32   rPAD1;               // 0x0C
    UINT32   rMPLLCON;            // 0x10    // MPLL configuration register
    UINT32   rPAD2;               // 0x14
    UINT32   rEPLLCON;            // 0x18    // EPLL configuration register
    UINT32   rEPLLCON_K;               // 0x1C
    UINT32   rCLKSRC;             // 0x20
    UINT32   rCLKDIV0;            // 0x24
    UINT32   rCLKDIV1;            // 0x28
    UINT32   rCLKDIV2;               // 0x2C
    UINT32   rHCLKCON;            // 0x30
    UINT32   rPCLKCON;            // 0x34
    UINT32   rSCLKCON;            // 0x38
    UINT32   rPAD5;               // 0x3C
    UINT32   rPWRMODE;            // 0x40
    UINT32   rSWRST;              // 0x44        // Software reset control
    UINT32   rPAD6;               // 0x48
    UINT32   rPAD7;               // 0x4C
    UINT32   rBUSPRI0;            // 0x50
    UINT32   rPAD8;               // 0x54
    UINT32   rPAD9;            // 0x58
    UINT32   rENDIAN;              // 0x5C
    UINT32   rPWRCFG;             // 0x60
    UINT32   rRSTCON;             // 0x64
    UINT32   rRSTSTAT;            // 0x68
    UINT32   rWKUPSTAT;           // 0x6C
    UINT32   rINFORM0;            // 0x70
    UINT32   rINFORM1;            // 0x74
    UINT32   rINFORM2;            // 0x78
    UINT32   rINFORM3;            // 0x7C
    UINT32   rUSB_PHYCTRL;               // 0x80
    UINT32   rUSB_PHYPWR;               // 0x84
    UINT32   rUSB_RSTCON;               // 0x88
    UINT32   rUSB_CLKCON;               // 0x8C

//    UINT32   rCLKCON;                 // clock generator control register
//    UINT32   rCLKSLOW;                // slow clock control register
//    UINT32   rCLKDIVN;                // clock divider control register
//    UINT32	 CAMDIVN;				 // camera clock divider register

} SYSCON_REG;





#define CLOCK_BASE  0x4C000000
#define LOCKCON0_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x00))
#define LOCKCON1_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x04))
#define MPLLCON_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x10))
#define EPLLCON_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x18))
#define CLKSRC_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x20))
#define CLKDIV0_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x24))
#define CLKDIV1_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x28))
#define CLKDIV2_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x2C))
#define HCLKCON_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x30))
#define PCLKCON_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x34))
#define SCLKCON_OFS *((volatile uint32_t *)(CLOCK_BASE + 0x38))

#define BANKCFG *((volatile uint32_t *)(0x48000000))
#define BANKCON1 *((volatile uint32_t *)(0x48000004))
#define BANKCON2 *((volatile uint32_t *)(0x48000008))
#define BANKCON3 *((volatile uint32_t *)(0x4800000C))
#define REFRESH *((volatile uint32_t *)(0x48000010))
#define TIMEOUT *((volatile uint32_t *)(0x48000014))

#define WT_BASE  *((volatile uint32_t *)(0x53000000))


#define NFREG_BASE  0x4E000000
volatile uint32_t *NFCONF = (uint32_t *)(NFREG_BASE + 0x0);
volatile uint32_t *NFCONT = (uint32_t *)(NFREG_BASE + 0x4);
volatile uint32_t *NFCMD  = (uint32_t *)(NFREG_BASE + 0x8);
volatile uint32_t *NFADDR  = (uint32_t *)(NFREG_BASE + 0xc);
volatile uint32_t *NFDATA  = (uint32_t *)(NFREG_BASE + 0x10);
volatile uint32_t *NFMECCD0  = (uint32_t *)(NFREG_BASE + 0x14);
volatile uint32_t *NFMECCD1  = (uint32_t *)(NFREG_BASE + 0x18);
volatile uint32_t *NFSECCD  = (uint32_t *)(NFREG_BASE + 0x1c);

volatile uint32_t *NFSBLK = (uint32_t *)(NFREG_BASE + 0x20);
volatile uint32_t *NFEBLK = (uint32_t *)(NFREG_BASE + 0x24);
volatile uint32_t *NFSTAT = (uint32_t *)(NFREG_BASE + 0x28);
volatile uint32_t *NFECCERR0 = (uint32_t *)(NFREG_BASE + 0x2c);
volatile uint32_t *NFECCERR1 = (uint32_t *)(NFREG_BASE + 0x30);
volatile uint32_t *NFMECC0 = (uint32_t *)(NFREG_BASE + 0x34);
volatile uint32_t *NFMECC1 = (uint32_t *)(NFREG_BASE + 0x38);
volatile uint32_t *NFSECC = (uint32_t *)(NFREG_BASE + 0x3c);


volatile uint32_t *RTCCON   =  (volatile uint32_t*)0x57000040;

#define rNFCONT   *NFCONT
#define rNFSTAT   *NFSTAT

#define rNFCMD   *NFCMD
#define rNFADDR  *NFADDR
#define rNFDATA  *NFDATA

#define rNFMECCD0  *NFMECCD0
#define rNFMECCD1  *NFMECCD1
#define rNFSECCD  *NFSECCD
#define rNFECCERR0  *NFECCERR0

#define rNFDATA8  *((volatile uint8_t *)NFDATA)
