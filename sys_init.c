/**
 * =============================================================================
 * sys_init.c
 * Inicializacion del sistema para el NXP S32K144
 * Proyecto: Scanner Automotriz OBD-II
 * Target:   S32K144EVB-Q100
 * =============================================================================
 *
 * Usa FIRC (oscilador interno 48 MHz) -- no depende de cristal externo ni PLL.
 * Mas robusto para arranque desde debugger.
 *
 *   Core/System = 48 MHz (FIRC / 1)
 *   Bus clock   = 24 MHz (FIRC / 2)
 *   Flash clock = 24 MHz (FIRC / 2)
 *   LPUART2     = 24 MHz (FIRCDIV2)
 *   LPSPI1      = 24 MHz (FIRCDIV2) para SBC UJA1169
 */

#include "sys_init.h"

static void WDOG_Disable(void)
{
    WDOG_CNT   = 0xD928C520UL;
    WDOG_CNT   = 0xD928D480UL;
    WDOG_CS    = WDOG_CS_UPDATE;
    WDOG_TOVAL = 0xFFFFUL;
}

static void FIRC_Init(void)
{
    /* FIRC ya activo al reset -- solo configurar divisores */
    SCG->FIRCDIV = (1UL << 0)   /* FIRCDIV1 = /1 -> 48 MHz */
                 | (2UL << 8);  /* FIRCDIV2 = /2 -> 24 MHz */

    /* Esperar FIRCVLD */
    while (!(SCG->FIRCCSR & (1UL << 24)));
}

static void RUN_Mode_Config(void)
{
    /* FIRC como fuente, Core=48MHz, Bus=24MHz, Flash=24MHz */
    SCG->RCCR = SCG_RCCR_SCS_FIRC
              | (0UL << SCG_RCCR_DIVCORE_SHIFT)
              | (1UL << SCG_RCCR_DIVBUS_SHIFT)
              | (1UL << SCG_RCCR_DIVSLOW_SHIFT);

    while (((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 3UL);
}

static void PCC_Config(void)
{
    PCC_PORTA |= PCC_CGC;
    PCC_PORTB |= PCC_CGC;
    PCC_PORTC |= PCC_CGC;
    PCC_PORTD |= PCC_CGC;
    PCC_PORTE |= PCC_CGC;

    PCC_FlexCAN0 &= ~PCC_CGC;
    PCC_FlexCAN0 |= PCC_CGC;

    PCC_LPUART2 &= ~PCC_CGC;
    PCC_LPUART2 &= ~PCC_PCS_MASK;
    PCC_LPUART2 |= PCC_PCS_FIRC;   /* FIRCDIV2 = 24 MHz */
    PCC_LPUART2 |= PCC_CGC;

    /* LPSPI1 para configurar el SBC UJA1169 integrado del EVB */
    PCC_LPSPI1 &= ~PCC_CGC;
    PCC_LPSPI1 &= ~PCC_PCS_MASK;
    PCC_LPSPI1 |= PCC_PCS_FIRC;   /* FIRCDIV2 = 24 MHz */
    PCC_LPSPI1 |= PCC_CGC;
}

static void PIN_Mux_Config(void)
{
    PORTE->PCR[4] = PORT_PCR_MUX(5);   /* PTE4 = CAN0_RX */
    PORTE->PCR[5] = PORT_PCR_MUX(5);   /* PTE5 = CAN0_TX */

    PORTD->PCR[6] = PORT_PCR_MUX(2);   /* PTD6 = LPUART2_RX -> J5-18 */
    PORTD->PCR[7] = PORT_PCR_MUX(2);   /* PTD7 = LPUART2_TX -> J5-16 */

    PORTB->PCR[14] = PORT_PCR_MUX(3);  /* PTB14 = LPSPI1_SCK  -> UJA1169 SCK */
    PORTB->PCR[15] = PORT_PCR_MUX(3);  /* PTB15 = LPSPI1_SIN  <- UJA1169 SDO */
    PORTB->PCR[16] = PORT_PCR_MUX(3);  /* PTB16 = LPSPI1_SOUT -> UJA1169 SDI */
    PORTB->PCR[17] = PORT_PCR_MUX(3);  /* PTB17 = LPSPI1_PCS3 -> UJA1169 SCSN */

    PORTD->PCR[15] = PORT_PCR_MUX(1);  /* LED Rojo */
    PORTD->PCR[16] = PORT_PCR_MUX(1);  /* LED Verde */
    PORTD->PCR[0]  = PORT_PCR_MUX(1);  /* LED Azul */

    PTD->PDDR |= (1UL << 15) | (1UL << 16) | (1UL << 0);
    PTD->PSOR  = (1UL << 15) | (1UL << 16) | (1UL << 0);
}

void SysInit(void)
{
    WDOG_Disable();
    FIRC_Init();
    RUN_Mode_Config();
    PCC_Config();
    PIN_Mux_Config();
}


/* ========================================================================== */
/*  SBC UJA1169                                                               */
/* ========================================================================== */

#define UJA1169_REG_MODE_CONTROL       0x01U
#define UJA1169_REG_CAN_CONTROL        0x20U

#define UJA1169_MODE_NORMAL            0x07U
#define UJA1169_CAN_ACTIVE             0x01U

#define SBC_SPI_TIMEOUT                100000UL
#define SBC_SPI_PCS_UJA1169            3U

static void LPSPI1_InitForSBC(void)
{
    /* Reset logico del modulo */
    LPSPI1->CR = LPSPI_CR_RST;
    LPSPI1->CR = 0;

    /* FIFO simple: sin DMA, watermark default */
    LPSPI1->CR = LPSPI_CR_RTF | LPSPI_CR_RRF;

    /* Master mode. NOSTALL evita bloqueos si RX FIFO no se lee inmediatamente. */
    LPSPI1->CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_NOSTALL;

    /*
     * UJA1169: SCK idle low, muestreo en flanco de bajada y shift en subida.
     * Eso corresponde a CPOL=0, CPHA=1. Transferencias de 16 bits.
     * Con clock de 24 MHz y SCKDIV=11 -> SCK ~ 2 MHz, conservador para el SBC.
     */
    LPSPI1->CCR = (11UL << LPSPI_CCR_SCKDIV_SHIFT)
                | (2UL  << LPSPI_CCR_DBT_SHIFT)
                | (2UL  << LPSPI_CCR_PCSSCK_SHIFT)
                | (2UL  << LPSPI_CCR_SCKPCS_SHIFT);

    LPSPI1->TCR = (15UL << LPSPI_TCR_FRAMESZ_SHIFT)    /* 16 bits */
                | ((uint32_t)SBC_SPI_PCS_UJA1169 << LPSPI_TCR_PCS_SHIFT)
                | LPSPI_TCR_CPHA;

    LPSPI1->CR = LPSPI_CR_MEN;
}

static uint8_t SBC_SPI_Transfer16(uint8_t address, uint8_t data, uint8_t read_only)
{
    uint32_t timeout;
    uint16_t tx_word;
    uint32_t rx_word;

    /* Primer byte: A6..A0 + bit RO en LSB. RO=0 escribe, RO=1 lee. */
    tx_word = (uint16_t)((((uint16_t)address & 0x7FU) << 1) | (read_only ? 1U : 0U));
    tx_word = (uint16_t)((tx_word << 8) | data);

    /* Limpiar FIFOs antes de la transaccion */
    LPSPI1->CR = LPSPI_CR_MEN | LPSPI_CR_RTF | LPSPI_CR_RRF;

    timeout = SBC_SPI_TIMEOUT;
    while (!(LPSPI1->SR & LPSPI_SR_TDF) && timeout > 0U)
        timeout--;
    if (timeout == 0U)
        return 0U;

    LPSPI1->TDR = tx_word;

    timeout = SBC_SPI_TIMEOUT;
    while (!(LPSPI1->SR & LPSPI_SR_RDF) && timeout > 0U)
        timeout--;
    if (timeout == 0U)
        return 0U;

    rx_word = LPSPI1->RDR;
    return (uint8_t)(rx_word & 0xFFU);
}

static void SBC_WriteReg(uint8_t address, uint8_t value)
{
    (void)SBC_SPI_Transfer16(address, value, 0U);
}

void SBC_Init(void)
{
    volatile uint32_t delay;

    LPSPI1_InitForSBC();

    /* Espera conservadora despues de reset antes del primer acceso SPI al UJA1169. */
    for (delay = 0; delay < 50000UL; delay++) { }

    /* Sacar el SBC de Standby y activar el transceptor CAN integrado. */
    SBC_WriteReg(UJA1169_REG_MODE_CONTROL, UJA1169_MODE_NORMAL);
    SBC_WriteReg(UJA1169_REG_CAN_CONTROL,  UJA1169_CAN_ACTIVE);
}
