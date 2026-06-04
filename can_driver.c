/**
 * =============================================================================
 * can_driver.c
 * Implementacion del driver FlexCAN0
 * Proyecto: Scanner Automotriz OBD-II
 * Target:   S32K144EVB-Q100
 * =============================================================================
 *
 * FlexCAN0 configurado para OBD-II:
 *   - 500 kbps, Standard ID (11-bit)
 *   - Clock source: Bus clock (24 MHz)
 *   - MB0: Recepcion (filtrado a 0x7E8 = respuesta ECU)
 *   - MB4: Transmision
 *   - Sample Point: 75% (optimo para CAN automotriz)
 *
 * Secuencia de uso tipica:
 *   1. CAN_Init()
 *   2. Llenar CAN_Message_t con request OBD-II
 *   3. CAN_Transmit(&msg)
 *   4. CAN_Receive(&response, timeout)
 *
 * Referencias:
 *   - S32K1xx Reference Manual, Cap. 53 (FlexCAN)
 *   - ISO 15765-4 (CAN de diagnostico)
 *   - SAE J1979 (OBD-II PIDs)
 */

#include "can_driver.h"
#include "sys_init.h"

/* ========================================================================== */
/*  Constantes internas                                                       */
/* ========================================================================== */

/** Timeout para operaciones de freeze/soft reset (iteraciones) */
#define CAN_INIT_TIMEOUT    10000UL

/** Timeout default para transmision (iteraciones) */
#define CAN_TX_TIMEOUT      100000UL

/* ========================================================================== */
/*  Funciones privadas                                                        */
/* ========================================================================== */

/**
 * @brief Entra en modo Freeze (necesario para configurar FlexCAN)
 *
 * FlexCAN solo permite modificar registros de configuracion en Freeze mode.
 * Secuencia: habilitar FRZ + HALT, luego esperar FRZACK.
 *
 * @return CAN_OK si entro en freeze, CAN_TIMEOUT si no
 */
static CAN_Status_t CAN_EnterFreezeMode(void)
{
    uint32_t timeout = CAN_INIT_TIMEOUT;

    CAN0->MCR |= FlexCAN_MCR_FRZ | FlexCAN_MCR_HALT;

    while (!(CAN0->MCR & FlexCAN_MCR_FRZACK) && (timeout > 0))
    {
        timeout--;
    }

    return (timeout > 0) ? CAN_OK : CAN_TIMEOUT;
}

/**
 * @brief Sale de modo Freeze para operacion normal
 *
 * Limpia los bits FRZ y HALT, luego espera a que se desactive FRZACK.
 *
 * @return CAN_OK si salio de freeze, CAN_TIMEOUT si no
 */
static CAN_Status_t CAN_ExitFreezeMode(void)
{
    uint32_t timeout = CAN_INIT_TIMEOUT;

    CAN0->MCR &= ~(FlexCAN_MCR_FRZ | FlexCAN_MCR_HALT);

    while ((CAN0->MCR & FlexCAN_MCR_FRZACK) && (timeout > 0))
    {
        timeout--;
    }

    return (timeout > 0) ? CAN_OK : CAN_TIMEOUT;
}

/**
 * @brief Limpia todos los Message Buffers
 *
 * Pone todos los MBs en estado inactivo y limpia sus datos.
 * Necesario hacer esto en Freeze mode durante la inicializacion.
 */
static void CAN_ClearMessageBuffers(void)
{
    uint8_t i;

    for (i = 0; i < 32; i++)
    {
        CAN0->MB[i].CS    = 0;
        CAN0->MB[i].ID    = 0;
        CAN0->MB[i].DATA[0] = 0;
        CAN0->MB[i].DATA[1] = 0;
    }
}

/* ========================================================================== */
/*  Funciones publicas                                                        */
/* ========================================================================== */

void CAN_Init(void)
{
    /* ===== Paso 1: Habilitar modulo y entrar en Freeze ===== */

    /*
     * Para configurar FlexCAN, primero debemos:
     * 1. Deshabilitar el modulo (MDIS=1) -> esto lo pone en low-power
     * 2. Seleccionar clock source
     * 3. Habilitar el modulo (MDIS=0) -> entra automaticamente en Freeze
     */

    /* Deshabilitar modulo */
    CAN0->MCR |= FlexCAN_MCR_MDIS;

    /* Seleccionar clock source = bus clock (CLKSRC=1 en CTRL1) */
    CAN0->CTRL1 |= FlexCAN_CTRL1_CLKSRC;

    /* Habilitar modulo (quitar MDIS) -> entra en Freeze automaticamente */
    CAN0->MCR &= ~FlexCAN_MCR_MDIS;

    /* Esperar a que el modulo este listo y en Freeze */
    CAN_EnterFreezeMode();

    /* ===== Paso 2: Soft Reset ===== */
    /*
     * Soft reset limpia todos los registros a valores default.
     * Solo funciona en Freeze mode.
     */
    CAN0->MCR |= FlexCAN_MCR_SOFTRST;
    {
        uint32_t timeout = CAN_INIT_TIMEOUT;
        while ((CAN0->MCR & FlexCAN_MCR_SOFTRST) && (timeout > 0))
        {
            timeout--;
        }
    }

    /* Despues del soft reset, re-entrar en Freeze mode */
    CAN_EnterFreezeMode();

    /* Re-seleccionar bus clock despues del reset */
    CAN0->CTRL1 |= FlexCAN_CTRL1_CLKSRC;

    /* ===== Paso 3: Configurar MCR ===== */
    /*
     * MCR Configuration:
     *   - SRXDIS = 1: Deshabilitar self-reception (no queremos eco)
     *   - IRMQ   = 1: Individual Rx Mask (cada MB tiene su propia mascara)
     *   - MAXMB  = 15: Usar MBs 0-15 (suficiente para OBD-II)
     *   - FRZ    = 1: Mantener en freeze por ahora
     *   - HALT   = 1: Mantener halt
     */
    CAN0->MCR = FlexCAN_MCR_FRZ
              | FlexCAN_MCR_HALT
              | FlexCAN_MCR_SRXDIS
              | FlexCAN_MCR_IRMQ
              | (15UL & FlexCAN_MCR_MAXMB_MASK);  /* 16 MBs (0-15) */

    /* ===== Paso 4: Configurar Bit Timing (CTRL1) ===== */
    /*
     * Bus clock = 24 MHz, Target = 500 kbps
     *
     * PRESDIV = 2 -> Tq = (2+1)/24MHz = 125 ns
     * PROPSEG = 6 -> 7 Tq
     * PSEG1   = 3 -> 4 Tq
     * PSEG2   = 3 -> 4 Tq
     * RJW     = 1 -> 2 Tq
     *
     * Total Tq per bit = 1 + 7 + 4 + 4 = 16 Tq
     * Bit Rate = 1 / (16 x 125ns) = 500 kbps OK
     * Sample Point = (1 + 7 + 4) / 16 = 75% OK
     */
    CAN0->CTRL1 = FlexCAN_CTRL1_CLKSRC    /* Bus clock como fuente */
                 | ((uint32_t)CAN_PRESDIV << FlexCAN_CTRL1_PRESDIV_SHIFT)
                 | ((uint32_t)CAN_RJW     << FlexCAN_CTRL1_RJW_SHIFT)
                 | ((uint32_t)CAN_PSEG1   << FlexCAN_CTRL1_PSEG1_SHIFT)
                 | ((uint32_t)CAN_PSEG2   << FlexCAN_CTRL1_PSEG2_SHIFT)
                 | ((uint32_t)CAN_PROPSEG << FlexCAN_CTRL1_PROPSEG_SHIFT);

    /* ===== Paso 5: Limpiar Message Buffers ===== */
    CAN_ClearMessageBuffers();

    /* ===== Paso 6: Configurar MB de recepcion (MB0) ===== */
    /*
     * MB0 configurado para recibir respuestas OBD-II:
     *   - Code = RX_EMPTY (0x4): listo para recibir
     *   - Standard ID = 0x7E8 (respuesta del ECU principal)
     *   - IDE = 0: Standard frame (11-bit ID)
     */
    CAN0->MB[CAN_RX_MB].CS = FlexCAN_MB_CS_CODE_RX_EMPTY;
    CAN0->MB[CAN_RX_MB].ID = (OBD2_RX_ID_ECU1 << FlexCAN_MB_ID_STD_SHIFT);

    /* Mascara individual para MB0: coincidir con todos los bits del ID */
    /* 0x7FF = 11 bits -> solo acepta exactamente 0x7E8 */
    CAN0->RXIMR[CAN_RX_MB] = (0x7FFUL << FlexCAN_MB_ID_STD_SHIFT);

    /* ===== Paso 7: Configurar MB de transmision (MB4) ===== */
    /*
     * MB4 en estado TX_INACTIVE: listo para cargar y enviar.
     */
    CAN0->MB[CAN_TX_MB].CS = FlexCAN_MB_CS_CODE_TX_INACTIVE;
    CAN0->MB[CAN_TX_MB].ID = 0;
    CAN0->MB[CAN_TX_MB].DATA[0] = 0;
    CAN0->MB[CAN_TX_MB].DATA[1] = 0;

    /* ===== Paso 8: Limpiar flags de interrupcion ===== */
    /* Escribir 1 para limpiar (W1C) */
    CAN0->IFLAG1 = 0xFFFFFFFFUL;

    /* ===== Paso 9: Salir de Freeze -> operacion normal ===== */
    CAN_ExitFreezeMode();
}

CAN_Status_t CAN_Transmit(const CAN_Message_t *msg)
{
    uint32_t timeout = CAN_TX_TIMEOUT;
    uint32_t cs_word;
    uint32_t data0, data1;
    uint8_t  i;

    if (msg == (void *)0 || msg->dlc > CAN_MAX_DLC)
        return CAN_ERROR;

    /* ===== Paso 1: Verificar que el MB TX esta libre ===== */
    /*
     * Si hay una transmision previa en curso, esperamos a que termine.
     * Verificamos limpiando el flag del MB.
     */
    if (CAN0->IFLAG1 & (1UL << CAN_TX_MB))
    {
        CAN0->IFLAG1 = (1UL << CAN_TX_MB);   /* Limpiar flag (W1C) */
    }

    /* ===== Paso 2: Poner MB en TX_INACTIVE antes de cargar datos ===== */
    /*
     * IMPORTANTE: Siempre poner INACTIVE antes de modificar ID/DATA.
     * Si no, el modulo podria intentar transmitir datos parciales.
     */
    CAN0->MB[CAN_TX_MB].CS = FlexCAN_MB_CS_CODE_TX_INACTIVE;

    /* ===== Paso 3: Cargar ID (Standard, 11-bit) ===== */
    CAN0->MB[CAN_TX_MB].ID = (msg->id << FlexCAN_MB_ID_STD_SHIFT);

    /* ===== Paso 4: Cargar datos ===== */
    /*
     * FlexCAN almacena datos en formato Big-Endian en 2 words de 32 bits:
     *   DATA[0] = byte[0] en bits [31:24], byte[1] en [23:16], etc.
     *   DATA[1] = byte[4] en bits [31:24], byte[5] en [23:16], etc.
     *
     * Para OBD-II, un request tipico es:
     *   data[0] = 0x02 (numero de bytes de datos adicionales)
     *   data[1] = 0x01 (modo: Show Current Data)
     *   data[2] = PID  (ej: 0x0C = RPM)
     *   data[3..7] = 0x55 (padding ISO-TP, o 0x00)
     */
    data0 = 0;
    data1 = 0;

    for (i = 0; i < msg->dlc && i < 4; i++)
    {
        data0 |= ((uint32_t)msg->data[i]) << (24 - (i * 8));
    }
    for (i = 4; i < msg->dlc && i < 8; i++)
    {
        data1 |= ((uint32_t)msg->data[i]) << (24 - ((i - 4) * 8));
    }

    CAN0->MB[CAN_TX_MB].DATA[0] = data0;
    CAN0->MB[CAN_TX_MB].DATA[1] = data1;

    /* ===== Paso 5: Activar transmision ===== */
    /*
     * Escribir CODE = TX_DATA (0xC) activa la transmision.
     * Tambien configuramos DLC y SRR (para compatibilidad).
     */
    cs_word = FlexCAN_MB_CS_CODE_TX_DATA
            | FlexCAN_MB_CS_SRR
            | ((uint32_t)(msg->dlc & 0xF) << FlexCAN_MB_CS_DLC_SHIFT);

    CAN0->MB[CAN_TX_MB].CS = cs_word;

    /* ===== Paso 6: Esperar confirmacion de transmision ===== */
    /*
     * El flag de IFLAG1 se setea cuando el MB termina de transmitir
     * (despues del ACK del receptor en el bus CAN).
     */
    while (!(CAN0->IFLAG1 & (1UL << CAN_TX_MB)) && (timeout > 0))
    {
        timeout--;
    }

    /* Limpiar flag */
    CAN0->IFLAG1 = (1UL << CAN_TX_MB);

    return (timeout > 0) ? CAN_OK : CAN_TIMEOUT;
}

CAN_Status_t CAN_Receive(CAN_Message_t *msg, uint32_t timeout)
{
    uint32_t cs_word;
    uint32_t data0, data1;
    uint8_t  i, dlc;

    if (msg == (void *)0)
        return CAN_ERROR;

    /* ===== Paso 1: Esperar a que llegue un mensaje ===== */
    /*
     * Polling del flag de IFLAG1 para el MB de recepcion.
     * El flag se setea cuando el MB recibe un mensaje que pasa el filtro.
     */
    while (!(CAN0->IFLAG1 & (1UL << CAN_RX_MB)) && (timeout > 0))
    {
        timeout--;
    }

    if (timeout == 0)
        return CAN_TIMEOUT;

    /* ===== Paso 2: Leer el mensaje (procedimiento de lock) ===== */
    /*
     * PROCEDIMIENTO CORRECTO para leer un MB (Ref Manual 53.5.6):
     *   1. Verificar IFLAG
     *   2. Leer CS (esto "lockea" el MB, previene sobreescritura)
     *   3. Leer ID
     *   4. Leer DATA
     *   5. Leer el registro TIMER (esto "desbloquea" el MB)
     *   6. Limpiar IFLAG
     *
     * Este procedimiento asegura la integridad de los datos.
     */

    /* Leer CS (lock del MB) */
    cs_word = CAN0->MB[CAN_RX_MB].CS;

    /* Extraer DLC */
    dlc = (uint8_t)((cs_word & FlexCAN_MB_CS_DLC_MASK) >> FlexCAN_MB_CS_DLC_SHIFT);
    if (dlc > CAN_MAX_DLC)
        dlc = CAN_MAX_DLC;

    /* Leer ID */
    msg->id = (CAN0->MB[CAN_RX_MB].ID >> FlexCAN_MB_ID_STD_SHIFT) & 0x7FF;

    /* Leer datos */
    data0 = CAN0->MB[CAN_RX_MB].DATA[0];
    data1 = CAN0->MB[CAN_RX_MB].DATA[1];

    for (i = 0; i < dlc && i < 4; i++)
    {
        msg->data[i] = (uint8_t)(data0 >> (24 - (i * 8)));
    }
    for (i = 4; i < dlc && i < 8; i++)
    {
        msg->data[i] = (uint8_t)(data1 >> (24 - ((i - 4) * 8)));
    }

    msg->dlc = dlc;

    /* Leer TIMER para desbloquear el MB */
    (void)CAN0->TIMER;

    /* Limpiar flag de interrupcion (W1C) */
    CAN0->IFLAG1 = (1UL << CAN_RX_MB);

    return CAN_OK;
}

uint8_t CAN_MessageAvailable(void)
{
    return (CAN0->IFLAG1 & (1UL << CAN_RX_MB)) ? 1U : 0U;
}

void CAN_SetRxFilter(uint8_t mb_index, uint32_t id, uint32_t mask)
{
    if (mb_index > 15)
        return;

    /*
     * Para cambiar filtros, necesitamos estar en Freeze mode.
     * Esto es una operacion "pesada" asi que usala con cuidado.
     */
    CAN_EnterFreezeMode();

    /* Configurar MB como RX_EMPTY con el nuevo ID */
    CAN0->MB[mb_index].CS = FlexCAN_MB_CS_CODE_RX_EMPTY;
    CAN0->MB[mb_index].ID = (id << FlexCAN_MB_ID_STD_SHIFT);

    /* Configurar mascara individual */
    CAN0->RXIMR[mb_index] = (mask << FlexCAN_MB_ID_STD_SHIFT);

    CAN_ExitFreezeMode();
}
