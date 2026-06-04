/**
 * =============================================================================
 * uart_driver.c
 * Implementacion del driver LPUART2 para ESP32
 * Proyecto: Scanner Automotriz OBD-II
 * Target:   S32K144EVB-Q100
 * =============================================================================
 *
 * LPUART2 del S32K144 conectado al ESP32 via UART:
 *   - PTD7 -> TX (hacia ESP32 RX)  -> J5-16
 *   - PTD6 <- RX (desde ESP32 TX)  <- J5-18
 *   - 115200 baud, 8 bits, sin paridad, 1 stop bit
 *   - Clock source: FIRCDIV2 = 24 MHz
 *
 * Calculo del Baud Rate:
 *   LPUART Baud = Clock / ((OSR+1) x SBR)
 *
 *   Con Clock = 24 MHz (FIRCDIV2), OSR = 15 (oversampling x16):
 *   SBR = 24,000,000 / (16 x 115200) = 13.02 ~ 13
 *   Baud real = 24,000,000 / (16 x 13) = 115,384 -> Error = +0.16% OK (<3%)
 *
 *   Usamos OSR=15 (mejor oversampling, mas robusto al ruido).
 *
 * Referencias:
 *   - S32K1xx Reference Manual, Cap. 48 (LPUART)
 */

#include "uart_driver.h"
#include "sys_init.h"

/* ========================================================================== */
/*  Constantes de configuracion                                               */
/* ========================================================================== */

/**
 * Oversampling Ratio:
 *   OSR register value = ratio - 1
 *   OSR=15 -> oversampling x16 (maximo, mejor inmunidad al ruido)
 */
#define UART_OSR            15

/**
 * SBR (Baud Rate Modulo Divisor):
 *   SBR = Clock / ((OSR+1) x Baudrate)
 *   SBR = 24,000,000 / (16 x 115200) = 13.02 -> 13
 */
#define UART_SBR            13

/* Verificacion del error de baudrate */
/* Baud real = 24000000 / (16 * 13) = 115384.6 */
/* Error = (115384 - 115200) / 115200 = +0.16% -> OK (< 3%) */

/* ========================================================================== */
/*  Funciones publicas                                                        */
/* ========================================================================== */

void UART_Init(void)
{
    /*
     * La configuracion de PCC (clock enable + source) y pin mux
     * ya fue hecha en SysInit(). Aqui solo configuramos los
     * registros internos del LPUART2.
     *
     * Secuencia de inicializacion LPUART:
     *   1. Asegurar que TX y RX estan deshabilitados
     *   2. Configurar BAUD (SBR, OSR)
     *   3. Configurar CTRL (word length, parity, etc.)
     *   4. Habilitar TX y RX
     */

    /* ===== Paso 1: Deshabilitar TX/RX durante configuracion ===== */
    LPUART2->CTRL &= ~(LPUART_CTRL_TE | LPUART_CTRL_RE);

    /* ===== Paso 2: Configurar Baud Rate ===== */
    /*
     * Registro BAUD:
     *   OSR  [28:24] = 15 (oversampling x16)
     *   SBR  [12:0]  = 11
     *   SBNS [13]    = 0  (1 stop bit)
     *
     * No usamos BOTHEDGE (both edge sampling) porque con OSR=15
     * ya tenemos suficiente oversampling.
     */
    LPUART2->BAUD = ((uint32_t)UART_OSR << LPUART_BAUD_OSR_SHIFT)
                   | ((uint32_t)UART_SBR & LPUART_BAUD_SBR_MASK);
    /* SBNS=0 por default -> 1 stop bit */

    /* ===== Paso 3: Configurar CTRL ===== */
    /*
     * Configuracion 8N1:
     *   M   [4]  = 0 -> 8 bits de datos
     *   PE  [1]  = 0 -> Sin paridad
     *   (SBNS ya configurado en BAUD -> 1 stop bit)
     *
     * Sin interrupciones por ahora (polling mode).
     * Se pueden habilitar despues con RIE para recepcion por IRQ.
     */
    /* CTRL ya es 0 por default (8N1, sin IRQ) -> solo habilitamos TX/RX */

    /* ===== Paso 4: Habilitar Transmisor y Receptor ===== */
    LPUART2->CTRL |= (LPUART_CTRL_TE | LPUART_CTRL_RE);
}

void UART_SendChar(uint8_t byte)
{
    /*
     * Esperar a que TDRE (Transmit Data Register Empty) sea 1.
     * TDRE=1 indica que el registro de datos esta vacio y
     * podemos escribir un nuevo byte.
     */
    while (!(LPUART2->STAT & LPUART_STAT_TDRE))
    {
        /* Busy wait */
    }

    /* Escribir byte al registro de datos (inicia transmision) */
    LPUART2->DATA = (uint32_t)byte;
}

void UART_SendString(const char *str)
{
    if (str == (void *)0)
        return;

    while (*str != '\0')
    {
        UART_SendChar((uint8_t)*str);
        str++;
    }
}

void UART_SendBuffer(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == (void *)0)
        return;

    for (i = 0; i < length; i++)
    {
        UART_SendChar(data[i]);
    }
}

UART_Status_t UART_ReceiveChar(uint8_t *byte, uint32_t timeout)
{
    if (byte == (void *)0)
        return UART_ERROR;

    /*
     * Esperar a que RDRF (Receive Data Register Full) sea 1.
     * RDRF=1 indica que hay un byte disponible para leer.
     */
    if (timeout == 0)
    {
        /* Sin timeout: espera infinita */
        while (!(LPUART2->STAT & LPUART_STAT_RDRF))
        {
            /* Busy wait indefinido */
        }
    }
    else
    {
        /* Con timeout */
        while (!(LPUART2->STAT & LPUART_STAT_RDRF))
        {
            if (--timeout == 0)
                return UART_TIMEOUT;
        }
    }

    /* Leer byte del registro de datos (tambien limpia RDRF) */
    *byte = (uint8_t)(LPUART2->DATA & 0xFF);

    return UART_OK;
}

uint8_t UART_ReadLine(char *buffer, uint8_t max_len)
{
    uint8_t pos = 0;
    uint8_t byte;
    UART_Status_t status;

    if (buffer == (void *)0 || max_len < 2)
        return 0;

    /*
     * Leer caracter por caracter hasta:
     *   - Recibir '\n' (fin de comando)
     *   - Llenar el buffer (max_len - 1, dejando espacio para '\0')
     *   - Timeout sin recibir datos
     *
     * Ignora '\r' (por si el ESP32/App envia "\r\n").
     */
    while (pos < (max_len - 1))
    {
        status = UART_ReceiveChar(&byte, UART_READLINE_TIMEOUT);

        if (status == UART_TIMEOUT)
        {
            /* Timeout: si ya tenemos algo, retornarlo; si no, retornar 0 */
            break;
        }

        if (status != UART_OK)
            break;

        /* Ignorar '\r' (compatibilidad con "\r\n") */
        if (byte == '\r')
            continue;

        /* '\n' = fin de linea -> terminar */
        if (byte == UART_LINE_TERMINATOR)
            break;

        /*
         * Filtrar basura/control del enlace UART/Bluetooth.
         * Si llega 0x00 u otro control antes del comando, antes se guardaba
         * en buffer y process_command() veia un string vacio:
         *   ERR:UNKNOWN_CMD:
         * Los comandos validos son texto ASCII imprimible: PID:0C, SCAN, etc.
         */
        if (byte < 0x20 || byte > 0x7E)
            continue;

        /* Acumular caracter imprimible */
        buffer[pos++] = (char)byte;
    }

    /* Terminar string con null */
    buffer[pos] = '\0';

    return pos;
}

uint8_t UART_DataAvailable(void)
{
    return (LPUART2->STAT & LPUART_STAT_RDRF) ? 1 : 0;
}

void UART_FlushRx(void)
{
    volatile uint32_t dummy;

    /*
     * Leer y descartar todos los bytes pendientes.
     * Seguir mientras RDRF este seteado.
     */
    while (LPUART2->STAT & LPUART_STAT_RDRF)
    {
        dummy = LPUART2->DATA;  /* Leer para limpiar RDRF */
        (void)dummy;
    }
}
