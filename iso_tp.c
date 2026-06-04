/**
 * =============================================================================
 * iso_tp.c
 * Implementacion de ISO 15765-2 Transport Protocol
 * Proyecto: Scanner Automotriz OBD-II
 * Target:   S32K144EVB-Q100
 * =============================================================================
 *
 * Ejemplo de flujo multi-frame (lectura de VIN):
 *
 *   TESTER -> ECU:
 *     [02] [09] [02] [55] [55] [55] [55] [55]   <- SF: Mode 09, PID 02 (VIN)
 *
 *   ECU -> TESTER:
 *     [10] [14] [49] [02] [01] [57] [42] [41]   <- FF: total=20, "WBA..."
 *     ^ First Frame: PCI=0x10, len=0x014=20 bytes
 *
 *   TESTER -> ECU:
 *     [30] [00] [0A] [55] [55] [55] [55] [55]   <- FC: CTS, BS=0, STmin=10ms
 *
 *   ECU -> TESTER:
 *     [21] [48] [5A] [5A] [42] [56] [31] [33]   <- CF seq=1: "HZZBV13"
 *     [22] [37] [4E] [55] [55] [55] [55] [55]   <- CF seq=2: "7N" + padding
 *
 *   Resultado re-ensamblado (20 bytes):
 *     [49][02][01] "WBAHZZBV137N....."
 *     ^ SID 0x49 = response mode 09, PID 0x02, msg count=01, VIN data
 *
 * Notas de implementacion:
 *   - Solo Standard CAN (11-bit ID), no Extended
 *   - Block Size = 0 -> el ECU envia todos los CFs sin pausar
 *   - STmin = 10 ms -> minimo tiempo entre CFs (el ECU respeta esto)
 *   - Maximo payload = 256 bytes (suficiente para OBD-II)
 *
 * Referencias:
 *   - ISO 15765-2:2016, Secciones 9-10
 */

#include "iso_tp.h"
#include <stddef.h>   /* NULL */

/* ========================================================================== */
/*  Funciones privadas                                                        */
/* ========================================================================== */

/**
 * @brief Envia un Single Frame (payload <= 7 bytes)
 *
 * Formato SF:
 *   Byte 0:  0x0N (N = numero de bytes de datos, 1-7)
 *   Byte 1-N: datos
 *   Byte N+1..7: padding (0x55)
 */
static ISOTP_Status_t isotp_send_sf(uint32_t tx_id,
                                     const uint8_t *data, uint8_t length)
{
    CAN_Message_t frame;
    uint8_t i;

    frame.id  = tx_id;
    frame.dlc = 8;

    /* PCI: tipo SF (0x0) + largo */
    frame.data[0] = ISOTP_PCI_SF | (length & 0x0F);

    /* Copiar datos */
    for (i = 0; i < length && i < 7; i++)
    {
        frame.data[1 + i] = data[i];
    }

    /* Padding */
    for (i = 1 + length; i < 8; i++)
    {
        frame.data[i] = ISOTP_PADDING_BYTE;
    }

    return (CAN_Transmit(&frame) == CAN_OK) ? ISOTP_OK : ISOTP_ERR_CAN;
}

/**
 * @brief Envia un First Frame (inicio de multi-frame, payload > 7 bytes)
 *
 * Formato FF:
 *   Byte 0:  0x1H (H = nibble alto del largo total)
 *   Byte 1:  byte bajo del largo total
 *   Byte 2-7: primeros 6 bytes de datos
 *
 * El largo total va en 12 bits: max 4095 bytes.
 */
static ISOTP_Status_t isotp_send_ff(uint32_t tx_id,
                                     const uint8_t *data, uint16_t total_length)
{
    CAN_Message_t frame;
    uint8_t i;

    frame.id  = tx_id;
    frame.dlc = 8;

    /* PCI: tipo FF (0x1) + largo total en 12 bits */
    frame.data[0] = ISOTP_PCI_FF | (uint8_t)((total_length >> 8) & 0x0F);
    frame.data[1] = (uint8_t)(total_length & 0xFF);

    /* Primeros 6 bytes de datos */
    for (i = 0; i < 6 && i < total_length; i++)
    {
        frame.data[2 + i] = data[i];
    }

    return (CAN_Transmit(&frame) == CAN_OK) ? ISOTP_OK : ISOTP_ERR_CAN;
}

/**
 * @brief Envia un Consecutive Frame
 *
 * Formato CF:
 *   Byte 0:    0x2N (N = sequence number, 0-F, wraps around)
 *   Byte 1-7:  hasta 7 bytes de datos (padding si es el ultimo)
 */
static ISOTP_Status_t isotp_send_cf(uint32_t tx_id,
                                     const uint8_t *data, uint8_t length,
                                     uint8_t seq_num)
{
    CAN_Message_t frame;
    uint8_t i;

    frame.id  = tx_id;
    frame.dlc = 8;

    /* PCI: tipo CF + sequence number (0-F) */
    frame.data[0] = ISOTP_PCI_CF | (seq_num & 0x0F);

    /* Copiar datos (hasta 7 bytes) */
    for (i = 0; i < length && i < 7; i++)
    {
        frame.data[1 + i] = data[i];
    }

    /* Padding si quedan bytes libres */
    for (i = 1 + length; i < 8; i++)
    {
        frame.data[i] = ISOTP_PADDING_BYTE;
    }

    return (CAN_Transmit(&frame) == CAN_OK) ? ISOTP_OK : ISOTP_ERR_CAN;
}

/**
 * @brief Envia un Flow Control frame
 *
 * Formato FC:
 *   Byte 0:  0x30 + Flow Status (0=CTS, 1=Wait, 2=Overflow)
 *   Byte 1:  Block Size (0 = sin limite)
 *   Byte 2:  STmin (ms entre CFs)
 *   Byte 3-7: padding
 *
 * Nosotros siempre enviamos CTS (Continue To Send) con BS=0
 * para que el ECU envie todo de corrido.
 */
static ISOTP_Status_t isotp_send_fc(uint32_t tx_id,
                                     uint8_t flow_status,
                                     uint8_t block_size,
                                     uint8_t stmin)
{
    CAN_Message_t frame;

    frame.id  = tx_id;
    frame.dlc = 8;

    frame.data[0] = ISOTP_PCI_FC | (flow_status & 0x0F);
    frame.data[1] = block_size;
    frame.data[2] = stmin;
    frame.data[3] = ISOTP_PADDING_BYTE;
    frame.data[4] = ISOTP_PADDING_BYTE;
    frame.data[5] = ISOTP_PADDING_BYTE;
    frame.data[6] = ISOTP_PADDING_BYTE;
    frame.data[7] = ISOTP_PADDING_BYTE;

    return (CAN_Transmit(&frame) == CAN_OK) ? ISOTP_OK : ISOTP_ERR_CAN;
}

/**
 * @brief Espera y recibe una trama CAN con timeout
 *
 * Wrapper sobre CAN_Receive para uso interno de ISO-TP.
 */
static ISOTP_Status_t isotp_receive_frame(CAN_Message_t *frame, uint32_t timeout)
{
    CAN_Status_t status = CAN_Receive(frame, timeout);

    if (status == CAN_TIMEOUT)
        return ISOTP_ERR_TIMEOUT;
    if (status != CAN_OK)
        return ISOTP_ERR_CAN;

    return ISOTP_OK;
}

/**
 * @brief Delay simple entre CFs (respeta STmin del FC recibido)
 *
 * En una implementacion real con timer esto seria preciso.
 * Aqui usamos un busy-wait aproximado.
 */
static void isotp_delay_ms(uint8_t ms)
{
    volatile uint32_t count;
    uint32_t total = (uint32_t)ms;

    for (; total > 0; total--)
    {
        for (count = 0; count < 8000UL; count++)
        {
            /* ~1ms a 80 MHz (aprox) */
        }
    }
}

/* ========================================================================== */
/*  Funciones publicas                                                        */
/* ========================================================================== */

void ISOTP_InitSession(ISOTP_Session_t *session, uint32_t tx_id, uint32_t rx_id)
{
    uint16_t i;

    if (session == NULL)
        return;

    session->total_length    = 0;
    session->received_length = 0;
    session->next_seq        = 0;
    session->state           = ISOTP_IDLE;
    session->tx_id           = tx_id;
    session->rx_id           = rx_id;

    for (i = 0; i < ISOTP_MAX_PAYLOAD; i++)
        session->payload[i] = 0;
}

ISOTP_Status_t ISOTP_Send(ISOTP_Session_t *session,
                           const uint8_t *data, uint16_t length)
{
    ISOTP_Status_t status;
    CAN_Message_t  fc_frame;
    uint16_t       offset;
    uint8_t        seq_num;
    uint8_t        chunk_len;
    uint8_t        fc_block_size;
    uint8_t        fc_stmin;
    uint8_t        block_count;

    if (session == NULL || data == NULL || length == 0)
        return ISOTP_ERR_PARAM;

    if (length > 4095)
        return ISOTP_ERR_OVERFLOW;

    /* ===== Caso 1: Single Frame (<= 7 bytes) ===== */
    if (length <= 7)
    {
        return isotp_send_sf(session->tx_id, data, (uint8_t)length);
    }

    /* ===== Caso 2: Multi-Frame (> 7 bytes) ===== */

    /* Enviar First Frame (primeros 6 bytes de datos) */
    status = isotp_send_ff(session->tx_id, data, length);
    if (status != ISOTP_OK)
        return status;

    /* Esperar Flow Control del receptor */
    status = isotp_receive_frame(&fc_frame, ISOTP_CF_TIMEOUT);
    if (status != ISOTP_OK)
        return status;

    /* Verificar que es un FC */
    if ((fc_frame.data[0] & ISOTP_PCI_TYPE_MASK) != ISOTP_PCI_FC)
        return ISOTP_ERR_CAN;

    /* Parsear FC */
    {
        uint8_t fs = fc_frame.data[0] & 0x0F;
        if (fs == ISOTP_FC_OVERFLOW)
            return ISOTP_ERR_OVERFLOW;
        if (fs == ISOTP_FC_WAIT)
        {
            /* Simplificado: no manejamos WAIT, re-intentar seria mas robusto */
            return ISOTP_ERR_TIMEOUT;
        }
        /* fs == ISOTP_FC_CTS -> continuar */
    }

    fc_block_size = fc_frame.data[1];   /* 0 = sin limite */
    fc_stmin      = fc_frame.data[2];   /* ms entre CFs */

    /* Enviar Consecutive Frames */
    offset    = 6;      /* Ya enviamos 6 bytes en el FF */
    seq_num   = 1;      /* CF sequence empieza en 1 */
    block_count = 0;

    while (offset < length)
    {
        /* Calcular bytes restantes para este CF */
        chunk_len = (uint8_t)((length - offset > 7) ? 7 : (length - offset));

        /* Respetar STmin entre CFs */
        if (fc_stmin > 0)
            isotp_delay_ms(fc_stmin);

        /* Enviar CF */
        status = isotp_send_cf(session->tx_id, &data[offset], chunk_len, seq_num);
        if (status != ISOTP_OK)
            return status;

        offset  += chunk_len;
        seq_num  = (seq_num + 1) & 0x0F;   /* Wrap 0-F */
        block_count++;

        /* Si hay block size, esperar nuevo FC cada N bloques */
        if (fc_block_size > 0 && block_count >= fc_block_size && offset < length)
        {
            status = isotp_receive_frame(&fc_frame, ISOTP_CF_TIMEOUT);
            if (status != ISOTP_OK)
                return status;

            if ((fc_frame.data[0] & ISOTP_PCI_TYPE_MASK) != ISOTP_PCI_FC)
                return ISOTP_ERR_CAN;

            block_count = 0;
        }
    }

    return ISOTP_OK;
}

ISOTP_Status_t ISOTP_Receive(ISOTP_Session_t *session, uint32_t timeout)
{
    CAN_Message_t  frame;
    ISOTP_Status_t status;
    uint8_t pci_type;
    uint8_t i;
    uint16_t copy_len;

    if (session == NULL)
        return ISOTP_ERR_PARAM;

    /* Reset del estado de recepcion */
    session->received_length = 0;
    session->total_length    = 0;
    session->next_seq        = 1;
    session->state           = ISOTP_IDLE;

    /* ===== Esperar primera trama (SF o FF) ===== */
    status = isotp_receive_frame(&frame, timeout);
    if (status != ISOTP_OK)
    {
        session->state = ISOTP_ERROR;
        return status;
    }

    pci_type = frame.data[0] & ISOTP_PCI_TYPE_MASK;

    /* ===== Caso 1: Single Frame ===== */
    if (pci_type == ISOTP_PCI_SF)
    {
        uint8_t sf_len = frame.data[0] & 0x0F;

        if (sf_len == 0 || sf_len > 7)
        {
            session->state = ISOTP_ERROR;
            return ISOTP_ERR_PARAM;
        }


        /* Copiar datos del SF (bytes 1 a sf_len) */
        for (i = 0; i < sf_len; i++)
        {
            session->payload[i] = frame.data[1 + i];
        }

        session->total_length    = sf_len;
        session->received_length = sf_len;
        session->state           = ISOTP_COMPLETE;

        return ISOTP_OK;
    }

    /* ===== Caso 2: First Frame -> Multi-frame ===== */
    if (pci_type == ISOTP_PCI_FF)
    {
        /* Extraer largo total (12 bits) */
        session->total_length = (uint16_t)(frame.data[0] & 0x0F) << 8;
        session->total_length |= (uint16_t)frame.data[1];

        if (session->total_length > ISOTP_MAX_PAYLOAD)
        {
            /* Enviar FC con overflow y abortar */
            isotp_send_fc(session->tx_id, ISOTP_FC_OVERFLOW, 0, 0);
            session->state = ISOTP_ERROR;
            return ISOTP_ERR_OVERFLOW;
        }

        /* Copiar primeros 6 bytes de datos del FF */
        copy_len = session->total_length;
        if (copy_len > 6)
            copy_len = 6;

        for (i = 0; i < copy_len; i++)
        {
            session->payload[i] = frame.data[2 + i];
        }

        session->received_length = copy_len;
        session->next_seq        = 1;
        session->state           = ISOTP_RECEIVING;

        /* Enviar Flow Control: ContinueToSend, BS=0, STmin=10ms */
        status = isotp_send_fc(session->tx_id,
                                ISOTP_FC_CTS,
                                ISOTP_FC_BLOCK_SIZE,
                                ISOTP_FC_STMIN);
        if (status != ISOTP_OK)
        {
            session->state = ISOTP_ERROR;
            return status;
        }

        /* ===== Recolectar Consecutive Frames ===== */
        while (session->received_length < session->total_length)
        {
            status = isotp_receive_frame(&frame, ISOTP_CF_TIMEOUT);
            if (status != ISOTP_OK)
            {
                session->state = ISOTP_ERROR;
                return status;
            }

            /* Verificar que es un CF */
            pci_type = frame.data[0] & ISOTP_PCI_TYPE_MASK;
            if (pci_type != ISOTP_PCI_CF)
            {
                session->state = ISOTP_ERROR;
                return ISOTP_ERR_SEQUENCE;
            }

            /* Verificar sequence number */
            {
                uint8_t recv_seq = frame.data[0] & 0x0F;
                if (recv_seq != session->next_seq)
                {
                    session->state = ISOTP_ERROR;
                    return ISOTP_ERR_SEQUENCE;
                }
                session->next_seq = (session->next_seq + 1) & 0x0F;
            }

            /* Copiar hasta 7 bytes de datos del CF */
            {
                uint16_t remaining = session->total_length - session->received_length;
                copy_len = (remaining > 7) ? 7 : remaining;

                for (i = 0; i < copy_len; i++)
                {
                    if (session->received_length < ISOTP_MAX_PAYLOAD)
                    {
                        session->payload[session->received_length] = frame.data[1 + i];
                        session->received_length++;
                    }
                }
            }
        }

        session->state = ISOTP_COMPLETE;
        return ISOTP_OK;
    }

    /* Tipo de frame inesperado */
    session->state = ISOTP_ERROR;
    return ISOTP_ERR_CAN;
}

ISOTP_Status_t ISOTP_Transaction(ISOTP_Session_t *session,
                                  const uint8_t *req_data, uint16_t req_len)
{
    ISOTP_Status_t status;

    if (session == NULL || req_data == NULL || req_len == 0)
        return ISOTP_ERR_PARAM;

    /* Enviar request (normalmente SF para OBD-II) */
    status = ISOTP_Send(session, req_data, req_len);
    if (status != ISOTP_OK)
        return status;

    /* Recibir respuesta completa (SF o multi-frame) */
    return ISOTP_Receive(session, ISOTP_FF_TIMEOUT);
}
