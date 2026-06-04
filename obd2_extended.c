/**
 * =============================================================================
 * obd2_extended.c
 * Funciones OBD-II extendidas con ISO-TP multi-frame
 * Proyecto: Scanner Automotriz OBD-II
 * Target:   S32K144EVB-Q100
 * =============================================================================
 *
 * Lectura de VIN (Mode 09, PID 02):
 *
 *   Request (SF):
 *     [02] [09] [02] [55] [55] [55] [55] [55]
 *
 *   Response (multi-frame, ~20 bytes):
 *     Byte 0:    Response SID = 0x49
 *     Byte 1:    PID = 0x02
 *     Byte 2:    Message count = 0x01
 *     Byte 3-19: VIN (17 caracteres ASCII)
 *
 * Lectura de DTCs (Mode 03):
 *
 *   Request (SF):
 *     [01] [03] [55] [55] [55] [55] [55] [55]
 *
 *   Response (puede ser SF o multi-frame):
 *     Byte 0:    Response SID = 0x43
 *     Byte 1:    Numero de DTCs
 *     Byte 2-3:  DTC #1 (2 bytes)
 *     Byte 4-5:  DTC #2 (2 bytes)
 *     ...
 *
 *   Decodificacion de DTC (2 bytes -> codigo alfanumerico):
 *
 *     Bits:  [15:14] [13:12] [11:8]  [7:4]  [3:0]
 *            Tipo    Dig.1   Dig.2   Dig.3  Dig.4
 *
 *     Tipo: 00=P, 01=C, 10=B, 11=U
 *
 *     Ejemplo: 0x0143
 *       [15:14] = 00 -> P
 *       [13:12] = 00 -> 0
 *       [11:8]  = 1  -> 1
 *       [7:4]   = 4  -> 4
 *       [3:0]   = 3  -> 3
 *       -> "P0143"
 */

#include "obd2_extended.h"
#include <stddef.h>  /* NULL */

/* ========================================================================== */
/*  Funciones privadas                                                        */
/* ========================================================================== */

/** Letras de categoria de DTC indexadas por bits [15:14] */
static const char dtc_category_char[4] = { 'P', 'C', 'B', 'U' };

/**
 * @brief Convierte un nibble (0-15) a caracter hex
 */
static char nib_to_hex(uint8_t n)
{
    n &= 0x0F;
    return (n < 10) ? ('0' + n) : ('A' + (n - 10));
}

/**
 * @brief Decodifica 2 bytes crudos a un DTC legible
 *
 * @param byte_a  Byte alto del DTC
 * @param byte_b  Byte bajo del DTC
 * @param dtc     Puntero al DTC donde almacenar el resultado
 */
static void decode_dtc(uint8_t byte_a, uint8_t byte_b, OBD2_DTC_t *dtc)
{
    uint8_t type;
    uint8_t digit1;

    dtc->raw[0] = byte_a;
    dtc->raw[1] = byte_b;

    /* Bits [7:6] del byte A = tipo de DTC */
    type   = (byte_a >> 6) & 0x03;
    dtc->category = type;

    /* Bits [5:4] del byte A = primer digito numerico */
    digit1 = (byte_a >> 4) & 0x03;

    /* Armar string: "P0143\0" */
    dtc->code[0] = dtc_category_char[type];     /* P, C, B, o U */
    dtc->code[1] = '0' + digit1;                /* 0-3 */
    dtc->code[2] = nib_to_hex(byte_a & 0x0F);   /* Segundo digito hex */
    dtc->code[3] = nib_to_hex(byte_b >> 4);      /* Tercer digito hex */
    dtc->code[4] = nib_to_hex(byte_b & 0x0F);    /* Cuarto digito hex */
    dtc->code[5] = '\0';
}

/**
 * @brief Lectura generica de DTCs (usado por Mode 03 y Mode 07)
 *
 * @param mode      OBD2_MODE_READ_DTC (0x03) o 0x07 para pending
 * @param dtc_list  Puntero donde almacenar resultados
 * @return OBD2_OK si la lectura fue exitosa
 */
static OBD2_Status_t read_dtcs_generic(uint8_t mode, OBD2_DTC_List_t *dtc_list)
{
    ISOTP_Session_t session;
    ISOTP_Status_t  tp_status;
    uint8_t request[2];
    uint8_t response_sid;
    uint8_t data_offset;
    if (dtc_list == NULL)
        return OBD2_ERR_PARAM;

    /* Inicializar resultado */
    dtc_list->count = 0;
    dtc_list->valid = 0;

    /* Configurar sesion ISO-TP */
    ISOTP_InitSession(&session, OBD2_TX_ID_ECU1, OBD2_RX_ID_ECU1);

    /* Armar request: [PCI][SID] -> para ISO-TP solo mandamos el payload */
    request[0] = mode;    /* 0x03 o 0x07 */

    /* Enviar request y recibir respuesta completa via ISO-TP */
    tp_status = ISOTP_Transaction(&session, request, 1);

    if (tp_status != ISOTP_OK)
    {
        if (tp_status == ISOTP_ERR_TIMEOUT)
            return OBD2_ERR_TIMEOUT;
        return OBD2_ERR_CAN_TX;
    }

    /* Validar respuesta */
    if (session.received_length < 1)
        return OBD2_ERR_INVALID_RESPONSE;

    /*
     * Formato de la respuesta ISO-TP (ya sin PCI):
     *   Byte 0: Response SID (0x43 para mode 03, 0x47 para mode 07)
     *   Byte 1+: Pares de bytes, cada par es un DTC
     *
     * NOTA: Algunos ECUs ponen un byte de conteo despues del SID,
     * otros van directo a los DTCs. Verificamos el SID primero.
     */
    response_sid = session.payload[0];

    if (response_sid != (mode + OBD2_RESPONSE_OFFSET))
        return OBD2_ERR_INVALID_RESPONSE;

    /* Los DTCs empiezan despues del SID (byte 1) */
    data_offset = 1;

    /* Decodificar DTCs (cada DTC = 2 bytes) */
    while (data_offset + 1 < session.received_length &&
           dtc_list->count < MAX_DTC_COUNT)
    {
        uint8_t byte_a = session.payload[data_offset];
        uint8_t byte_b = session.payload[data_offset + 1];

        /* DTC 0x0000 = padding, ignorar */
        if (byte_a == 0x00 && byte_b == 0x00)
        {
            data_offset += 2;
            continue;
        }

        decode_dtc(byte_a, byte_b, &dtc_list->dtcs[dtc_list->count]);
        dtc_list->count++;
        data_offset += 2;
    }

    dtc_list->valid = 1;
    return OBD2_OK;
}

/* ========================================================================== */
/*  Funciones publicas                                                        */
/* ========================================================================== */

OBD2_Status_t OBD2_ReadVIN(OBD2_VIN_t *vin_out)
{
    ISOTP_Session_t session;
    ISOTP_Status_t  tp_status;
    uint8_t request[2];
    uint8_t vin_start;
    uint8_t i;

    if (vin_out == NULL)
        return OBD2_ERR_PARAM;

    /* Inicializar resultado */
    vin_out->valid = 0;
    for (i = 0; i <= VIN_LENGTH; i++)
        vin_out->vin[i] = '\0';

    /* Configurar sesion ISO-TP */
    ISOTP_InitSession(&session, OBD2_TX_ID_ECU1, OBD2_RX_ID_ECU1);

    /* Request Mode 09, PID 02 (VIN) */
    request[0] = OBD2_MODE_VEHICLE_INFO;   /* 0x09 */
    request[1] = PID_VIN;                  /* 0x02 */

    /* Transaccion ISO-TP completa */
    tp_status = ISOTP_Transaction(&session, request, 2);

    if (tp_status != ISOTP_OK)
    {
        if (tp_status == ISOTP_ERR_TIMEOUT)
            return OBD2_ERR_TIMEOUT;
        return OBD2_ERR_CAN_TX;
    }

    /*
     * Respuesta esperada (payload ISO-TP, sin PCI):
     *   Byte 0: 0x49 (response SID = 0x09 + 0x40)
     *   Byte 1: 0x02 (PID confirmado)
     *   Byte 2: 0x01 (message count, siempre 1 para VIN)
     *   Byte 3-19: VIN (17 caracteres ASCII)
     *
     * Total esperado: 20 bytes (3 header + 17 VIN)
     */
    if (session.received_length < 20)
        return OBD2_ERR_INVALID_RESPONSE;

    /* Verificar Response SID */
    if (session.payload[0] != (OBD2_MODE_VEHICLE_INFO + OBD2_RESPONSE_OFFSET))
        return OBD2_ERR_INVALID_RESPONSE;

    /* Verificar PID */
    if (session.payload[1] != PID_VIN)
        return OBD2_ERR_INVALID_RESPONSE;

    /* Extraer VIN (17 caracteres empezando en byte 3) */
    vin_start = 3;
    for (i = 0; i < VIN_LENGTH; i++)
    {
        uint8_t ch = session.payload[vin_start + i];

        /* Validar que sea un caracter ASCII imprimible */
        if (ch >= 0x20 && ch <= 0x7E)
            vin_out->vin[i] = (char)ch;
        else
            vin_out->vin[i] = '?';   /* Caracter invalido */
    }
    vin_out->vin[VIN_LENGTH] = '\0';

    vin_out->valid = 1;
    return OBD2_OK;
}

OBD2_Status_t OBD2_ReadDTCs(OBD2_DTC_List_t *dtc_list)
{
    return read_dtcs_generic(OBD2_MODE_READ_DTC, dtc_list);
}

OBD2_Status_t OBD2_ReadPendingDTCs(OBD2_DTC_List_t *dtc_list)
{
    /* Mode 07 = Pending DTCs (misma estructura que mode 03) */
    return read_dtcs_generic(0x07, dtc_list);
}

uint8_t OBD2_FormatDTC(const OBD2_DTC_t *dtc, char *buffer, uint8_t buflen)
{
    uint8_t pos = 0;

    if (dtc == NULL || buffer == NULL || buflen < 11)
        return 0;

    /* Formato: "DTC:P0143\n\0" = 11 bytes minimo */
    buffer[pos++] = 'D';
    buffer[pos++] = 'T';
    buffer[pos++] = 'C';
    buffer[pos++] = ':';
    buffer[pos++] = dtc->code[0];   /* P/C/B/U */
    buffer[pos++] = dtc->code[1];
    buffer[pos++] = dtc->code[2];
    buffer[pos++] = dtc->code[3];
    buffer[pos++] = dtc->code[4];
    buffer[pos++] = '\n';
    buffer[pos]   = '\0';

    return pos;
}
