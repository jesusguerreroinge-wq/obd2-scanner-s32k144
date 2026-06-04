/**
 * =============================================================================
 * obd2.c
 * Implementacion de la capa OBD-II sobre CAN (ISO 15765-4)
 * Proyecto: Scanner Automotriz OBD-II
 * Target:   S32K144EVB-Q100
 * =============================================================================
 *
 * Estructura de una trama OBD-II Single Frame sobre CAN:
 *
 *   CAN ID: 0x7DF (broadcast) o 0x7E0 (ECU especifico)
 *   DLC: 8 (siempre 8 bytes en ISO 15765-4)
 *
 *   Byte:  [0]    [1]    [2]     [3]     [4]    [5]    [6]    [7]
 *          PCI    SID    PID    (frame)  0x55   0x55   0x55   0x55
 *
 *   PCI  = No. de bytes significativos que siguen (ej: 0x02 para Mode01+PID)
 *   SID  = Service ID (modo): 0x01, 0x02, 0x03, etc.
 *   PID  = Parameter ID solicitado
 *   0x55 = Padding (ISO 15765-4)
 *
 * Respuesta del ECU (ejemplo RPM, CAN ID = 0x7E8):
 *
 *   Byte:  [0]    [1]    [2]     [3]     [4]    [5]    [6]    [7]
 *          0x04   0x41   0x0C    A       B      0x55   0x55   0x55
 *
 *   0x04 = 4 bytes significativos siguen
 *   0x41 = Response SID (0x01 + 0x40)
 *   0x0C = PID confirmado (RPM)
 *   A, B = Datos -> RPM = (256*A + B) / 4
 *
 * Referencias:
 *   - SAE J1979 (definicion de PIDs y formulas)
 *   - ISO 15765-2 (transporte CAN para diagnostico)
 */

#include "obd2.h"
#include <stddef.h>   /* NULL */

/* ========================================================================== */
/*  Catalogo de PIDs conocidos                                                */
/* ========================================================================== */

/**
 * Tabla estatica con la informacion de cada PID que el scanner soporta.
 * data_bytes = bytes de datos en la respuesta (no cuenta PCI, SID, PID).
 *
 * Para agregar un nuevo PID:
 *   1. Definir el #define en obd2.h
 *   2. Agregar la entrada aqui
 *   3. Agregar su formula en OBD2_DecodePID()
 */
static const OBD2_PID_Info_t pid_catalog[] = {
    /* PID   bytes  unidad               nombre completo                     nombre corto */
    { 0x00,  4, OBD2_UNIT_NONE,      "PIDs soportados [01-20]",           "PIDS_A"    },
    { 0x01,  4, OBD2_UNIT_NONE,      "Monitor Status",                    "MON_STAT"  },
    { 0x04,  1, OBD2_UNIT_PERCENT,   "Carga del motor",                   "ENG_LOAD"  },
    { 0x05,  1, OBD2_UNIT_CELSIUS,   "Temp. refrigerante",                "COOL_TMP"  },
    { 0x0A,  1, OBD2_UNIT_KPA,       "Presion combustible",               "FUEL_PRS"  },
    { 0x0B,  1, OBD2_UNIT_KPA,       "Presion abs. admision",             "MAP"       },
    { 0x0C,  2, OBD2_UNIT_RPM,       "RPM del motor",                     "RPM"       },
    { 0x0D,  1, OBD2_UNIT_KMH,       "Velocidad del vehiculo",            "SPEED"     },
    { 0x0F,  1, OBD2_UNIT_CELSIUS,   "Temp. aire admision",               "IAT"       },
    { 0x10,  2, OBD2_UNIT_GRAMS_SEC, "Flujo MAF",                         "MAF"       },
    { 0x11,  1, OBD2_UNIT_PERCENT,   "Posicion acelerador",               "THROTTLE"  },
    { 0x14,  2, OBD2_UNIT_VOLTS,     "Voltaje O2 B1S1",                   "O2_B1S1"   },
    { 0x1C,  1, OBD2_UNIT_NONE,      "Estandar OBD soportado",            "OBD_STD"   },
    { 0x1F,  2, OBD2_UNIT_SECONDS,   "Tiempo desde arranque",             "RUN_TIME"  },
    { 0x20,  4, OBD2_UNIT_NONE,      "PIDs soportados [21-40]",           "PIDS_B"    },
    { 0x2F,  1, OBD2_UNIT_PERCENT,   "Nivel de combustible",              "FUEL_LVL"  },
    { 0x33,  1, OBD2_UNIT_KPA,       "Presion barometrica",               "BARO"      },
    { 0x40,  4, OBD2_UNIT_NONE,      "PIDs soportados [41-60]",           "PIDS_C"    },
    { 0x42,  2, OBD2_UNIT_VOLTS,     "Voltaje modulo de control",         "ECU_VOLT"  },
    { 0x46,  1, OBD2_UNIT_CELSIUS,   "Temperatura ambiente",              "AMB_TEMP"  },
    { 0x51,  1, OBD2_UNIT_NONE,      "Tipo de combustible",               "FUEL_TYP"  },
    { 0x52,  1, OBD2_UNIT_PERCENT,   "Etanol en combustible",             "ETHANOL"   },
};

#define PID_CATALOG_SIZE    (sizeof(pid_catalog) / sizeof(pid_catalog[0]))

/** Ultimo Negative Response Code recibido: 0x7F [SID solicitado] [NRC]. */
static uint8_t obd2_last_nrc = 0U;

/* ========================================================================== */
/*  Funciones privadas                                                        */
/* ========================================================================== */

/**
 * @brief Construye y envia una trama OBD-II Single Frame
 *
 * Formato ISO 15765-4 Single Frame:
 *   [PCI] [SID] [PID] [extra] [pad] [pad] [pad] [pad]
 *
 * @param mode    Modo OBD-II (SID): 0x01, 0x02, 0x03, etc.
 * @param pid     PID a solicitar (ignorado si mode=0x03 o 0x04)
 * @param extra   Byte extra (freeze frame # para mode 02, 0 para otros)
 * @return CAN_OK si se transmitio exitosamente
 */
static CAN_Status_t OBD2_SendRequest(uint8_t mode, uint8_t pid, uint8_t extra)
{
    CAN_Message_t tx_msg;
    uint8_t pci;

    tx_msg.id = OBD2_TX_ID_BROADCAST;   /* 0x7DF = pregunta a todos los ECUs */
    tx_msg.dlc = 8;                      /* Siempre 8 bytes en ISO 15765-4 */

    /*
     * Determinar PCI (Protocol Control Information):
     * PCI = numero de bytes significativos que siguen.
     *
     *   Mode 01: PCI=2 -> [SID][PID]
     *   Mode 02: PCI=3 -> [SID][PID][Frame#]
     *   Mode 03: PCI=1 -> [SID]  (no lleva PID)
     *   Mode 04: PCI=1 -> [SID]  (no lleva PID)
     *   Mode 09: PCI=2 -> [SID][InfoType]
     */
    switch (mode)
    {
        case OBD2_MODE_CURRENT_DATA:    /* 0x01 */
        case OBD2_MODE_VEHICLE_INFO:    /* 0x09 */
            pci = 2;
            break;

        case OBD2_MODE_FREEZE_FRAME:    /* 0x02 */
            pci = 3;
            break;

        case OBD2_MODE_READ_DTC:        /* 0x03 */
        case OBD2_MODE_CLEAR_DTC:       /* 0x04 */
            pci = 1;
            break;

        default:
            pci = 2;
            break;
    }

    /* Armar trama */
    tx_msg.data[0] = pci;
    tx_msg.data[1] = mode;
    tx_msg.data[2] = pid;
    tx_msg.data[3] = extra;                 /* Frame # para mode 02, o padding */
    tx_msg.data[4] = OBD2_PADDING_BYTE;
    tx_msg.data[5] = OBD2_PADDING_BYTE;
    tx_msg.data[6] = OBD2_PADDING_BYTE;
    tx_msg.data[7] = OBD2_PADDING_BYTE;

    /* Para modes sin PID, asegurar padding desde byte 2 */
    if (mode == OBD2_MODE_READ_DTC || mode == OBD2_MODE_CLEAR_DTC)
    {
        tx_msg.data[2] = OBD2_PADDING_BYTE;
        tx_msg.data[3] = OBD2_PADDING_BYTE;
    }

    return CAN_Transmit(&tx_msg);
}

/**
 * @brief Recibe y valida una respuesta OBD-II
 *
 * Espera la respuesta del ECU y verifica que corresponda
 * al modo y PID solicitados.
 *
 * @param expected_mode  Modo que se espera en la respuesta
 * @param expected_pid   PID que se espera (ignorado para mode 03/04)
 * @param response       Puntero donde almacenar la respuesta parseada
 * @return OBD2_OK si la respuesta es valida
 */
static OBD2_Status_t OBD2_ReceiveResponse(uint8_t expected_mode,
                                            uint8_t expected_pid,
                                            OBD2_Response_t *response)
{
    CAN_Message_t rx_msg;
    CAN_Status_t  can_status;
    uint8_t response_sid;
    uint8_t pci_length;
    uint8_t i;

    /* Inicializar respuesta como invalida */
    response->valid = 0;

    /* Esperar respuesta CAN */
    can_status = CAN_Receive(&rx_msg, OBD2_RESPONSE_TIMEOUT);
    if (can_status != CAN_OK)
        return OBD2_ERR_TIMEOUT;

    /* ===== Parsear la respuesta ===== */

    /*
     * Formato respuesta Single Frame:
     *   rx_msg.data[0] = PCI (no. de bytes que siguen)
     *   rx_msg.data[1] = Response SID = Request SID + 0x40
     *   rx_msg.data[2] = PID confirmado
     *   rx_msg.data[3..] = Datos del PID
     *
     * Ejemplo (RPM = 3000):
     *   [04] [41] [0C] [2E] [E0] [55] [55] [55]
     *   PCI=4, SID=0x41, PID=0x0C, A=0x2E, B=0xE0
     *   RPM = (256*0x2E + 0xE0)/4 = (11744+224)/4 = 2992 ~ 3000
     */

    pci_length   = rx_msg.data[0];
    response_sid = rx_msg.data[1];

    /* Respuesta negativa UDS/OBD: 0x7F, SID original, NRC especifico */
    if (response_sid == 0x7FU)
    {
        if (rx_msg.dlc >= 4U && rx_msg.data[2] == expected_mode)
        {
            obd2_last_nrc = rx_msg.data[3];
            return OBD2_ERR_NEGATIVE_RESPONSE;
        }
        return OBD2_ERR_INVALID_RESPONSE;
    }

    /* Validar Response SID */
    if (response_sid != (expected_mode + OBD2_RESPONSE_OFFSET))
        return OBD2_ERR_INVALID_RESPONSE;

    /* Para modos con PID, verificar que el PID coincida */
    if (expected_mode == OBD2_MODE_CURRENT_DATA  ||
        expected_mode == OBD2_MODE_FREEZE_FRAME  ||
        expected_mode == OBD2_MODE_VEHICLE_INFO)
    {
        if (rx_msg.data[2] != expected_pid)
            return OBD2_ERR_INVALID_RESPONSE;

        response->pid  = rx_msg.data[2];
        response->mode = expected_mode;

        /*
         * Extraer bytes de datos:
         * pci_length incluye SID + PID + datos
         * -> datos = pci_length - 2 (SID y PID)
         */
        response->raw_bytes = (pci_length > 2) ? (pci_length - 2) : 0;

        if (response->raw_bytes > 4)
            response->raw_bytes = 4;  /* Max 4 bytes para Single Frame PIDs */

        for (i = 0; i < response->raw_bytes; i++)
        {
            response->raw[i] = rx_msg.data[3 + i];  /* Datos empiezan en byte 3 */
        }
    }
    else
    {
        /* Mode 03/04: no tienen PID en la respuesta */
        response->pid  = 0;
        response->mode = expected_mode;
        response->raw_bytes = (pci_length > 1) ? (pci_length - 1) : 0;

        if (response->raw_bytes > 4)
            response->raw_bytes = 4;

        for (i = 0; i < response->raw_bytes; i++)
        {
            response->raw[i] = rx_msg.data[2 + i];
        }
    }

    /* Decodificar valor a unidades de ingenieria */
    response->value = OBD2_DecodePID(response->pid, response->raw, response->raw_bytes);

    /* Buscar la unidad en el catalogo */
    {
        const OBD2_PID_Info_t *info = OBD2_GetPIDInfo(response->pid);
        response->unit = (info != NULL) ? info->unit : OBD2_UNIT_NONE;
    }

    response->valid = 1;
    return OBD2_OK;
}

/* ========================================================================== */
/*  Funciones publicas                                                        */
/* ========================================================================== */

OBD2_Status_t OBD2_RequestPID(uint8_t pid, OBD2_Response_t *response)
{
    CAN_Status_t  can_status;
    OBD2_Status_t status;
    uint8_t       attempt;

    if (response == NULL)
        return OBD2_ERR_PARAM;

    for (attempt = 0U; attempt < OBD2_PID_MAX_RETRIES; attempt++)
    {
        response->valid = 0U;
        obd2_last_nrc = 0U;

        /* Enviar request Mode 01 */
        can_status = OBD2_SendRequest(OBD2_MODE_CURRENT_DATA, pid, 0x00);
        if (can_status != CAN_OK)
        {
            status = OBD2_ERR_CAN_TX;
        }
        else
        {
            /* Recibir y decodificar respuesta */
            status = OBD2_ReceiveResponse(OBD2_MODE_CURRENT_DATA, pid, response);
        }

        if (status == OBD2_OK)
            return OBD2_OK;

        /* No reintentar si el ECU respondio explicitamente con NRC. */
        if (status == OBD2_ERR_NEGATIVE_RESPONSE || status == OBD2_ERR_INVALID_RESPONSE)
            return status;
    }

    return status;
}

OBD2_Status_t OBD2_RequestFreezeFrame(uint8_t pid, uint8_t frame,
                                       OBD2_Response_t *response)
{
    CAN_Status_t can_status;

    if (response == NULL)
        return OBD2_ERR_PARAM;

    /* Enviar request Mode 02 con frame number */
    can_status = OBD2_SendRequest(OBD2_MODE_FREEZE_FRAME, pid, frame);
    if (can_status != CAN_OK)
        return OBD2_ERR_CAN_TX;

    return OBD2_ReceiveResponse(OBD2_MODE_FREEZE_FRAME, pid, response);
}

OBD2_Status_t OBD2_ClearDTCs(void)
{
    CAN_Status_t can_status;
    OBD2_Response_t response;

    /* Enviar request Mode 04 (Clear DTCs) */
    can_status = OBD2_SendRequest(OBD2_MODE_CLEAR_DTC, 0x00, 0x00);
    if (can_status != CAN_OK)
        return OBD2_ERR_CAN_TX;

    /*
     * El ECU responde con SID 0x44 si acepto el borrado.
     * Si hay error, puede responder con NRC (Negative Response Code).
     */
    return OBD2_ReceiveResponse(OBD2_MODE_CLEAR_DTC, 0x00, &response);
}

OBD2_Status_t OBD2_GetSupportedPIDs(uint32_t supported_mask[3])
{
    OBD2_Response_t response;
    OBD2_Status_t status;
    uint8_t query_pids[3] = { 0x00, 0x20, 0x40 };
    uint8_t i;

    /* Inicializar mascaras en 0 */
    supported_mask[0] = 0;
    supported_mask[1] = 0;
    supported_mask[2] = 0;

    for (i = 0; i < 3; i++)
    {
        status = OBD2_RequestPID(query_pids[i], &response);

        if (status == OBD2_OK && response.valid && response.raw_bytes == 4)
        {
            /*
             * Los 4 bytes forman una bitmask de 32 PIDs.
             * Bit 31 (MSB de byte A) = PID base+1
             * Bit 0  (LSB de byte D) = PID base+32
             *
             * Ej: para PID 0x00, bit 31 = PID 0x01, bit 0 = PID 0x20
             */
            supported_mask[i] = ((uint32_t)response.raw[0] << 24)
                              | ((uint32_t)response.raw[1] << 16)
                              | ((uint32_t)response.raw[2] << 8)
                              | ((uint32_t)response.raw[3]);

            /*
             * Si el bit del PID "puente" (0x20 o 0x40) no esta seteado,
             * no tiene sentido seguir preguntando el siguiente rango.
             */
            if (i < 2)
            {
                /* Bit 0 de la mascara = PID base+0x20 (el siguiente "puente") */
                if (!(supported_mask[i] & 0x01))
                    break;  /* ECU no soporta el siguiente rango */
            }
        }
        else if (i == 0)
        {
            /* Si ni siquiera el primer query funciona, error */
            return (status != OBD2_OK) ? status : OBD2_ERR_INVALID_RESPONSE;
        }
        else
        {
            break;  /* Rangos adicionales son opcionales */
        }
    }

    return OBD2_OK;
}

uint8_t OBD2_IsPIDSupported(uint8_t pid, const uint32_t supported_mask[3])
{
    uint8_t group;
    uint8_t bit_pos;

    if (pid == 0x00 || pid > 0x60)
        return 0;

    /*
     * Determinar en que grupo cae el PID:
     *   PIDs 0x01-0x20 -> supported_mask[0], bit (0x20 - pid)
     *   PIDs 0x21-0x40 -> supported_mask[1], bit (0x40 - pid)
     *   PIDs 0x41-0x60 -> supported_mask[2], bit (0x60 - pid)
     */
    if (pid <= 0x20)
    {
        group   = 0;
        bit_pos = 0x20 - pid;
    }
    else if (pid <= 0x40)
    {
        group   = 1;
        bit_pos = 0x40 - pid;
    }
    else
    {
        group   = 2;
        bit_pos = 0x60 - pid;
    }

    return (supported_mask[group] & (1UL << bit_pos)) ? 1 : 0;
}

const OBD2_PID_Info_t *OBD2_GetPIDInfo(uint8_t pid)
{
    uint8_t i;

    for (i = 0; i < PID_CATALOG_SIZE; i++)
    {
        if (pid_catalog[i].pid == pid)
            return &pid_catalog[i];
    }

    return NULL;  /* PID no esta en el catalogo */
}


uint8_t OBD2_GetLastNRC(void)
{
    return obd2_last_nrc;
}

float OBD2_DecodePID(uint8_t pid, const uint8_t *raw_data, uint8_t num_bytes)
{
    uint8_t A, B;

    if (raw_data == NULL || num_bytes == 0)
        return 0.0f;

    A = raw_data[0];
    B = (num_bytes > 1) ? raw_data[1] : 0;

    /*
     * Formulas de conversion por PID (SAE J1979).
     *
     * Cada PID tiene su propia formula. Las mas comunes son:
     *   - Porcentaje:    A * 100 / 255
     *   - Temperatura:   A - 40
     *   - RPM:           (256*A + B) / 4
     *   - Velocidad:     A directamente (km/h)
     */
    switch (pid)
    {
        /* ===== Porcentajes (A * 100 / 255) ===== */
        case PID_ENGINE_LOAD:       /* 0x04 */
        case PID_THROTTLE_POS:      /* 0x11 */
        case PID_FUEL_LEVEL:        /* 0x2F */
        case PID_ETHANOL_PERCENT:   /* 0x52 */
            return (float)A * 100.0f / 255.0f;

        /* ===== Temperaturas (A - 40) ===== */
        case PID_COOLANT_TEMP:      /* 0x05 */
        case PID_INTAKE_AIR_TEMP:   /* 0x0F */
        case PID_AMBIENT_TEMP:      /* 0x46 */
            return (float)A - 40.0f;

        /* ===== RPM: (256*A + B) / 4 ===== */
        case PID_ENGINE_RPM:        /* 0x0C */
            return (float)(256U * A + B) / 4.0f;

        /* ===== Velocidad: A directamente (km/h) ===== */
        case PID_VEHICLE_SPEED:     /* 0x0D */
            return (float)A;

        /* ===== Presion combustible: A * 3 (kPa) ===== */
        case PID_FUEL_PRESSURE:     /* 0x0A */
            return (float)A * 3.0f;

        /* ===== Presion directa en kPa: A ===== */
        case PID_INTAKE_MAP:        /* 0x0B */
        case PID_BARO_PRESSURE:     /* 0x33 */
            return (float)A;

        /* ===== Flujo MAF: (256*A + B) / 100 (g/s) ===== */
        case PID_MAF_RATE:          /* 0x10 */
            return (float)(256U * A + B) / 100.0f;

        /* ===== Voltaje O2: A / 200 (V) ===== */
        case PID_O2_VOLTAGE_B1S1:   /* 0x14 */
            return (float)A / 200.0f;

        /* ===== Tiempo desde arranque: 256*A + B (seg) ===== */
        case PID_RUNTIME_SINCE_START:  /* 0x1F */
            return (float)(256U * A + B);

        /* ===== Voltaje modulo: (256*A + B) / 1000 (V) ===== */
        case PID_CONTROL_MODULE_VOLTAGE:  /* 0x42 */
            return (float)(256U * A + B) / 1000.0f;

        /* ===== PIDs de bitmask / lookup: retornar raw como entero ===== */
        case PID_SUPPORTED_PIDS_01_20:  /* 0x00 */
        case PID_SUPPORTED_PIDS_21_40:  /* 0x20 */
        case PID_SUPPORTED_PIDS_41_60:  /* 0x40 */
        case PID_MONITOR_STATUS:        /* 0x01 */
        case PID_OBD_STANDARD:          /* 0x1C */
        case PID_FUEL_TYPE:             /* 0x51 */
            return (float)A;  /* Valor raw, interpretar con lookup externo */

        /* ===== PID desconocido: retornar byte A ===== */
        default:
            return (float)A;
    }
}
