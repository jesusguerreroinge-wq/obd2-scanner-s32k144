/**
 * =============================================================================
 * main.c
 * Punto de entrada - Scanner Automotriz OBD-II
 * Target: S32K144EVB-Q100
 * =============================================================================
 *
 * Flujo principal:
 *   1. Inicializar sistema (clocks, pines)
 *   2. Inicializar FlexCAN0 (250 kbps, OBD-II)
 *   3. Inicializar LPUART1 (115200 baud -> ESP32)
 *   4. Consultar PIDs soportados por el ECU
 *   5. Loop principal:
 *      a. Recibir comando de la App via ESP32/UART
 *      b. Ejecutar el comando (leer PID, borrar DTCs, etc.)
 *      c. Enviar resultado formateado de vuelta a la App
 *
 * Protocolo con la App (sobre UART/ESP32):
 *   Request:  "PID:XX\n"       -> Solicitar PID 0xXX
 *             "SCAN\n"          -> Escanear todos los PIDs del dashboard
 *             "CLEAR\n" / "CLEAR_DTC\n" -> Borrar DTCs
 *             "DTC\n"           -> Leer DTCs almacenados
 *             "PENDING\n"       -> Leer DTCs pendientes
 *             "VIN\n"           -> Leer VIN
 *             "INFO\n"          -> Leer informacion basica
 *             "SUPPORTED\n"     -> Listar PIDs soportados
 *             "PING\n"          -> Test de conexion
 *   Response: "OK:PID:XX:valor:unidad\n"  -> Respuesta exitosa
 *             "ERR:codigo\n"              -> Error
 */

#include "sys_init.h"
#include "can_driver.h"
#include "obd2.h"
#include "obd2_extended.h"
#include "uart_driver.h"

/* ========================================================================== */
/*  Constantes                                                                */
/* ========================================================================== */

/** Buffer para comandos recibidos de la App */
#define CMD_BUFFER_SIZE     64

/** Buffer para respuestas hacia la App */
#define RSP_BUFFER_SIZE     128

/** PIDs a leer en modo SCAN (los mas comunes para un dashboard) */
static const uint8_t dashboard_pids[] = {
    PID_ENGINE_RPM,             /* 0x0C */
    PID_VEHICLE_SPEED,          /* 0x0D */
    PID_COOLANT_TEMP,           /* 0x05 */
    PID_ENGINE_LOAD,            /* 0x04 */
    PID_THROTTLE_POS,           /* 0x11 */
    PID_FUEL_LEVEL,             /* 0x2F */
    PID_INTAKE_AIR_TEMP,        /* 0x0F */
    PID_AMBIENT_TEMP,           /* 0x46 */
    PID_CONTROL_MODULE_VOLTAGE, /* 0x42 */
    PID_MAF_RATE,               /* 0x10 */
};

#define DASHBOARD_PID_COUNT  (sizeof(dashboard_pids) / sizeof(dashboard_pids[0]))

/* ========================================================================== */
/*  Variables globales                                                        */
/* ========================================================================== */

/** Mascaras de PIDs soportados por el ECU conectado */
static uint32_t ecu_supported_pids[3] = { 0, 0, 0 };

/** Flag: ya se consultaron los PIDs soportados */
static uint8_t pids_queried = 0;

/* ========================================================================== */
/*  Utilidades de string (bare-metal, sin stdlib)                             */
/* ========================================================================== */

/**
 * @brief Convierte un nibble (0-15) a caracter hex ASCII
 */
static char nibble_to_hex(uint8_t n)
{
    n &= 0x0F;
    return (n < 10) ? ('0' + n) : ('A' + (n - 10));
}

/**
 * @brief Convierte un byte a 2 caracteres hex en el buffer
 */
static void byte_to_hex(uint8_t val, char *buf)
{
    buf[0] = nibble_to_hex(val >> 4);
    buf[1] = nibble_to_hex(val & 0x0F);
}

/**
 * @brief Obtiene el string de unidad para enviar a la App
 */
static const char *unit_to_str(OBD2_Unit_t unit)
{
    switch (unit)
    {
        case OBD2_UNIT_PERCENT:    return "%";
        case OBD2_UNIT_CELSIUS:    return "C";
        case OBD2_UNIT_RPM:        return "RPM";
        case OBD2_UNIT_KMH:        return "km/h";
        case OBD2_UNIT_KPA:        return "kPa";
        case OBD2_UNIT_GRAMS_SEC:  return "g/s";
        case OBD2_UNIT_VOLTS:      return "V";
        case OBD2_UNIT_SECONDS:    return "s";
        case OBD2_UNIT_COUNT:      return "cnt";
        default:                   return "-";
    }
}

/**
 * @brief Convierte float a string simplificado (entero + 1 decimal)
 */
static uint8_t float_to_str(float val, char *buf, uint8_t buf_size)
{
    uint8_t pos = 0;
    int32_t integer_part;
    uint8_t decimal_part;
    uint8_t started = 0;
    int32_t divisor;

    if (buf_size < 8)
        return 0;

    if (val < 0.0f)
    {
        buf[pos++] = '-';
        val = -val;
    }

    /* Redondeo a 1 decimal: 2992.97 -> 2993.0 */
    val += 0.05f;
    integer_part = (int32_t)val;
    decimal_part = (uint8_t)((val - (float)integer_part) * 10.0f);

    if (integer_part == 0)
    {
        buf[pos++] = '0';
    }
    else
    {
        divisor = 1000000;
        while (divisor > integer_part && divisor > 1)
            divisor /= 10;

        while (divisor > 0)
        {
            uint8_t digit = (uint8_t)(integer_part / divisor);
            if (digit > 0 || started || divisor == 1)
            {
                buf[pos++] = '0' + digit;
                started = 1;
            }
            integer_part %= divisor;
            divisor /= 10;
        }
    }

    buf[pos++] = '.';
    buf[pos++] = '0' + (decimal_part % 10);
    buf[pos] = '\0';

    return pos;
}

/**
 * @brief Copia string (retorna puntero al final)
 */
static char *str_append(char *dst, const char *src)
{
    while (*src)
        *dst++ = *src++;
    return dst;
}

/**
 * @brief Compara dos strings (retorna 1 si iguales, 0 si no)
 */
static uint8_t str_equal(const char *a, const char *b)
{
    while (*a && *b)
    {
        if (*a != *b)
            return 0;
        a++;
        b++;
    }
    return (*a == *b) ? 1 : 0;
}

/**
 * @brief Convierte 2 caracteres hex ASCII a byte
 */
static uint8_t hex_to_byte(char hi, char lo)
{
    uint8_t val = 0;

    if (hi >= '0' && hi <= '9')       val = (hi - '0') << 4;
    else if (hi >= 'A' && hi <= 'F')  val = (hi - 'A' + 10) << 4;
    else if (hi >= 'a' && hi <= 'f')  val = (hi - 'a' + 10) << 4;
    else return 0xFF;

    if (lo >= '0' && lo <= '9')       val |= (lo - '0');
    else if (lo >= 'A' && lo <= 'F')  val |= (lo - 'A' + 10);
    else if (lo >= 'a' && lo <= 'f')  val |= (lo - 'a' + 10);
    else return 0xFF;

    return val;
}


static void send_status_error(const char *prefix, OBD2_Status_t status)
{
    char hex[3];

    UART_SendString(prefix);

    switch (status)
    {
        case OBD2_ERR_TIMEOUT:
            UART_SendString(":TIMEOUT\n");
            break;
        case OBD2_ERR_CAN_TX:
            UART_SendString(":CAN_TX_FAIL\n");
            break;
        case OBD2_ERR_NEGATIVE_RESPONSE:
            UART_SendString(":NRC:");
            byte_to_hex(OBD2_GetLastNRC(), hex);
            hex[2] = '\0';
            UART_SendString(hex);
            UART_SendString("\n");
            break;
        case OBD2_ERR_INVALID_RESPONSE:
            UART_SendString(":INVALID_RESPONSE\n");
            break;
        default:
            UART_SendString(":UNKNOWN\n");
            break;
    }
}

/* ========================================================================== */
/*  Funciones del scanner                                                     */
/* ========================================================================== */

static void scanner_query_supported(void)
{
    OBD2_Status_t status;

    status = OBD2_GetSupportedPIDs(ecu_supported_pids);

    if (status == OBD2_OK)
    {
        char hex_buf[12];
        uint8_t i;

        pids_queried = 1;
        UART_SendString("OK:SUPPORTED:");

        for (i = 0; i < 3; i++)
        {
            byte_to_hex((uint8_t)(ecu_supported_pids[i] >> 24), &hex_buf[0]);
            byte_to_hex((uint8_t)(ecu_supported_pids[i] >> 16), &hex_buf[2]);
            byte_to_hex((uint8_t)(ecu_supported_pids[i] >> 8),  &hex_buf[4]);
            byte_to_hex((uint8_t)(ecu_supported_pids[i]),        &hex_buf[6]);
            hex_buf[8] = '\0';
            UART_SendString(hex_buf);
            if (i < 2) UART_SendChar(':');
        }
        UART_SendString("\n");

        LED_GREEN_ON();
    }
    else
    {
        UART_SendString("ERR:NO_ECU\n");
        LED_RED_ON();
    }
}

static void scanner_read_pid(uint8_t pid)
{
    OBD2_Response_t response;
    OBD2_Status_t   status;
    char rsp_buf[RSP_BUFFER_SIZE];
    char *p;
    char hex[3];
    char val_str[16];

    if (pids_queried && !OBD2_IsPIDSupported(pid, ecu_supported_pids))
    {
        UART_SendString("ERR:PID_NOT_SUPPORTED\n");
        return;
    }

    LED_BLUE_ON();
    status = OBD2_RequestPID(pid, &response);
    LED_BLUE_OFF();

    if (status == OBD2_OK && response.valid)
    {
        p = rsp_buf;
        p = str_append(p, "OK:PID:");

        byte_to_hex(pid, hex);
        hex[2] = '\0';
        p = str_append(p, hex);
        p = str_append(p, ":");

        float_to_str(response.value, val_str, sizeof(val_str));
        p = str_append(p, val_str);
        p = str_append(p, ":");

        p = str_append(p, unit_to_str(response.unit));
        p = str_append(p, "\n");
        *p = '\0';

        UART_SendString(rsp_buf);
        LED_GREEN_ON();
    }
    else
    {
        p = rsp_buf;
        p = str_append(p, "ERR:PID:");
        byte_to_hex(pid, hex);
        hex[2] = '\0';
        p = str_append(p, hex);
        *p = '\0';

        UART_SendString(rsp_buf);

        switch (status)
        {
            case OBD2_ERR_TIMEOUT:
                UART_SendString(":TIMEOUT\n");
                break;
            case OBD2_ERR_CAN_TX:
                UART_SendString(":CAN_TX_FAIL\n");
                break;
            case OBD2_ERR_NEGATIVE_RESPONSE:
                UART_SendString(":NRC:");
                byte_to_hex(OBD2_GetLastNRC(), hex);
                hex[2] = '\0';
                UART_SendString(hex);
                UART_SendString("\n");
                break;
            case OBD2_ERR_INVALID_RESPONSE:
                UART_SendString(":INVALID_RESPONSE\n");
                break;
            default:
                UART_SendString(":UNKNOWN\n");
                break;
        }

        LED_RED_ON();
    }
}

static void scanner_full_scan(void)
{
    uint8_t i;

    UART_SendString("OK:SCAN:START\n");

    for (i = 0; i < DASHBOARD_PID_COUNT; i++)
    {
        if (!pids_queried || OBD2_IsPIDSupported(dashboard_pids[i], ecu_supported_pids))
        {
            scanner_read_pid(dashboard_pids[i]);
        }
    }

    UART_SendString("OK:SCAN:END\n");
}


static void scanner_read_vin(void)
{
    OBD2_VIN_t vin;
    OBD2_Status_t status;

    LED_BLUE_ON();
    status = OBD2_ReadVIN(&vin);
    LED_BLUE_OFF();

    if (status == OBD2_OK && vin.valid)
    {
        UART_SendString("OK:VIN:");
        UART_SendString(vin.vin);
        UART_SendString("\n");
        LED_GREEN_ON();
    }
    else
    {
        send_status_error("ERR:VIN", status);
        LED_RED_ON();
    }
}

static void scanner_send_dtc_list(const OBD2_DTC_List_t *list, const char *prefix)
{
    uint8_t i;

    if (list == (void *)0 || !list->valid || list->count == 0U)
    {
        UART_SendString(prefix);
        UART_SendString(":NONE\n");
        return;
    }

    UART_SendString(prefix);
    UART_SendString(":");

    for (i = 0U; i < list->count; i++)
    {
        UART_SendString(list->dtcs[i].code);
        UART_SendString(",STORED");
        if (i < (uint8_t)(list->count - 1U))
            UART_SendString(";");
    }

    UART_SendString("\n");
}

static void scanner_read_dtcs(uint8_t pending)
{
    OBD2_DTC_List_t list;
    OBD2_Status_t status;

    LED_BLUE_ON();
    status = pending ? OBD2_ReadPendingDTCs(&list) : OBD2_ReadDTCs(&list);
    LED_BLUE_OFF();

    if (status == OBD2_OK && list.valid)
    {
        scanner_send_dtc_list(&list, pending ? "PENDING" : "DTC");
        LED_GREEN_ON();
    }
    else
    {
        send_status_error(pending ? "ERR:PENDING" : "ERR:DTC", status);
        LED_RED_ON();
    }
}

static void scanner_info(void)
{
    scanner_read_vin();
    scanner_query_supported();
}

static void process_command(const char *cmd)
{
    /* Ignorar lineas vacias o punteros invalidos. */
    if (cmd == (void *)0 || cmd[0] == '\0')
    {
        return;
    }

    if (str_equal(cmd, "PING"))
    {
        UART_SendString("OK:PONG\n");
        return;
    }

    if (str_equal(cmd, "SUPPORTED"))
    {
        scanner_query_supported();
        return;
    }

    if (str_equal(cmd, "SCAN"))
    {
        scanner_full_scan();
        return;
    }

    if (str_equal(cmd, "VIN"))
    {
        scanner_read_vin();
        return;
    }

    if (str_equal(cmd, "DTC"))
    {
        scanner_read_dtcs(0U);
        return;
    }

    if (str_equal(cmd, "PENDING"))
    {
        scanner_read_dtcs(1U);
        return;
    }

    if (str_equal(cmd, "INFO"))
    {
        scanner_info();
        return;
    }

    if (str_equal(cmd, "CLEAR") || str_equal(cmd, "CLEAR_DTC"))
    {
        OBD2_Status_t status = OBD2_ClearDTCs();
        if (status == OBD2_OK)
            UART_SendString("OK:DTC_CLEARED\n");
        else
            send_status_error("ERR:CLEAR_FAILED", status);
        return;
    }

    if (cmd[0] == 'P' && cmd[1] == 'I' && cmd[2] == 'D' && cmd[3] == ':')
    {
        uint8_t pid = hex_to_byte(cmd[4], cmd[5]);
        if (pid != 0xFF)
            scanner_read_pid(pid);
        else
            UART_SendString("ERR:BAD_PID\n");
        return;
    }

    /* Ignorar basura UART / comandos no reconocidos */
    return;
}

/* ========================================================================== */
/*  Delay simple (sin timer)                                                  */
/* ========================================================================== */

static void delay_ms(uint32_t ms)
{
    volatile uint32_t count;
    for (; ms > 0; ms--)
        for (count = 0; count < 8000UL; count++);
}

/* ========================================================================== */
/*  Main                                                                      */
/* ========================================================================== */

int main(void)
{
    char cmd_buffer[CMD_BUFFER_SIZE];
    uint8_t cmd_len;

    /* ===== Paso 1: Inicializar hardware ===== */
    SysInit();
    SBC_Init();      /* Despierta UJA1169 y habilita transceptor CAN del EVB */
    CAN_Init();
    UART_Init();

    /* ===== Paso 2: Senal visual de arranque ===== */
    LED_RED_ON();    delay_ms(200);  LED_RED_OFF();
    LED_GREEN_ON();  delay_ms(200);  LED_GREEN_OFF();
    LED_BLUE_ON();   delay_ms(200);  LED_BLUE_OFF();

    /* Mensaje de bienvenida */
    UART_SendString("OBD2-SCANNER:READY\n");

    /* ===== Paso 3: Intentar conectar con el ECU ===== */
    scanner_query_supported();

    /* ===== Paso 4: Loop principal (comando-respuesta) ===== */
    while (1)
    {
        cmd_len = UART_ReadLine(cmd_buffer, CMD_BUFFER_SIZE);

        if (cmd_len > 0)
        {
            LED_GREEN_TOGGLE();
            process_command(cmd_buffer);
        }
    }

    return 0;
}
