#include "flight_state.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug_macro.h"
#include <math.h>
#include <stdbool.h>

// Constantes sintonizaveis
#define FS_TASK_PERIOD_MS     100u
#define FS_MUTEX_TIMEOUT_MS   50u

#define FS_CAL_SAMPLES        20u  // ~2 s de media na plataforma
#define FS_ALT_EMA_ALPHA      0.30 // low-pass da altitude
#define FS_VZ_EMA_ALPHA       0.30 // low-pass da velocidade vertical
#define FS_CONFIRM_SAMPLES    3u   // amostras consecutivas p/ confirmar transicao

// Atmosfera padrao
#define FS_LAPSE_RATE_K_M     0.0065 // gradiente termico da troposfera [K/m]
#define FS_PRESSURE_EXP       0.1903 // R*L/(g*M), inverso do expoente barometrico
#define FS_ISA_T0_K           288.15 // 15 C, fallback quando nao ha temperatura
#define FS_TEMP_MIN_C         (-40.0) // faixa plausivel para o sensor no solo
#define FS_TEMP_MAX_C         60.0

#define FS_LIFTOFF_ALT_M      15.0
#define FS_LIFTOFF_VZ_MS      5.0
#define FS_BURNOUT_VZ_DROP_MS 2.0
#define FS_APOGEE_VZ_MS       (-2.0)
#define FS_LANDED_VZ_MS       1.0
#define FS_LANDED_ALT_M       20.0
#define FS_LANDED_HOLD_MS     3000u

// Estado interno
static FlightStage_t s_stage = FLIGHT_STAGE_LAUNCH;

static double s_ground_pressure_pa = 0.0;
static double s_ground_temp_k = FS_ISA_T0_K;
static double s_cal_sum = 0.0;
static double s_cal_temp_sum = 0.0;
static uint32_t s_cal_count = 0u;

static double s_alt_m = 0.0;
static double s_vz_ms = 0.0;
static int s_alt_initialized = 0;

static double s_apogee_alt_m = 0.0;
static uint32_t s_apogee_tick = 0u;

static double s_peak_vz_ms = 0.0;
static uint32_t s_confirm = 0u;
static uint32_t s_landed_since_tick = 0u;
static int s_landed_pending = 0;
static uint32_t inicio_simulacao = 0;
static double s_last_pressure = 0.0;
static uint32_t s_last_tick = 0u;

typedef struct
{
    double temperatura;
    double pressao;
    double altitude;
    double velocidade;
} BMEstruct;

BMEstruct simularFoguete(double tempo)
{
    BMEstruct dados;

    const double P0 = 1000.0;   // pressão no chão
    const double T0 = 25.0;

    const double APOGEU_T   = 20.0;
    const double APOGEU_ALT = 1000.0;

    double altitude;

    // =========================
    // SUBIDA
    // =========================
    if (tempo < APOGEU_T)
    {
        altitude =
            100.0 * tempo
            - 2.5 * tempo * tempo;
    }

    // =========================
    // DESCIDA
    // =========================
    else
    {
        double t = tempo - APOGEU_T;

        // Paraquedas abrindo
        if (t < 2.0)
        {
            altitude = 1000.0 - 10.0 * t;
        }
        else
        {
            // Descida a 8 m/s
            altitude = 980.0 - 8.0 * (t - 2.0);
        }
    }

    // =========================
    // CHÃO
    // =========================
    if (altitude <= 0.0)
    {
        dados.altitude = 0.0;
        dados.pressao = P0;      // 1000 hPa
        dados.temperatura = T0;

        return dados;
    }

    // =========================
    // PRESSÃO
    // =========================

    dados.altitude = altitude;

    dados.pressao =
        P0 * exp(-altitude / 8500.0);

    // =========================
    // TEMPERATURA
    // =========================

    dados.temperatura =
        T0 - 0.0065 * altitude;

    return dados;
}

// Conversao pressao -> altitude
/**
 * Temperatura de referencia do solo, em kelvin. Leitura fora da faixa plausivel
 * (sensor mudo, I2C travado, DTO ainda zerado) cai para a ISA, o que reproduz
 * exatamente o coeficiente 44330 usado antes da correcao de temperatura.
 */
static double temperature_reference_k(double temp_c)
{
    if (temp_c < FS_TEMP_MIN_C || temp_c > FS_TEMP_MAX_C)
    {
        DEBUG_PRINT("[FSM] temperatura de solo invalida (%.1f C), usando ISA\r\n", temp_c);
        return FS_ISA_T0_K;
    }
    return temp_c + 273.15;
}

/**
 * Formula hipsometrica da atmosfera padrao. Como a referencia usada e a
 * pressao medida no solo, o resultado ja e altitude AGL: le ~0 m na plataforma, 
 * qualquer que seja o clima do dia.
 * 
 * RESUMINDO: converte a pressão medida em altitude acima do ponto de lançamento, usando a fórmula hipsométrica da troposfera padrão
 */
static double pressure_to_altitude(double pressure_pa, double ground_pa, double ground_temp_k)
{
    return (ground_temp_k / FS_LAPSE_RATE_K_M) * (1.0 - pow(pressure_pa / ground_pa, FS_PRESSURE_EXP));
}

const char *FlightState_Name(FlightStage_t stage) 
{
    switch (stage) 
    {
    case FLIGHT_STAGE_LAUNCH:
        return "LAUNCH";
    case FLIGHT_STAGE_POWERED_ASCENT:
        return "POWERED_ASCENT";
    case FLIGHT_STAGE_COASTING:
        return "COASTING";
    case FLIGHT_STAGE_DESCENT:
        return "DESCENT";
    case FLIGHT_STAGE_RECOVERY:
        return "RECOVERY";
    default:
        return "UNKNOWN";
    }
}

static void flight_state_advance(FlightStage_t next) 
{
    const FlightStage_t prev = s_stage;
    (void)prev;

    s_stage = next;
    s_confirm = 0u;

    DEBUG_PRINT("[FSM] %s -> %s | alt=%.1f m vz=%.1f m/s\r\n", FlightState_Name(prev), FlightState_Name(next), s_alt_m, s_vz_ms);
}

// Nucleo da maquina de estados
static void flight_state_update(double pressure_pa, double temp_c, uint32_t now)
{
    // 1) Calibracao da referencia de solo, ainda na plataforma. Pressao E
    //    temperatura: as duas definem a atmosfera do dia. A temperatura fica
    //    congelada aqui de proposito -- ela e a base da coluna de ar, medida no
    //    solo. Usar a leitura instantanea em voo seria errado duas vezes: o ar
    //    esfria com a altura (e isso ja esta no modelo, via L) e o BME280 dentro
    //    da coifa mede a caixa, nao o ar externo.
    if (s_cal_count < FS_CAL_SAMPLES)
    {
        s_cal_sum += pressure_pa;
        s_cal_temp_sum += temp_c;
        s_cal_count++;
        if (s_cal_count == FS_CAL_SAMPLES)
        {
            s_ground_pressure_pa = s_cal_sum / (double)FS_CAL_SAMPLES;
            s_ground_temp_k = temperature_reference_k(s_cal_temp_sum / (double)FS_CAL_SAMPLES);
            s_last_pressure = pressure_pa;
            s_last_tick = now;
            DEBUG_PRINT("[FSM] referencia de solo = %.1f Pa, %.1f K (escala %.0f k/m)\r\n", s_ground_pressure_pa, s_ground_temp_k, s_ground_temp_k / FS_LAPSE_RATE_K_M);
        }
        return; // sem referencia nao ha altitude, logo nao ha transicao
    }

    if (s_ground_pressure_pa <= 0.0)
        return;

    // 2) A task BME280 publica a 10 Hz e esta roda a 10 Hz; a deriva de fase
    //    faz algumas iteracoes lerem a MESMA pressao. Derivar nesse caso daria
    //    um vz = 0 espurio. Sai sem tocar em s_last_tick, para que o dt acumule
    //    ate a proxima amostra fresca.
    if (pressure_pa == s_last_pressure)
        return;

    // 3) dt medido dos ticks reais (nao assumido igual ao periodo), para que a
    //    velocidade continue correta mesmo com iteracoes puladas. A subtracao
    //    em uint32_t trata o wraparound do tick corretamente.
    const double dt = (double)(now - s_last_tick) / (double)configTICK_RATE_HZ;
    s_last_pressure = pressure_pa;
    s_last_tick = now;
    if (dt <= 0.0)
        return;

    const double alt_raw = pressure_to_altitude(pressure_pa, s_ground_pressure_pa, s_ground_temp_k);

    // 4) Primeira amostra valida: so semeia o filtro, ainda nao ha derivada.
    if (!s_alt_initialized) 
    {
        taskENTER_CRITICAL();
        s_alt_m = alt_raw;
        taskEXIT_CRITICAL();
        s_alt_initialized = 1;
        return;
    }

    // 5) EMA na altitude, derivada, EMA na velocidade. A dupla suavizacao
    //    absorve o ruido do barometro sem atrasar demais a deteccao do apogeu.
    const double alt_prev = s_alt_m;
    const double alt_new = FS_ALT_EMA_ALPHA * alt_raw + (1.0 - FS_ALT_EMA_ALPHA) * alt_prev;
    const double vz_raw = (alt_new - alt_prev) / dt;
    const double vz_new = FS_VZ_EMA_ALPHA * vz_raw + (1.0 - FS_VZ_EMA_ALPHA) * s_vz_ms;

    const int track_apogee = (s_stage == FLIGHT_STAGE_POWERED_ASCENT || s_stage == FLIGHT_STAGE_COASTING) && (alt_new > s_apogee_alt_m);

    // Publicacao atomica: doubles tem 8 bytes e nao sao lidos/escritos em uma
    // instrucao no Cortex-M4, entao escrita e leitura usam secao critica.
    taskENTER_CRITICAL();
    s_alt_m = alt_new;
    s_vz_ms = vz_new;
    if (track_apogee) 
    {
        s_apogee_alt_m = alt_new;
        s_apogee_tick = now;
    }
    taskEXIT_CRITICAL();

    // 6) Avaliacao da transicao do estado atual.
    switch (s_stage) 
    {
    case FLIGHT_STAGE_LAUNCH:
        // Decolagem: precisa de altitude E velocidade, para que rajada de vento
        // ou variacao de pressao ambiente sozinhas nao disparem a maquina.
        if (s_alt_m > FS_LIFTOFF_ALT_M && s_vz_ms > FS_LIFTOFF_VZ_MS) 
        {
            if (++s_confirm >= FS_CONFIRM_SAMPLES)
            {
                s_peak_vz_ms = s_vz_ms;
                flight_state_advance(FLIGHT_STAGE_POWERED_ASCENT);
            }
        } else 
        {
            s_confirm = 0u;
        }
        break;

    case FLIGHT_STAGE_POWERED_ASCENT:
        // Burnout: a velocidade para de crescer e cai de forma sustentada em
        // relacao ao pico. Detector grosseiro -- o barometro e ruidoso sob
        // empuxo -- mas suficiente para separar as fases.
        if (s_vz_ms > s_peak_vz_ms) 
        {
            s_peak_vz_ms = s_vz_ms;
        }
        if (s_vz_ms < (s_peak_vz_ms - FS_BURNOUT_VZ_DROP_MS)) 
        {
            if (++s_confirm >= FS_CONFIRM_SAMPLES) 
            {
                flight_state_advance(FLIGHT_STAGE_COASTING);
            }
        } else 
        {
            s_confirm = 0u;
        }
        break;

    case FLIGHT_STAGE_COASTING:
        // Apogeu cruzado: a velocidade vertical trocou de sinal. A altitude
        // maxima ja ficou registrada em s_apogee_alt_m.
        if (s_vz_ms < FS_APOGEE_VZ_MS) 
        {
            if (++s_confirm >= FS_CONFIRM_SAMPLES) 
            {
                flight_state_advance(FLIGHT_STAGE_DESCENT);
            }
        } else 
        {
            s_confirm = 0u;
        }
        break;

    case FLIGHT_STAGE_DESCENT:
        // Pouso: repouso perto do solo sustentado por FS_LANDED_HOLD_MS. Usa
        // marca de tick em vez de contagem de amostras porque o intervalo aqui
        // e um tempo real exigido, nao um numero de leituras.
        if (fabs(s_vz_ms) < FS_LANDED_VZ_MS && s_alt_m < FS_LANDED_ALT_M) 
        {
            if (!s_landed_pending) 
            {
                s_landed_pending = 1;
                s_landed_since_tick = now;
            } else if ((now - s_landed_since_tick) >= pdMS_TO_TICKS(FS_LANDED_HOLD_MS)) 
            {
                flight_state_advance(FLIGHT_STAGE_RECOVERY);
                DEBUG_PRINT("FIM DO TESTE");
            }
        } else 
        {
            s_landed_pending = 0;
        }
        break;

    case FLIGHT_STAGE_RECOVERY:
    default:
        break; // terminal
    }
}

// Interface publica
FlightStage_t FlightState_Get(void) 
{
    return s_stage;
}

double FlightState_GetAltitude(void)
{
    double value;
    taskENTER_CRITICAL();
    value = s_alt_m;
    taskEXIT_CRITICAL();
    return value;
}

double FlightState_GetVerticalSpeed(void)
{
    double value;
    taskENTER_CRITICAL();
    value = s_vz_ms;
    taskEXIT_CRITICAL();
    return value;
}

double FlightState_GetApogeeAltitude(void)
{
    double value;
    taskENTER_CRITICAL();
    value = s_apogee_alt_m;
    taskEXIT_CRITICAL();
    return value;
}

uint32_t FlightState_GetApogeeTick(void)
{
    return s_apogee_tick;
}

void task_flight_state(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    double pressure = 1000.0;
    double temperature = 22.0;
    int tempo = 0;
    while (1)
    {
        int got = 0;

        // As duas leituras saem do mesmo lock: precisam vir da mesma amostra do
        // BME280 para descrever a mesma atmosfera.
        if (osMutexAcquire(lapesp_mutex, osWaitForever) == osOK)
        {
           pressure = lapesp_dto.BME_pressao;
            temperature = lapesp_dto.BME_temperatura;
            osMutexRelease(lapesp_mutex);
            got = 1;
        }

        if (got && pressure > 0.0)
        {
            flight_state_update(
                pressure,
                temperature,
                (uint32_t)xTaskGetTickCount()
            );

        }

        osDelay(100);

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(FS_TASK_PERIOD_MS));
    }
}
