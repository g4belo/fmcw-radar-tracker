#include "app_main.h"
#include "main.h"
#include "radar_engine.h"
#include "target_simulator.h"
#include "telemetry.h"
#include "arm_const_structs.h"
#include "arm_math.h"

// 1: Executa simulação balística em malha fechada (HIL)
// 0: Coleta sinais reais dos 4 canais analógicos de RF via ADCs físicos
#define SIMULATE_DYNAMIC_TARGET 1
#define ENABLE_USB_TELEMETRY    1

#define FFT_SIZE 512
#define NUM_CHIRPS 16
#define RANGE_BINS (FFT_SIZE / 2)
#define DMA_BUFFER_SIZE (FFT_SIZE * 2)

// --- Constantes de RF e Cinemática FMCW ---
constexpr float32_t SPEED_OF_LIGHT = 299792458.0f;
constexpr float32_t SAMPLING_FREQ  = 1000000.0f;
constexpr float32_t TX_FREQ_CENTER = 4.2e9f;
constexpr float32_t CHIRP_BW       = 200.0e6f;
constexpr float32_t CHIRP_TIME     = (float32_t)FFT_SIZE / SAMPLING_FREQ;
constexpr float32_t WAVELENGTH     = SPEED_OF_LIGHT / TX_FREQ_CENTER;
constexpr float32_t PI_F           = 3.141592653589793f;
constexpr float32_t RAD_TO_DEG     = 180.0f / PI_F;
constexpr float32_t DEG_TO_RAD     = PI_F / 180.0f;

constexpr float32_t FREQ_BIN_RES   = SAMPLING_FREQ / (float32_t)FFT_SIZE;
constexpr float32_t RANGE_RES_M    = SPEED_OF_LIGHT / (2.0f * CHIRP_BW);
constexpr float32_t SLOW_FREQ_RES  = (1.0f / CHIRP_TIME) / (float32_t)NUM_CHIRPS;
constexpr float32_t VELOCITY_RES   = (WAVELENGTH * 0.5f) * SLOW_FREQ_RES;

// --- Mapeamento de Hardware da Torreta Pan-Tilt e Efetor Laser ---
#define PAN_STEP_PIN    GPIO_PIN_0
#define PAN_STEP_PORT   GPIOB
#define PAN_DIR_PIN     GPIO_PIN_1
#define PAN_DIR_PORT    GPIOB

#define TILT_STEP_PIN   GPIO_PIN_2
#define TILT_STEP_PORT  GPIOB
#define TILT_DIR_PIN    GPIO_PIN_10
#define TILT_DIR_PORT   GPIOB

#define LASER_PIN       GPIO_PIN_12
#define LASER_PORT      GPIOB

constexpr float32_t DEG_PER_MICROSTEP = 0.1125f;

// Instâncias dos 4 ADCs e do Timer geradas pelo CubeMX
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;


// --- Filtro de Kalman Estendido 6D (EKF) ---
class ExtendedKalmanFilter6D {
public:
    float32_t x[6];
    float32_t P[6][6];
    float32_t Q[6][6];
    float32_t R[4][4];
    float32_t gamma;
    float32_t g;
    bool initialized;

    ExtendedKalmanFilter6D() {
        gamma = 0.025f;
        g = 9.81f;
        initialized = false;
        reset();
    }

    void reset() {
        for (int i = 0; i < 6; i++) {
            x[i] = 0.0f;
            for (int j = 0; j < 6; j++) {
                P[i][j] = (i == j) ? 2.0f : 0.0f;
                Q[i][j] = 0.0f;
            }
        }
        Q[0][0] = Q[1][1] = Q[2][2] = 0.02f;
        Q[3][3] = Q[4][4] = Q[5][5] = 0.5f;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) R[i][j] = 0.0f;
        }
        R[0][0] = 0.25f;
        R[1][1] = 0.0004f;
        R[2][2] = 0.0004f;
        R[3][3] = 1.0f;

        initialized = false;
    }

    void init_state(float32_t r, float32_t theta_rad, float32_t phi_rad, float32_t vr) {
        float32_t cos_phi = cosf(phi_rad);
        x[0] = r * cos_phi * cosf(theta_rad);
        x[1] = r * cos_phi * sinf(theta_rad);
        x[2] = r * sinf(phi_rad);
        x[3] = vr * cos_phi * cosf(theta_rad);
        x[4] = vr * cos_phi * sinf(theta_rad);
        x[5] = vr * sinf(phi_rad);
        initialized = true;
    }

    void predict(float32_t dt) {
        if (!initialized) return;

        float32_t v_norm = sqrtf(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]);
        if (v_norm < 0.001f) v_norm = 0.001f;

        float32_t ax = -gamma * v_norm * x[3];
        float32_t ay = -gamma * v_norm * x[4];
        float32_t az = -g - (gamma * v_norm * x[5]);

        x[0] += x[3] * dt;
        x[1] += x[4] * dt;
        x[2] += x[5] * dt;
        x[3] += ax * dt;
        x[4] += ay * dt;
        x[5] += az * dt;

        float32_t inv_v_sq = 1.0f / (v_norm * v_norm);
        float32_t alpha_x = gamma * v_norm * (1.0f + x[3] * x[3] * inv_v_sq) * dt;
        float32_t alpha_y = gamma * v_norm * (1.0f + x[4] * x[4] * inv_v_sq) * dt;
        float32_t alpha_z = gamma * v_norm * (1.0f + x[5] * x[5] * inv_v_sq) * dt;

        float32_t F[6][6] = {0};
        for (int i = 0; i < 6; i++) F[i][i] = 1.0f;
        F[0][3] = dt; F[1][4] = dt; F[2][5] = dt;
        F[3][3] = 1.0f - alpha_x;
        F[4][4] = 1.0f - alpha_y;
        F[5][5] = 1.0f - alpha_z;

        float32_t FP[6][6] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 6; k++) {
                    FP[i][j] += F[i][k] * P[k][j];
                }
            }
        }

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float32_t sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += FP[i][k] * F[j][k];
                }
                P[i][j] = sum + Q[i][j];
            }
        }
    }

    void update(float32_t z_r, float32_t z_theta, float32_t z_phi, float32_t z_vr) {
        if (!initialized) {
            init_state(z_r, z_theta, z_phi, z_vr);
            return;
        }

        float32_t r_xy_sq = x[0] * x[0] + x[1] * x[1];
        if (r_xy_sq < 0.001f) r_xy_sq = 0.001f;
        float32_t r_xy = sqrtf(r_xy_sq);
        float32_t r_sq = r_xy_sq + x[2] * x[2];
        float32_t r = sqrtf(r_sq);
        if (r < 0.1f) return;

        float32_t h[4];
        h[0] = r;
        h[1] = atan2f(x[1], x[0]);
        h[2] = atan2f(x[2], r_xy);
        h[3] = (x[0] * x[3] + x[1] * x[4] + x[2] * x[5]) / r;

        float32_t y[4];
        y[0] = z_r - h[0];
        float32_t d_th = z_theta - h[1];
        y[1] = atan2f(sinf(d_th), cosf(d_th));
        float32_t d_ph = z_phi - h[2];
        y[2] = atan2f(sinf(d_ph), cosf(d_ph));
        y[3] = z_vr - h[3];

        float32_t H[4][6] = {0};
        H[0][0] = x[0] / r;
        H[0][1] = x[1] / r;
        H[0][2] = x[2] / r;
        H[1][0] = -x[1] / r_xy_sq;
        H[1][1] =  x[0] / r_xy_sq;
        H[2][0] = -x[0] * x[2] / (r_sq * r_xy);
        H[2][1] = -x[1] * x[2] / (r_sq * r_xy);
        H[2][2] = r_xy / r_sq;

        float32_t vr_pred = h[3];
        H[3][0] = (x[3] / r) - (x[0] * vr_pred / r_sq);
        H[3][1] = (x[4] / r) - (x[1] * vr_pred / r_sq);
        H[3][2] = (x[5] / r) - (x[2] * vr_pred / r_sq);
        H[3][3] = x[0] / r;
        H[3][4] = x[1] / r;
        H[3][5] = x[2] / r;

        float32_t HP[4][6] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 6; k++) {
                    HP[i][j] += H[i][k] * P[k][j];
                }
            }
        }

        float32_t S[4][4] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float32_t sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    sum += HP[i][k] * H[j][k];
                }
                S[i][j] = sum + R[i][j];
            }
        }

        float32_t S_inv[4][4] = {0};
        float32_t A[4][8];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) A[i][j] = S[i][j];
            for (int j = 4; j < 8; j++) A[i][j] = (j - 4 == i) ? 1.0f : 0.0f;
        }

        for (int i = 0; i < 4; i++) {
            int max_r = i;
            for (int k = i + 1; k < 4; k++) {
                if (fabsf(A[k][i]) > fabsf(A[max_r][i])) max_r = k;
            }
            if (max_r != i) {
                for (int k = 0; k < 8; k++) {
                    float32_t tmp = A[i][k];
                    A[i][k] = A[max_r][k];
                    A[max_r][k] = tmp;
                }
            }

            float32_t pivot = A[i][i];
            if (fabsf(pivot) < 1.0e-7f) return;
            float32_t inv_pivot = 1.0f / pivot;
            for (int k = 0; k < 8; k++) A[i][k] *= inv_pivot;

            for (int k = 0; k < 4; k++) {
                if (k != i) {
                    float32_t factor = A[k][i];
                    for (int l = 0; l < 8; l++) {
                        A[k][l] -= factor * A[i][l];
                    }
                }
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                S_inv[i][j] = A[i][j + 4];
            }
        }

        float32_t PHT[6][4] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 6; k++) {
                    PHT[i][j] += P[i][k] * H[j][k];
                }
            }
        }

        float32_t K[6][4] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    K[i][j] += PHT[i][k] * S_inv[k][j];
                }
            }
        }

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                x[i] += K[i][j] * y[j];
            }
        }

        float32_t KH[6][6] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 4; k++) {
                    KH[i][j] += K[i][k] * H[k][j];
                }
            }
        }

        float32_t P_new[6][6] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float32_t sum = 0.0f;
                for (int k = 0; k < 6; k++) {
                    float32_t I_minus_KH_ik = ((i == k) ? 1.0f : 0.0f) - KH[i][k];
                    sum += I_minus_KH_ik * P[k][j];
                }
                P_new[i][j] = sum;
            }
        }

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float32_t p_sym = 0.5f * (P_new[i][j] + P_new[j][i]);
                if (i == j && p_sym < 1.0e-5f) p_sym = 1.0e-5f;
                P[i][j] = p_sym;
            }
        }
    }
};

// --- Atuador Pan-Tilt e Controle Óptico ---
class TurretActuator {
public:
    float32_t current_pan_deg;
    float32_t current_tilt_deg;
    int32_t total_pan_steps;
    int32_t total_tilt_steps;
    bool laser_active;

    void init() {
        current_pan_deg = 0.0f;
        current_tilt_deg = 0.0f;
        total_pan_steps = 0;
        total_tilt_steps = 0;
        laser_active = false;

        HAL_GPIO_WritePin(PAN_STEP_PORT, PAN_STEP_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PAN_DIR_PORT, PAN_DIR_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TILT_STEP_PORT, TILT_STEP_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TILT_DIR_PORT, TILT_DIR_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    }

    void set_laser(bool enable) {
        laser_active = enable;
        HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void step_delay_us(uint32_t us) {
        uint32_t cycles = us * 170;
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles);
    }

    void update_target(float32_t target_pan, float32_t target_tilt, bool tracking_valid) {
        set_laser(tracking_valid);

        if (!tracking_valid) {
            return;
        }

        float32_t delta_pan = target_pan - current_pan_deg;
        int32_t steps_pan = (int32_t)roundf(delta_pan / DEG_PER_MICROSTEP);

        if (steps_pan != 0) {
            HAL_GPIO_WritePin(PAN_DIR_PORT, PAN_DIR_PIN, (steps_pan > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            uint32_t count = (uint32_t)abs(steps_pan);
            for (uint32_t i = 0; i < count; i++) {
                HAL_GPIO_WritePin(PAN_STEP_PORT, PAN_STEP_PIN, GPIO_PIN_SET);
                step_delay_us(2);
                HAL_GPIO_WritePin(PAN_STEP_PORT, PAN_STEP_PIN, GPIO_PIN_RESET);
                step_delay_us(2);
            }
            current_pan_deg += (float32_t)steps_pan * DEG_PER_MICROSTEP;
            total_pan_steps += steps_pan;
        }

        float32_t delta_tilt = target_tilt - current_tilt_deg;
        int32_t steps_tilt = (int32_t)roundf(delta_tilt / DEG_PER_MICROSTEP);

        if (steps_tilt != 0) {
            HAL_GPIO_WritePin(TILT_DIR_PORT, TILT_DIR_PIN, (steps_tilt > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            uint32_t count = (uint32_t)abs(steps_tilt);
            for (uint32_t i = 0; i < count; i++) {
                HAL_GPIO_WritePin(TILT_STEP_PORT, TILT_STEP_PIN, GPIO_PIN_SET);
                step_delay_us(2);
                HAL_GPIO_WritePin(TILT_STEP_PORT, TILT_STEP_PIN, GPIO_PIN_RESET);
                step_delay_us(2);
            }
            current_tilt_deg += (float32_t)steps_tilt * DEG_PER_MICROSTEP;
            total_tilt_steps += steps_tilt;
        }
    }
};

// --- Motor Central de Processamento Radar FMCW ---
class RadarEngine {
public:
    // Buffers DMA dedicados para cada um dos 4 canais de RF físicos
    uint16_t adc1_dma_buffer[DMA_BUFFER_SIZE]; // Antena 0 (Referência)
    uint16_t adc2_dma_buffer[DMA_BUFFER_SIZE]; // Antena 1 (Azimute)
    uint16_t adc3_dma_buffer[DMA_BUFFER_SIZE]; // Antena 2 (Elevação)
    uint16_t adc4_dma_buffer[DMA_BUFFER_SIZE]; // Antena 3 (Diagonal)

    float32_t input_signal[FFT_SIZE];
    float32_t windowed_signal[FFT_SIZE];
    float32_t hann_window[FFT_SIZE];
    float32_t fft_output[FFT_SIZE];

    float32_t rd_matrix[NUM_CHIRPS][RANGE_BINS * 2];
    float32_t slow_time_col[NUM_CHIRPS * 2];
    float32_t doppler_mag[NUM_CHIRPS];

    float32_t slow_time_ant1[NUM_CHIRPS * 2];
    float32_t slow_time_ant2[NUM_CHIRPS * 2];

    uint32_t chirp_counter = 0;

    volatile uint32_t burst_cycles = 0;
    volatile float32_t burst_time_us = 0.0f;
    volatile float32_t peak_value = 0.0f;
    volatile uint32_t bursts_processed = 0;

    volatile float32_t target_range_m = 0.0f;
    volatile float32_t target_azimuth_deg = 0.0f;
    volatile float32_t target_elevation_deg = 0.0f;
    volatile float32_t target_velocity_mps = 0.0f;

    ExtendedKalmanFilter6D ekf;
    volatile float32_t est_x = 0.0f;
    volatile float32_t est_y = 0.0f;
    volatile float32_t est_z = 0.0f;
    volatile float32_t turret_pan_deg = 0.0f;
    volatile float32_t turret_tilt_deg = 0.0f;

    TurretActuator turret;
    BallisticTargetSimulator sim_target;
    TelemetryTransmitter telemetry_tx;

    float32_t sim_delta_psi_x = 0.0f;
    float32_t sim_delta_psi_y = 0.0f;

    volatile bool half_buffer_ready = false;
    volatile bool full_buffer_ready = false;

private:
    arm_rfft_fast_instance_f32 rfft_instance;

public:
    void init() {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        arm_rfft_fast_init_f32(&rfft_instance, FFT_SIZE);

        for (uint32_t i = 0; i < FFT_SIZE; i++) {
            hann_window[i] = 0.5f * (1.0f - cosf(2.0f * PI_F * (float32_t)i / (float32_t)(FFT_SIZE - 1)));
        }

        sim_target.reset();
        turret.init();

        // 1. Calibração dos 4 núcleos conversores analógicos independentes
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

        // 2. Armação das transferências circulares de DMA para os 4 canais
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, DMA_BUFFER_SIZE);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buffer, DMA_BUFFER_SIZE);
        HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_dma_buffer, DMA_BUFFER_SIZE);
        HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc4_dma_buffer, DMA_BUFFER_SIZE);

        // 3. Disparo mestre de amostragem síncrona a 1 MSPS
        HAL_TIM_Base_Start(&htim2);
    }

    void process_fast_time(uint32_t buffer_offset) {

#if SIMULATE_DYNAMIC_TARGET
        float32_t r_xy_sq = sim_target.x * sim_target.x + sim_target.y * sim_target.y;
        float32_t r = sqrtf(r_xy_sq + sim_target.z * sim_target.z);
        float32_t r_xy = sqrtf(r_xy_sq);
        float32_t theta = atan2f(sim_target.y, sim_target.x);
        float32_t phi = atan2f(sim_target.z, r_xy);
        float32_t vr = (sim_target.x * sim_target.vx + sim_target.y * sim_target.vy + sim_target.z * sim_target.vz) / r;

        float32_t k_range = r / RANGE_RES_M;
        float32_t k_doppler = vr / VELOCITY_RES;

        sim_delta_psi_x = PI_F * sinf(theta);
        sim_delta_psi_y = PI_F * sinf(phi);

        float32_t phase_slow = 2.0f * PI_F * (k_doppler * (float32_t)chirp_counter / (float32_t)NUM_CHIRPS);

        for (uint32_t i = 0; i < FFT_SIZE; i++) {
            float32_t phase_fast = 2.0f * PI_F * (k_range * (float32_t)i / (float32_t)(FFT_SIZE));
            input_signal[i] = 0.8f * cosf(phase_fast + phase_slow);
        }
#else
        // Coleta real do sinal de IF do Canal 0
        const uint16_t* raw_ant0 = &adc1_dma_buffer[buffer_offset];

        for (uint32_t i = 0; i < FFT_SIZE; i++) {
            input_signal[i] = ((float32_t)raw_ant0[i] - 2048.0f) / 2048.0f;
        }
#endif

        arm_mult_f32(input_signal, hann_window, windowed_signal, FFT_SIZE);
        arm_rfft_fast_f32(&rfft_instance, windowed_signal, fft_output, 0);

        rd_matrix[chirp_counter][0] = fft_output[0];
        rd_matrix[chirp_counter][1] = 0.0f;

        for (uint32_t k = 1; k < RANGE_BINS; k++) {
            rd_matrix[chirp_counter][2 * k]     = fft_output[2 * k];
            rd_matrix[chirp_counter][2 * k + 1] = fft_output[2 * k + 1];
        }

        chirp_counter++;
        if (chirp_counter >= NUM_CHIRPS) {
            chirp_counter = 0;
            process_slow_time(buffer_offset);
        }
    }

    void process_slow_time(uint32_t buffer_offset) {
        uint32_t start_cycles = DWT->CYCCNT;

        float32_t max_energy = 0.0f;
        uint32_t best_range = 0;
        uint32_t best_doppler_idx = 0;
        int32_t best_doppler = 0;

        // 1. Busca do pico Range-Doppler na Antena 0
        for (uint32_t r = 2; r < RANGE_BINS; r++) {
            for (uint32_t m = 0; m < NUM_CHIRPS; m++) {
                slow_time_col[2 * m]     = rd_matrix[m][2 * r];
                slow_time_col[2 * m + 1] = rd_matrix[m][2 * r + 1];
            }

            arm_cfft_f32(&arm_cfft_sR_f32_len16, slow_time_col, 0, 1);
            arm_cmplx_mag_squared_f32(slow_time_col, doppler_mag, NUM_CHIRPS);

            doppler_mag[0] = 0.0f;

            for (uint32_t d = 0; d < NUM_CHIRPS; d++) {
                if (doppler_mag[d] > max_energy) {
                    max_energy = doppler_mag[d];
                    best_range = r;
                    best_doppler_idx = d;
                    best_doppler = (d <= NUM_CHIRPS / 2) ? (int32_t)d : (int32_t)d - (int32_t)NUM_CHIRPS;
                }
            }
        }

        // 2. Extração de fase na Antena 0
        for (uint32_t m = 0; m < NUM_CHIRPS; m++) {
            slow_time_col[2 * m]     = rd_matrix[m][2 * best_range];
            slow_time_col[2 * m + 1] = rd_matrix[m][2 * best_range + 1];
        }
        arm_cfft_f32(&arm_cfft_sR_f32_len16, slow_time_col, 0, 1);
        float32_t phase_ant0 = atan2f(slow_time_col[2 * best_doppler_idx + 1], slow_time_col[2 * best_doppler_idx]);

        // 3. Emulação sintética coerente ou leitura física das Antenas 1 e 2
#if SIMULATE_DYNAMIC_TARGET
        float32_t cos_dx = cosf(sim_delta_psi_x);
        float32_t sin_dx = sinf(sim_delta_psi_x);
        float32_t cos_dy = cosf(sim_delta_psi_y);
        float32_t sin_dy = sinf(sim_delta_psi_y);

        for (uint32_t m = 0; m < NUM_CHIRPS; m++) {
            float32_t re = rd_matrix[m][2 * best_range];
            float32_t im = rd_matrix[m][2 * best_range + 1];

            slow_time_ant1[2 * m]     = re * cos_dx - im * sin_dx;
            slow_time_ant1[2 * m + 1] = re * sin_dx + im * cos_dx;

            slow_time_ant2[2 * m]     = re * cos_dy - im * sin_dy;
            slow_time_ant2[2 * m + 1] = re * sin_dy + im * cos_dy;
        }
#else
        // Leitura física: as amostras brutas de adc2 e adc3 alimentam o vetor
        const uint16_t* raw_ant1 = &adc2_dma_buffer[buffer_offset];
        const uint16_t* raw_ant2 = &adc3_dma_buffer[buffer_offset];
        for (uint32_t m = 0; m < NUM_CHIRPS; m++) {
            slow_time_ant1[2 * m]     = ((float32_t)raw_ant1[m] - 2048.0f) / 2048.0f;
            slow_time_ant1[2 * m + 1] = 0.0f;
            slow_time_ant2[2 * m]     = ((float32_t)raw_ant2[m] - 2048.0f) / 2048.0f;
            slow_time_ant2[2 * m + 1] = 0.0f;
        }
#endif

        // 4. Transformada Doppler e extração de fase nas Antenas 1 e 2
        arm_cfft_f32(&arm_cfft_sR_f32_len16, slow_time_ant1, 0, 1);
        float32_t phase_ant1 = atan2f(slow_time_ant1[2 * best_doppler_idx + 1], slow_time_ant1[2 * best_doppler_idx]);

        arm_cfft_f32(&arm_cfft_sR_f32_len16, slow_time_ant2, 0, 1);
        float32_t phase_ant2 = atan2f(slow_time_ant2[2 * best_doppler_idx + 1], slow_time_ant2[2 * best_doppler_idx]);

        // 5. Interferometria de fase DoA
        float32_t d_psi_x = atan2f(sinf(phase_ant1 - phase_ant0), cosf(phase_ant1 - phase_ant0));
        float32_t d_psi_y = atan2f(sinf(phase_ant2 - phase_ant0), cosf(phase_ant2 - phase_ant0));

        float32_t sin_theta = d_psi_x / PI_F;
        if (sin_theta > 1.0f) sin_theta = 1.0f;
        if (sin_theta < -1.0f) sin_theta = -1.0f;

        float32_t sin_phi = d_psi_y / PI_F;
        if (sin_phi > 1.0f) sin_phi = 1.0f;
        if (sin_phi < -1.0f) sin_phi = -1.0f;

        // 6. Vetor de medição Z = [r, theta, phi, vr]
        float32_t meas_r = (float32_t)best_range * RANGE_RES_M;
        float32_t meas_theta = asinf(sin_theta);
        float32_t meas_phi = asinf(sin_phi);
        float32_t meas_vr = (float32_t)best_doppler * VELOCITY_RES;

        target_range_m = meas_r;
        target_azimuth_deg = meas_theta * RAD_TO_DEG;
        target_elevation_deg = meas_phi * RAD_TO_DEG;
        target_velocity_mps = meas_vr;

        // 7. EKF 6D com Track Gating
        const float32_t dt_burst = (float32_t)NUM_CHIRPS * CHIRP_TIME;
        ekf.predict(dt_burst);

        float32_t pred_r = sqrtf(ekf.x[0] * ekf.x[0] + ekf.x[1] * ekf.x[1] + ekf.x[2] * ekf.x[2]);
        if (ekf.initialized && fabsf(meas_r - pred_r) > 3.0f) {
            ekf.reset();
        }

        ekf.update(meas_r, meas_theta, meas_phi, meas_vr);

        est_x = ekf.x[0];
        est_y = ekf.x[1];
        est_z = ekf.x[2];

        // 8. Cinemática da Torreta Pan-Tilt
        turret_pan_deg = atan2f(est_y, est_x) * RAD_TO_DEG;
        turret_tilt_deg = atan2f(est_z, sqrtf(est_x * est_x + est_y * est_y)) * RAD_TO_DEG;

        // 9. Atualização Mecânica dos Motores e Disparo do Laser
        turret.update_target(turret_pan_deg, turret_tilt_deg, ekf.initialized);

        // 10. Serialização e Transmissão USB CDC
#if ENABLE_USB_TELEMETRY
        telemetry_tx.send(sim_target.x, sim_target.y, sim_target.z,
                          est_x, est_y, est_z,
                          turret_pan_deg, turret_tilt_deg);
#endif

#if SIMULATE_DYNAMIC_TARGET
        sim_target.update(dt_burst);
#endif

        burst_cycles = DWT->CYCCNT - start_cycles;
        burst_time_us = (float32_t)burst_cycles / 170.0f;
        peak_value = max_energy;
        bursts_processed++;
    }

    void run() {
        if (half_buffer_ready) {
            half_buffer_ready = false;
            process_fast_time(0);
        }

        if (full_buffer_ready) {
            full_buffer_ready = false;
            process_fast_time(FFT_SIZE);
        }
    }
};

RadarEngine radar;

// Como os 4 ADCs disparam sincronizados pelo TIM2, o ADC1 atua como relógio mestre para o software
extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        radar.half_buffer_ready = true;
    }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        radar.full_buffer_ready = true;
    }
}

extern "C" void app_main(void) {
    radar.init();

    while (1) {
        radar.run();
    }
}
