#include "kalman2d.h"
#include <string.h>
#include <math.h>

/* ---------- GLOBAL FILTER ---------- */
Kalman2D kf;

/* ---------- UTILS ---------- */
static void mat_identity(float *A, int n)
{
    memset(A, 0, n*n*sizeof(float));
    for(int i=0;i<n;i++)
        A[i*n+i] = 1.0f;
}

/* ---------- INIT ---------- */
void kf_init(Kalman2D *kf)
{
    memset(kf, 0, sizeof(Kalman2D));

    for(int i=0;i<KF_STATE_DIM;i++)
        kf->P[i][i] = 5.0f;

    // Process noise (TUNING)
    kf->Q[0][0] = 0.01f;
    kf->Q[1][1] = 0.01f;
    kf->Q[2][2] = 1.0f;  //Velocity X
    kf->Q[3][3] = 1.0f;   //Velocity Y
    kf->Q[4][4] = 5.0f;   // heading
    kf->Q[5][5] = 0.01f;  // gyro bias
}

/* ---------- PREDICT (EKF) ---------- */
void kf_predict(Kalman2D *kf, float dt, float ax, float ay, float gyro_z,float *ax_w_out, float *ay_w_out){
    float *X = kf->X;

    float theta = X[4];
    float bias  = X[5];

    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    // Rotate accel to world frame
    float ax_w = ax * cos_t - ay * sin_t;
    float ay_w = ax * sin_t + ay * cos_t;

    /* export debug */
    if (ax_w_out) *ax_w_out = ax_w;
    if (ay_w_out) *ay_w_out = ay_w;

    /* STATE */
    X[0] += X[2]*dt;
    X[1] += X[3]*dt;

    X[2] += ax_w * dt;
    X[3] += ay_w * dt;

    X[4] += (gyro_z - bias) * dt;

    /* ---------- NORMALIZE HEADING ---------- */
    if (X[4] > M_PI)  X[4] -= 2*M_PI;
    if (X[4] < -M_PI) X[4] += 2*M_PI;

    /* JACOBIAN */
    float A[KF_STATE_DIM][KF_STATE_DIM] = {0};
    mat_identity((float*)A, KF_STATE_DIM);

    A[0][2] = dt;
    A[1][3] = dt;

    A[2][4] = (-ax * sin_t - ay * cos_t) * dt;
    A[3][4] = ( ax * cos_t - ay * sin_t) * dt;

    A[4][5] = -dt;

    /* COVARIANCE */
    float AP[6][6] = {0};
    float APA[6][6] = {0};

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            for(int k=0;k<6;k++)
                AP[i][j] += A[i][k]*kf->P[k][j];

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            for(int k=0;k<6;k++)
                APA[i][j] += AP[i][k]*A[j][k];

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            kf->P[i][j] = APA[i][j] + kf->Q[i][j];
}

/* ---------- GENERIC UPDATE ---------- */
static void kf_update_generic(Kalman2D *kf, float z, int idx, float R)
{
    float y = z - kf->X[idx];

    float S = kf->P[idx][idx] + R;

    float K[6];
    for(int i=0;i<6;i++)
        K[i] = kf->P[i][idx] / S;

    for(int i=0;i<6;i++)
        kf->X[i] += K[i] * y;

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            kf->P[i][j] -= K[i] * kf->P[idx][j];
}

/* ---------- GPS ---------- */
void kf_update_gps(Kalman2D *kf, float x, float y)
{
    float R;

    if (telemetry_data.num_sats >= 10)
        R = 0.025f;   // VERY strong trust
    else if (telemetry_data.num_sats >= 8)
        R = 0.08f;
    else if (telemetry_data.num_sats >= 6)
        R = 0.8f;
    else
        R = 3.0f;

    float vx_est = (x - kf->X[0]) * 5.0f;  // since 5 Hz (200 ms)
    float vy_est = (y - kf->X[1]) * 5.0f;

    kf_update_generic(kf, vx_est, 2, R * 1.2f);
    kf_update_generic(kf, vy_est, 3, R * 1.2f);

    kf_update_generic(kf, x, 0, R);
    kf_update_generic(kf, y, 1, R);
}

/* ---------- VELOCITY ---------- */
void kf_update_velocity(Kalman2D *kf, float vx, float vy, bool R_stopped)
{
    float R = R_stopped ? 0.00001f : 0.05f;
    kf_update_generic(kf, vx, 2, R);
    kf_update_generic(kf, vy, 3, R);
}

/* ---------- HEADING (MAG) ---------- */
// void kf_update_heading(Kalman2D *kf, float heading)
// {
//     float y = heading - kf->X[4];

//     while (y > M_PI) y -= 2*M_PI;
//     while (y < -M_PI) y += 2*M_PI;

//     float S = kf->P[4][4] + 0.1f;

//     float K[6];
//     for(int i=0;i<6;i++)
//         K[i] = kf->P[i][4] / S;

//     for(int i=0;i<6;i++)
//         kf->X[i] += K[i] * y;

//     for(int i=0;i<6;i++)
//         for(int j=0;j<6;j++)
//             kf->P[i][j] -= K[i] * kf->P[4][j];
// }
// void kf_update_heading(Kalman2D *kf, float heading)
// {
//     float y = heading - kf->X[4];

//     while (y > M_PI) y -= 2*M_PI;
//     while (y < -M_PI) y += 2*M_PI;

//     float R = 0.3f; // increased (less trust than GPS)

//     float S = kf->P[4][4] + R;

//     float K[6];
//     for(int i=0;i<6;i++)
//         K[i] = kf->P[i][4] / S;

//     for(int i=0;i<6;i++)
//         kf->X[i] += K[i] * y;

//     for(int i=0;i<6;i++)
//         for(int j=0;j<6;j++)
//             kf->P[i][j] -= K[i] * kf->P[4][j];
// }

void kf_update_heading(Kalman2D *kf, float heading, float current, bool calibrated)
{
    float y = heading - kf->X[4];

    while (y > M_PI) y -= 2*M_PI;
    while (y < -M_PI) y += 2*M_PI;

    /* -------- DYNAMIC TRUST -------- */
    float R;

    if (!calibrated)
    {
        R = 5.0f;  // ignore if not calibrated
    }
    else if (fabs(current) > CURRENT_VALID_THRESHOLD)
    {
        R = 5.0f;  // motor interference
    }
    else
    {
        R = 0.01f; // strong trust
    }

    float S = kf->P[4][4] + R;

    float K[6];
    for(int i=0;i<6;i++)
        K[i] = kf->P[i][4] / S;

    for(int i=0;i<6;i++)
        kf->X[i] += K[i] * y;

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            kf->P[i][j] -= K[i] * kf->P[4][j];

    kf->X[5] += 0.01f * y;
    if (kf->X[5] > 0.5f)  kf->X[5] = 0.5f;
    if (kf->X[5] < -0.5f) kf->X[5] = -0.5f;
    kf->P[4][4] += 0.05f;
}

void kf_update_gps_velocity(Kalman2D *kf, float vx, float vy, float speed)
{
    // Reject GPS velocity if too slow (noise region)
    if (speed < GPS_SPEED_MIN_VALID)
        return;

    float R;

    if (telemetry_data.num_sats >= 10)
        R = 0.01f;   // VERY strong trust
    else if (telemetry_data.num_sats >= 8)
        R = 0.02f;
    else if (telemetry_data.num_sats >= 6)
        R = 0.05f;
    else
        R = 0.15f;   // weak

    float vx_err = vx - kf->X[2];
    float vy_err = vy - kf->X[3];

    float err_mag = sqrtf(vx_err*vx_err + vy_err*vy_err);

    /* If EKF is far from GPS → force stronger correction */
    if (err_mag > 2.0f)   // ~7 km/h mismatch
    {
        R *= 0.2f;  // increase trust temporarily
    }

    kf_update_generic(kf, vx, 2, R);
    kf_update_generic(kf, vy, 3, R);
}

void kf_update_heading_gps(Kalman2D *kf, float heading, float speed)
{
    // Only trust GPS heading if moving fast enough
    // if (speed < GPS_SPEED_MIN_VALID)
    //     return;

    float y = heading - kf->X[4];

    while (y > M_PI) y -= 2*M_PI;
    while (y < -M_PI) y += 2*M_PI;

    float R;

    if (telemetry_data.num_sats >= 10)
        R = 0.01f;
    else if (telemetry_data.num_sats >= 8)
        R = 0.02f;
    else
        R = 0.08f;

    float S = kf->P[4][4] + R;

    float K[6];
    for(int i=0;i<6;i++)
        K[i] = kf->P[i][4] / S;

    for(int i=0;i<6;i++)
        kf->X[i] += K[i] * y;

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            kf->P[i][j] -= K[i] * kf->P[4][j];
}

/* ---------- TASK ---------- */
void kalman_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    float ax = 0, ay = 0, gyro_z = 0;
    float gps_speed = 0.0f;

    while (1)
    {
        kf_msg_t msg;
        //bool vehicle_stopped = (fabs(kf.X[2]) < 0.2f && fabs(kf.X[3]) < 0.2f);
        //bool vehicle_stopped = (gps_speed < GPS_SPEED_STATIONARY);


        while (xQueueReceive(kf_queue, &msg, 0) == pdTRUE)
        {
            switch(msg.type)
            {
                case KF_MEAS_ACCEL:
                    ax = msg.a;
                    ay = msg.b;
                    break;
                
                case KF_MEAS_GYRO:
                    gyro_z = msg.a;
                    break;

                case KF_MEAS_GPS:
                    kf_update_gps(&kf, msg.a, msg.b);
                    break;

                case KF_MEAS_GPS_VEL:
                    gps_speed = sqrtf(msg.a*msg.a + msg.b*msg.b);
                    kf_update_gps_velocity(&kf, msg.a, msg.b, gps_speed);
                    break;

                case KF_MEAS_HEADING_GPS:
                    kf_update_heading_gps(&kf, msg.a, gps_speed);
                    break;

                case KF_MEAS_VEL:
                    kf_update_velocity(&kf, msg.a, msg.b, false);
                    break;

                case KF_MEAS_HEADING:{
                    kf_update_heading(&kf, msg.a, msg.b, msg.c);
                    break;
                }
            }
        }

        // Predict AFTER consuming inputs
        // kf_predict(&kf, 0.01f, ax, ay, gyro_z);

        // xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
        // telemetry_data.velocity_x = kf.X[2];
        // telemetry_data.velocity_y = kf.X[3];
        // xSemaphoreGive(telemetry_mutex);

        // Predict AFTER consuming inputs
        float ax_w, ay_w;
        float accel_mag = sqrtf(ax*ax + ay*ay);
        bool imu_stationary = (accel_mag < 0.15f) && (fabs(gyro_z) < 0.05f);
        bool gps_stationary = (gps_speed < GPS_SPEED_STATIONARY);

        bool is_stationary = (gps_speed < GPS_SPEED_STATIONARY); // ONLY GPS decides
        if (is_stationary){
            kf_update_velocity(&kf, 0.0f, 0.0f, true);
            /* kill tiny drift explicitly */
            kf.X[2] *= 0.5f;
            kf.X[3] *= 0.5f;
        }
        kf_predict(&kf, 0.01f, ax, ay, gyro_z, &ax_w, &ay_w);

        // ESP_LOGI("KF_STATE",
        //     "X=%.2f Y=%.2f | Vx=%.2f Vy=%.2f | Th=%.2f deg | Bias=%.4f | gps=%.2f | stat=%d",
        //     kf.X[0],
        //     kf.X[1],
        //     kf.X[2],
        //     kf.X[3],
        //     kf.X[4] * 180.0f / M_PI,
        //     kf.X[5],
        //     gps_speed,
        //     is_stationary
        // );

        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);

        /* ---------- VELOCITY ---------- */
        telemetry_data.velocity_x = kf.X[2];
        telemetry_data.velocity_y = kf.X[3];

        /* ---------- ACCEL (BODY FRAME INPUT) ---------- */
        // telemetry_data.accel_x = ax;
        // telemetry_data.accel_y = ay;

        /* ---------- ACCEL (WORLD FRAME DEBUG) ---------- */
        telemetry_data.accel_x = ax_w;
        telemetry_data.accel_y = ay_w;

        /* ---------- ACCEL MAGNITUDE ---------- */
        //float accel_mag = sqrtf(ax*ax + ay*ay);

        /* ---------- DEG CONVERSION ---------- */
        float heading_deg = kf.X[4] * (180.0f / M_PI);
        if (heading_deg < 0) heading_deg += 360.0f;
        if (heading_deg >= 360.0f) heading_deg -= 360.0f;  
        telemetry_data.orient_z = heading_deg;

        xSemaphoreGive(telemetry_mutex);
        


        vTaskDelayUntil(&last, period);
    }
}