#include <stddef.h>
#include "ladrc.h"

static float ladrc_clamp(float val, float limit)
{
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}

void LADRC_Init(LADRC_t *ladrc, float b0, float wc, float wo, float r, float dt, float u_max)
{
    if (ladrc == NULL) return;

    ladrc->b0 = (b0 == 0.0f) ? 0.0001f : b0;
    ladrc->wc = wc;
    ladrc->wo = wo;
    ladrc->r  = r;  // New: Tracking speed
    ladrc->dt = dt;
    ladrc->u_max = (u_max > 0.0f) ? u_max : 0.0f;

    // Calculate Gains
    ladrc->kp = wc * wc;
    ladrc->kd = 2.0f * wc;

    ladrc->beta1 = 3.0f * wo;
    ladrc->beta2 = 3.0f * wo * wo;
    ladrc->beta3 = wo * wo * wo;

    // Reset states
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->z3 = 0.0f;
    
    ladrc->v1 = 0.0f;
    ladrc->v2 = 0.0f;
    
    ladrc->last_u = 0.0f;
}

void LADRC_Reset(LADRC_t *ladrc, float y0)
{
    if (ladrc == NULL) return;

    // ESO tracks current output
    ladrc->z1 = y0;
    ladrc->z2 = 0.0f;
    ladrc->z3 = 0.0f;
    
    // TD tracks current output (prevents jump at startup)
    ladrc->v1 = y0;
    ladrc->v2 = 0.0f;
    
    ladrc->last_u = 0.0f;
}

float LADRC_Update(LADRC_t *ladrc, float ref, float y)
{   
	ladrc->kp = ladrc->wc * ladrc->wc;
    ladrc->kd = 2.0f * ladrc->wc;

    ladrc->beta1 = 3.0f * ladrc->wo;
    ladrc->beta2 = 3.0f * ladrc->wo * ladrc->wo;
    ladrc->beta3 = ladrc->wo *ladrc-> wo *ladrc-> wo;
    if (ladrc == NULL) return 0.0f;

    const float dt = ladrc->dt;

    // --- 1. Linear Tracking Differentiator (TD) ---
    // Formula: v_dotdot = -r^2 * (v1 - ref) - 2*r * v2
    // This is a critically damped system that chases 'ref' smoothly.
    float td_acc = -1.0f * ladrc->r * ladrc->r * (ladrc->v1 - ref) - 2.0f * ladrc->r * ladrc->v2;
    
    ladrc->v1 += ladrc->v2 * dt;
    ladrc->v2 += td_acc * dt;

    // --- 2. Linear Extended State Observer (LESO) ---
    const float y_err = y - ladrc->z1;
    
    ladrc->z1 += (ladrc->z2 + ladrc->beta1 * y_err) * dt;
    ladrc->z2 += (ladrc->z3 + ladrc->b0 * ladrc->last_u + ladrc->beta2 * y_err) * dt;
    ladrc->z3 += (ladrc->beta3 * y_err) * dt;

    // --- 3. Linear State Error Feedback (LSEF) ---
    // Controller now tracks the TD's smooth trajectory (v1, v2) instead of raw ref
    const float e1 = ladrc->v1 - ladrc->z1; // Position Error
    const float e2 = ladrc->v2 - ladrc->z2; // Velocity Error
    
    const float u0 = ladrc->kp * e1 + ladrc->kd * e2;

    // --- 4. Disturbance Rejection & Output ---
    float u = (u0 - ladrc->z3) / ladrc->b0;

    u = ladrc_clamp(u, ladrc->u_max);

    ladrc->last_u = u;
    return u;
}
