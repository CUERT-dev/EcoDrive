#include "virtual_pmsm.h"
#include <math.h>



void euf_step(float *x, int n, float dt, DerivFunc f, float t, const void *params)
{
    float dxdt[MAX_STATE];
    f(x, t, params, dxdt);
    for(int i = 0; i < n; i++)
    {
        x[i] = x[i] + dxdt[i]*dt;
    }
}

void rk4_step(float *x, int n, float dt, DerivFunc f, float t, const void *params)
{
    float k1[MAX_STATE];
    float k2[MAX_STATE];
    float k3[MAX_STATE];
    float k4[MAX_STATE];
    float xtmp[MAX_STATE];

    // k1
    f(x, t, params, k1);

    // k2
    for(int i = 0; i < n; i++)
        xtmp[i] = x[i] + 0.5f * dt * k1[i];

    f(xtmp, t + 0.5f * dt, params, k2);

    // k3
    for(int i = 0; i < n; i++)
        xtmp[i] = x[i] + 0.5f * dt * k2[i];

    f(xtmp, t + 0.5f * dt, params, k3);

    // k4
    for(int i = 0; i < n; i++)
        xtmp[i] = x[i] + dt * k3[i];

    f(xtmp, t + dt, params, k4);

    // final update
    for(int i = 0; i < n; i++)
    {
        x[i] += dt * (
            k1[i]
          + 2.0f * k2[i]
          + 2.0f * k3[i]
          + k4[i]
        ) / 6.0f;
    }
}

void parke_transform();
void clarke_transform();



void pmsm_didt(const float* i, float t, const void* params, float *didt)
{
    pmsm_model* ctx = (pmsm_model*)(params);
    for(int j = 0; j < 3; j++){
        didt[j] = (ctx->elec.v[j] - ctx->Ra*i[j] - ctx->elec.e[j])/ctx->elec.L[j]; 
    }
}

void pmsm_domegadt(const float* w, float t, const void* params, float *dwdt)
{

}

void pmsm_dthetadt(const float* i, float t, const void* params, float *dxdt)
{

}

void pmsm_update_bemf(pmsm_model *m)
{
    m->elec.e[0] = m->Ke * m->elec.omega_e * sinf(m->elec.theta_e);
    m->elec.e[1] = m->Ke * m->elec.omega_e * sinf(m->elec.theta_e - 2.0f * (M_PI / 3.0f));
    m->elec.e[2] = m->Ke * m->elec.omega_e * sinf(m->elec.theta_e + 2.0f * (M_PI / 3.0f)); 
}

void pmsm_update_inductance(pmsm_model *m)
{
 
    
}

void pmsm_step(pmsm_model *m, float dt, float t)
{
    //step through the electrical model;
    pmsm_update_bemf(m);
    rk4_step(m->elec.i, 3, dt, pmsm_didt, t, m);
    m->elec.theta_e = 2 * M_PI * 600 * t;
    m->elec.theta_e = fmodf(m->elec.theta_e, 2*M_PI);
}



pmsm_model virtual_pmsm;