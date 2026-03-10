#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#define BUS_VOLTAGE 16

#define MAX_STATE 10
typedef void (*DerivFunc)(const float* x, float t, const void* params, float *dxdt);
void euf_step(float *x, int n, float dt, DerivFunc f, float t, const void* params);

typedef struct{
    struct{
        float i[3];
        float v[3];
        float e[3];
        float theta_e;
        float L[3];
        float omega_e;
        int switch_state[3];
    }elec;

    struct{
        float theta_m;


    }mech;


    float Ra;
    float Ld;
    float Lq;
    float Ke;
    float Kt;
    float J;
    float B;
    float pole_pairs;
}pmsm_model;

void pmsm_step(pmsm_model *m, float dt, float t);



extern pmsm_model virtual_pmsm;




#ifdef __cplusplus
}
#endif
