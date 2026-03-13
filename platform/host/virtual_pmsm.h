#pragma once

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>


typedef enum{
    SINUSOIDAL = 0,
    TRAPEZOIDAL = 1,
}bemf_type_t;


#define MAX_STATE 10
typedef void (*DerivFunc)(const float* x, float t, const void* params, float *dxdt);
void euf_step(float *x, int n, float dt, DerivFunc f, float t, const void* params);


typedef struct{
    float i[3];
    float v[3];
    bool driven[3];
}elec_bus;


typedef struct{
    struct{
        float vn;
        float id;
        float iq;
        float e[3];
        float L[3];
    }elec;

    struct{
        float torque_l;
        float torque_e;
        float theta;
        float omega;
    }mech;

    elec_bus *e_bus;

    float Ra;
    float Ld;
    float Lq;
    float Ke;
    float Kt;
    float J;
    float B;
    float pole_pairs;
    bemf_type_t bemf_type;
}pmsm_model;

typedef struct{
    float vbus;
    float min_fw_current;
    elec_bus *e_bus;
}inverter_model;

void inverter_step(inverter_model *inv, float duty[3],int switch_state[3], float dt, float t);
void pmsm_step(pmsm_model *m, float dt, float t);
void pmsm_init(pmsm_model *m, elec_bus *bus);



#ifdef __cplusplus
}
#endif
