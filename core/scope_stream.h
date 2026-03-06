#include "elcore.h"


#ifdef __cplusplus
extern "C" {
#endif

//===========================================================================
// elcore_scopestream
//===========================================================================
typedef struct{
    int16_t* buf;
    int16_t sample_depth;
    elcore_rstream_t rbuf;
    int16_t trigger_level;
    uint8_t decimation_counter;
    uint8_t decimation;
    uint16_t trigger_idx;
    uint8_t trig_last;
    uint8_t triggered;
    int16_t to_Fill;
    uint8_t channels_num;
    uint8_t sample_size;
    volatile uint8_t frozen;
}elcore_scopestream_t;

static inline void elcore_scopestream_init(elcore_scopestream_t *cp, int16_t *storage, uint8_t channels_num, uint8_t divs, uint8_t sample_per_div)
{
    cp->buf = storage;
    cp->channels_num = channels_num;
    cp->decimation_counter = 1;
    cp->decimation = 1;
    cp->sample_depth = sample_per_div * divs;
    cp->sample_size = channels_num*sizeof(int16_t);
    cp->trigger_level = 0;
    cp->trigger_idx = 0;
    cp->trig_last = 0;
    cp->triggered = 0;
    cp->to_Fill = 0;
    cp->frozen = false;
    elcore_rstream_init(&cp->rbuf, storage, cp->sample_size, cp->sample_depth);
}

static inline void elcore_scopestream_write(elcore_scopestream_t *cp, int16_t *data)
{
    uint8_t current_trigger = data[0] - cp->trigger_level > 0;
    if(current_trigger && (cp->trig_last == 0) && !cp->triggered)
    {
        cp->trigger_idx = cp->rbuf.head;
        cp->triggered = true;
        cp->to_Fill = 0.75 * cp->sample_depth;
    }
    if(!cp->frozen){
        if(--cp->decimation_counter < 1)
        {
            int16_t *w1, *w2;
            uint16_t c1, c2;
            elcore_rstream_reserveWriteOverride(&cp->rbuf, 1, (void**)&w1, &c1, (void**)&w2, &c2);
            elcore_rstream_commitWrite(&cp->rbuf, 1);
            memcpy(w1, data, cp->sample_size);
            cp->decimation_counter = cp->decimation;
            if(cp->triggered)
            {
                cp->to_Fill = cp->to_Fill - 1;
                if(cp->to_Fill == 0)
                {
                    cp->frozen = true;
                }
            }
        }
    }
    cp->trig_last = current_trigger;
}

static inline int16_t elcore_scopestream_read(elcore_scopestream_t *cp, int16_t *ret, uint8_t count)
{
    uint16_t remaining = 0;
    if(cp->frozen)
    {
        int16_t *r1, *r2;
        uint16_t c1, c2;
        elcore_rstream_peekRead(&cp->rbuf, (void**)&r1, &c1, (void**)&r2, &c2);
        if(c1)memcpy(ret, r1, c1 * cp->sample_size);
        if(c2)memcpy(ret+c1, r2, c2 * cp->sample_size);
        remaining = elcore_rstream_occupied(&cp->rbuf);
        if(remaining == 0)
        {
            cp->frozen = false;
        }
    }
    return remaining;
}

#ifdef __cplusplus
}
#endif