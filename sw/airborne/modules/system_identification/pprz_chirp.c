#include "pprz_chirp.h"

struct chirp_t* chirp_new(float f0_hz, float f1_hz, float length_s, float current_time_s, bool exponential_chirp, bool fade_in) {
    struct chirp_t* chirp = malloc(sizeof *chirp);
    chirp->f0_hz = f0_hz;
    chirp->f1_hz = f1_hz;

    chirp->length_s = length_s;
    if (fade_in) // The fade-in takes two of the longest wave-lengths, total_length is including that time
        chirp->total_length_s = length_s + 2 / f0_hz;

    chirp->start_time_s = current_time_s;
    chirp->exponential_chirp = exponential_chirp;
    chirp->fade_in = fade_in;

    chirp->current_frequency_hz = 0;
    chirp->current_value = 0;
    chirp->percentage_done = 0;
    return chirp;
}

void chirp_reset(struct chirp_t* chirp, float current_time_s) {
    chirp->current_time_s = current_time_s;
    chirp->start_time_s = current_time_s;
    chirp->current_frequency_hz = 0;
    chirp->current_value = 0;
    chirp->percentage_done = 0;
}

void chirp_del(struct chirp_t* chirp) {
    free(chirp);
}

bool chirp_is_running(struct chirp_t* chirp, float current_time_s) {
    float t = current_time_s - chirp->start_time_s;
    return (t >= 0) && (t <= chirp->total_length_s);
}

float chirp_update(struct chirp_t* chirp, float current_time_s) {
    if (!chirp_is_running(chirp, current_time_s)) { // Outside the chirp interval, return 0
        return 0;
    }

    float t = current_time_s - chirp->start_time_s; // Time since the start of the chirp
    chirp->current_time_s = current_time_s;
    chirp->percentage_done = t / chirp->total_length_s;
    // Fade-in is two times the wavelength of f0
    if (chirp->fade_in && t < 2 / chirp->f0_hz) {
        chirp->current_frequency_hz = chirp->f0_hz;
        if (t <= 1 / chirp->f0_hz) { // First wavelength has amplitude increasing until unity over one wavelength
            chirp->current_value = t * chirp->f0_hz * sinf(t * 2 * M_PI * chirp->f0_hz);
        }
        else { // Second wavelength has constant amplitude
            chirp->current_value = sinf(t * 2 * M_PI * chirp->f0_hz);
        }

        return chirp->current_value;
    }

    // If the chirp fades in, the current time t is the time since the fade-in stopped
    if (chirp->fade_in)
        t -= 2 / chirp->f0_hz;

    if (chirp->exponential_chirp) { // See the book referenced in the header for the equations
        float exponential = exp(chirp_C1 * t / chirp->length_s);
        float K = chirp_C2 * (exponential - 1);

        chirp->current_frequency_hz = chirp->f0_hz + K * (chirp->f1_hz - chirp->f0_hz);

        float theta = 2 * M_PI * (chirp->f0_hz*t
            + (chirp->f1_hz - chirp->f0_hz)*(chirp->length_s / chirp_C1 * K - chirp_C2 * t));

        chirp->current_value = sinf(theta);
    }
    else { // linear-time chirp
        float k = (chirp->f1_hz - chirp->f0_hz) / chirp->length_s;

        chirp->current_frequency_hz = k * t;
        chirp->current_value = sinf(2 * M_PI * t * (chirp->f0_hz + chirp->current_frequency_hz / 2));
    }

    return chirp->current_value;
}

void chirp_print(struct chirp_t* chirp) {
    printf("f0: %.10f\n", chirp->f0_hz);
    printf("f1: %.10f\n", chirp->f1_hz);
    printf("start time: %.10f\n", chirp->start_time_s);
    printf("length: %.10f\n", chirp->length_s);
    printf("Current frequency: %.10f\n", chirp->current_frequency_hz);
    printf("current value: %.10f\n", chirp->current_value);
}
