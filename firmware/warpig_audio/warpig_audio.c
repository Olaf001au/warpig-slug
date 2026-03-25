// ============================================================================
// warpig_audio — Synthesis audio engine for MicroPython
//
// Extracted and ported from PicoSound (IWILZ, GPL-3.0) to a MicroPython
// native C module. Works on ESP32-S3 (Xtensa) and RP2350 (Cortex-M33).
//
// Features:
//   - 8-channel mixer with per-channel volume
//   - Waveform synthesis: sine, square, sawtooth, triangle, noise
//   - Explosion generator (game-quality procedural)
//   - Melody sequencer with note arrays
//   - Frequency sweeps
//   - Fills a sample buffer that Python outputs via PWM/I2S/DAC
//
// Usage:
//   import warpig_audio as audio
//   audio.init(sample_rate=16000, channels=4)
//   audio.play(0, audio.SQUARE, freq=440, duration=200, volume=80)
//   audio.sweep(1, 1500, 200, duration=300, volume=70)
//   audio.explosion(2, volume=90)
//   buf = bytearray(512)
//   n = audio.fill(buf)  # fills buf with mixed int16 samples
// ============================================================================

#include "py/runtime.h"
#include "py/obj.h"
#include "py/binary.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// CONSTANTS
// ============================================================================

#define MAX_CHANNELS 8
#define DEFAULT_SAMPLE_RATE 16000
#define DEFAULT_FADE_SAMPLES 640  // 40ms @ 16kHz

// Waveform types
enum {
    WAVE_NONE = 0,
    WAVE_SINE,
    WAVE_SQUARE,
    WAVE_SAWTOOTH,
    WAVE_TRIANGLE,
    WAVE_NOISE,
    WAVE_EXPLOSION,
};

// ============================================================================
// CHANNEL STATE
// ============================================================================

typedef struct {
    uint8_t enabled;
    uint8_t wave_type;
    uint8_t volume;         // 0-100
    float frequency;
    float freq_start;
    float freq_end;
    int16_t amplitude;
    float phase;

    // Duration
    uint32_t total_samples;
    uint32_t position;

    // Sweep
    uint32_t sweep_samples;
    uint32_t sweep_pos;

    // Explosion state
    uint32_t expl_iter;
    uint32_t expl_max_iter;
    uint32_t expl_repeat;
    uint32_t expl_repeat_count;

    // Melody
    const float *melody_freqs;     // NULL or array of frequencies
    const uint16_t *melody_durs;   // NULL or array of durations (ms)
    uint16_t melody_count;
    uint16_t melody_idx;
    uint32_t melody_note_start;
    uint8_t melody_wave;
} audio_channel_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================

static audio_channel_t channels[MAX_CHANNELS];
static uint32_t g_sample_rate = DEFAULT_SAMPLE_RATE;
static uint8_t g_num_channels = 4;
static uint8_t g_master_volume = 80;
static uint8_t g_initialized = 0;

// ============================================================================
// INTERNAL: reset channel
// ============================================================================

static void reset_channel(int ch) {
    memset(&channels[ch], 0, sizeof(audio_channel_t));
}

// ============================================================================
// INTERNAL: generate one sample from a channel
// ============================================================================

static int16_t channel_sample(audio_channel_t *ch) {
    if (!ch->enabled) return 0;

    // Duration check
    if (ch->total_samples > 0) {
        if (ch->position >= ch->total_samples) {
            ch->enabled = 0;
            return 0;
        }
        ch->position++;
    }

    // Frequency sweep
    if (ch->freq_end != 0 && ch->sweep_samples > 0 && ch->wave_type != WAVE_NOISE && ch->wave_type != WAVE_EXPLOSION) {
        if (ch->sweep_pos < ch->sweep_samples) {
            float progress = (float)ch->sweep_pos / (float)ch->sweep_samples;
            ch->frequency = ch->freq_start + (ch->freq_end - ch->freq_start) * progress;
            ch->sweep_pos++;
        } else {
            ch->frequency = ch->freq_end;
        }
    }

    // Melody sequencer
    if (ch->melody_freqs != NULL && ch->melody_durs != NULL) {
        uint32_t elapsed_ms = (ch->position - ch->melody_note_start) * 1000 / g_sample_rate;
        if (elapsed_ms >= ch->melody_durs[ch->melody_idx]) {
            ch->melody_idx++;
            if (ch->melody_idx >= ch->melody_count) {
                ch->enabled = 0;
                return 0;
            }
            ch->frequency = ch->melody_freqs[ch->melody_idx];
            ch->phase = 0.0f;
            ch->melody_note_start = ch->position;
        }
    }

    int16_t sample = 0;
    float phase_inc = ch->frequency / (float)g_sample_rate;

    switch (ch->wave_type) {
        case WAVE_SINE:
            sample = (int16_t)(ch->amplitude * sinf(2.0f * M_PI * ch->phase));
            ch->phase += phase_inc;
            if (ch->phase >= 1.0f) ch->phase -= 1.0f;
            break;

        case WAVE_SQUARE:
            sample = (ch->phase < 0.5f) ? ch->amplitude : -ch->amplitude;
            ch->phase += phase_inc;
            if (ch->phase >= 1.0f) ch->phase -= 1.0f;
            break;

        case WAVE_SAWTOOTH:
            sample = (int16_t)(ch->amplitude * (2.0f * ch->phase - 1.0f));
            ch->phase += phase_inc;
            if (ch->phase >= 1.0f) ch->phase -= 1.0f;
            break;

        case WAVE_TRIANGLE:
            if (ch->phase < 0.5f)
                sample = (int16_t)(ch->amplitude * (4.0f * ch->phase - 1.0f));
            else
                sample = (int16_t)(ch->amplitude * (3.0f - 4.0f * ch->phase));
            ch->phase += phase_inc;
            if (ch->phase >= 1.0f) ch->phase -= 1.0f;
            break;

        case WAVE_NOISE:
            sample = (int16_t)((rand() % (ch->amplitude * 2)) - ch->amplitude);
            break;

        case WAVE_EXPLOSION: {
            if (ch->expl_max_iter == 0) {
                ch->expl_max_iter = 150;
                ch->expl_iter = 0;
                ch->expl_repeat = 0;
                ch->expl_repeat_count = 0;
                ch->phase = 0.0f;
            }
            sample = 0;
            if (ch->expl_repeat_count < ch->expl_repeat) {
                uint32_t k = ch->expl_iter;
                uint32_t base_period = 1000 + (k * 24);
                uint32_t impulse_dur = (base_period / 4) + (k > 0 ? (k - 1) * 12 : 0);
                uint32_t impulse_samp = (impulse_dur * g_sample_rate) / 1000000;
                if (impulse_samp < 2) impulse_samp = 2;
                uint32_t pos = (uint32_t)ch->phase;
                if (pos < impulse_samp * 2) {
                    sample = (pos < impulse_samp) ? ch->amplitude : -ch->amplitude;
                } else {
                    ch->expl_repeat_count++;
                    ch->phase = 0.0f;
                }
                ch->phase += 1.0f;
            } else {
                ch->expl_iter++;
                if (ch->expl_iter >= ch->expl_max_iter) {
                    ch->enabled = 0;
                    break;
                }
                ch->expl_repeat = (rand() % 3) * 3;
                ch->expl_repeat_count = 0;
                ch->phase = 0.0f;
            }
            float fade = 1.0f - ((float)ch->expl_iter / (float)ch->expl_max_iter);
            sample = (int16_t)(sample * fade);
            break;
        }

        default:
            sample = 0;
            break;
    }

    // Apply channel volume
    sample = (int16_t)((int32_t)sample * ch->volume / 100);
    return sample;
}

// ============================================================================
// INTERNAL: mix all channels into one sample
// ============================================================================

static int16_t mix_sample(void) {
    int32_t mix = 0;
    for (int i = 0; i < g_num_channels; i++) {
        mix += channel_sample(&channels[i]);
    }
    // Apply master volume
    mix = mix * g_master_volume / 100;
    // Clamp
    if (mix > 32767) mix = 32767;
    if (mix < -32768) mix = -32768;
    return (int16_t)mix;
}

// ============================================================================
// init(sample_rate=16000, channels=4, master_volume=80)
// ============================================================================

static mp_obj_t warpig_audio_init(size_t n_args, const mp_obj_t *args) {
    g_sample_rate = (n_args > 0) ? mp_obj_get_int(args[0]) : DEFAULT_SAMPLE_RATE;
    g_num_channels = (n_args > 1) ? mp_obj_get_int(args[1]) : 4;
    g_master_volume = (n_args > 2) ? mp_obj_get_int(args[2]) : 80;
    if (g_num_channels > MAX_CHANNELS) g_num_channels = MAX_CHANNELS;
    for (int i = 0; i < MAX_CHANNELS; i++) reset_channel(i);
    srand(0xDEADBEEF);  // deterministic seed; caller can re-seed
    g_initialized = 1;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(warpig_audio_init_obj, 0, 3, warpig_audio_init);

// ============================================================================
// play(ch, wave_type, freq=440, duration=0, volume=80, amplitude=12000)
// duration in ms, 0 = infinite until stop()
// ============================================================================

static mp_obj_t warpig_audio_play(size_t n_args, const mp_obj_t *args) {
    int ch = mp_obj_get_int(args[0]);
    if (ch < 0 || ch >= g_num_channels) return mp_const_none;

    int wave = mp_obj_get_int(args[1]);
    float freq = (n_args > 2) ? mp_obj_get_float(args[2]) : 440.0f;
    uint32_t dur_ms = (n_args > 3) ? mp_obj_get_int(args[3]) : 0;
    uint8_t vol = (n_args > 4) ? mp_obj_get_int(args[4]) : 80;
    int16_t amp = (n_args > 5) ? mp_obj_get_int(args[5]) : 12000;

    reset_channel(ch);
    channels[ch].enabled = 1;
    channels[ch].wave_type = wave;
    channels[ch].frequency = freq;
    channels[ch].freq_start = freq;
    channels[ch].amplitude = amp;
    channels[ch].volume = vol;
    if (dur_ms > 0) {
        channels[ch].total_samples = (dur_ms * g_sample_rate) / 1000;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(warpig_audio_play_obj, 2, 6, warpig_audio_play);

// ============================================================================
// sweep(ch, freq_start, freq_end, duration=200, volume=80)
// ============================================================================

static mp_obj_t warpig_audio_sweep(size_t n_args, const mp_obj_t *args) {
    int ch = mp_obj_get_int(args[0]);
    if (ch < 0 || ch >= g_num_channels) return mp_const_none;

    float f_start = mp_obj_get_float(args[1]);
    float f_end = mp_obj_get_float(args[2]);
    uint32_t dur_ms = (n_args > 3) ? mp_obj_get_int(args[3]) : 200;
    uint8_t vol = (n_args > 4) ? mp_obj_get_int(args[4]) : 80;

    reset_channel(ch);
    channels[ch].enabled = 1;
    channels[ch].wave_type = WAVE_SQUARE;
    channels[ch].frequency = f_start;
    channels[ch].freq_start = f_start;
    channels[ch].freq_end = f_end;
    channels[ch].amplitude = 12000;
    channels[ch].volume = vol;
    channels[ch].total_samples = (dur_ms * g_sample_rate) / 1000;
    channels[ch].sweep_samples = channels[ch].total_samples;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(warpig_audio_sweep_obj, 3, 5, warpig_audio_sweep);

// ============================================================================
// explosion(ch, duration=1800, volume=90)
// ============================================================================

static mp_obj_t warpig_audio_explosion(size_t n_args, const mp_obj_t *args) {
    int ch = mp_obj_get_int(args[0]);
    if (ch < 0 || ch >= g_num_channels) return mp_const_none;

    uint32_t dur_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 1800;
    uint8_t vol = (n_args > 2) ? mp_obj_get_int(args[2]) : 90;

    reset_channel(ch);
    channels[ch].enabled = 1;
    channels[ch].wave_type = WAVE_EXPLOSION;
    channels[ch].amplitude = 16000;
    channels[ch].volume = vol;
    channels[ch].total_samples = (dur_ms * g_sample_rate) / 1000;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(warpig_audio_explosion_obj, 1, 3, warpig_audio_explosion);

// ============================================================================
// stop(ch) or stop() to stop all
// ============================================================================

static mp_obj_t warpig_audio_stop(size_t n_args, const mp_obj_t *args) {
    if (n_args > 0) {
        int ch = mp_obj_get_int(args[0]);
        if (ch >= 0 && ch < g_num_channels) {
            channels[ch].enabled = 0;
        }
    } else {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            channels[i].enabled = 0;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(warpig_audio_stop_obj, 0, 1, warpig_audio_stop);

// ============================================================================
// fill(buf) — fill a bytearray with mixed int16 samples
// Returns number of samples written. buf length / 2 = max samples.
// This is the hot loop — called from a timer or audio output task.
// ============================================================================

static mp_obj_t warpig_audio_fill(mp_obj_t buf_obj) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_obj, &bufinfo, MP_BUFFER_WRITE);
    int16_t *out = (int16_t *)bufinfo.buf;
    int n_samples = bufinfo.len / 2;

    for (int i = 0; i < n_samples; i++) {
        out[i] = mix_sample();
    }

    return mp_obj_new_int(n_samples);
}
static MP_DEFINE_CONST_FUN_OBJ_1(warpig_audio_fill_obj, warpig_audio_fill);

// ============================================================================
// volume(master_vol) — set master volume 0-100
// ============================================================================

static mp_obj_t warpig_audio_volume(mp_obj_t vol_obj) {
    g_master_volume = mp_obj_get_int(vol_obj);
    if (g_master_volume > 100) g_master_volume = 100;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(warpig_audio_volume_obj, warpig_audio_volume);

// ============================================================================
// active() — returns number of active channels
// ============================================================================

static mp_obj_t warpig_audio_active(void) {
    int count = 0;
    for (int i = 0; i < g_num_channels; i++) {
        if (channels[i].enabled) count++;
    }
    return mp_obj_new_int(count);
}
static MP_DEFINE_CONST_FUN_OBJ_0(warpig_audio_active_obj, warpig_audio_active);

// ============================================================================
// MODULE DEFINITION
// ============================================================================

static const mp_rom_map_elem_t warpig_audio_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_warpig_audio) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&warpig_audio_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_play),        MP_ROM_PTR(&warpig_audio_play_obj) },
    { MP_ROM_QSTR(MP_QSTR_sweep),       MP_ROM_PTR(&warpig_audio_sweep_obj) },
    { MP_ROM_QSTR(MP_QSTR_explosion),   MP_ROM_PTR(&warpig_audio_explosion_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&warpig_audio_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill),        MP_ROM_PTR(&warpig_audio_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_volume),      MP_ROM_PTR(&warpig_audio_volume_obj) },
    { MP_ROM_QSTR(MP_QSTR_active),      MP_ROM_PTR(&warpig_audio_active_obj) },
    // Waveform type constants
    { MP_ROM_QSTR(MP_QSTR_SINE),        MP_ROM_INT(WAVE_SINE) },
    { MP_ROM_QSTR(MP_QSTR_SQUARE),      MP_ROM_INT(WAVE_SQUARE) },
    { MP_ROM_QSTR(MP_QSTR_SAWTOOTH),    MP_ROM_INT(WAVE_SAWTOOTH) },
    { MP_ROM_QSTR(MP_QSTR_TRIANGLE),    MP_ROM_INT(WAVE_TRIANGLE) },
    { MP_ROM_QSTR(MP_QSTR_NOISE),       MP_ROM_INT(WAVE_NOISE) },
    { MP_ROM_QSTR(MP_QSTR_EXPLOSION),   MP_ROM_INT(WAVE_EXPLOSION) },
};
static MP_DEFINE_CONST_DICT(warpig_audio_globals, warpig_audio_globals_table);

const mp_obj_module_t warpig_audio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&warpig_audio_globals,
};

MP_REGISTER_MODULE(MP_QSTR_warpig_audio, warpig_audio_module);
