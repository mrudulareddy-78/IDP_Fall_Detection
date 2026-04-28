// ============================================================
// inference.h — TFLite Micro Inference
// Quantization values from Colab Cell 23 output:
//   Input : scale=0.228922  zp=-44
//   Output: scale=0.003906  zp=-128
// ============================================================
#pragma once
#include "model.h"

#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/schema/schema_generated.h>

#define WIN      300
#define N_CH     8

// ── Quantization — from Colab Cell 23 ────────────────────────
#define INP_SCALE   0.228922f
#define INP_ZP      (-44)
#define OUT_SCALE   0.003906f
#define OUT_ZP      (-128)
#define THRESHOLD   0.57f

// ── Tensor arena ─────────────────────────────────────────────
// ESP32-C3 has ~400KB RAM. 120KB is safe starting point.
// If AllocateTensors() fails → increase to 150
// If compilation says not enough RAM → decrease to 100
#define ARENA_KB  120
static uint8_t tensor_arena[ARENA_KB * 1024];

// ── TFLite objects ────────────────────────────────────────────
static tflite::MicroErrorReporter error_reporter;
static tflite::AllOpsResolver     resolver;
static const tflite::Model*       tf_model   = nullptr;
static tflite::MicroInterpreter*  interpreter = nullptr;
static TfLiteTensor*              inp_t       = nullptr;
static TfLiteTensor*              out_t       = nullptr;

// ── Init — call in setup() ───────────────────────────────────
static bool inference_init() {
    // Variable name from model.h first line:
    // unsigned char _content_fall_model_int8_tflite[] = {...
    tf_model = tflite::GetModel(_content_fall_model_int8_tflite);

    if (tf_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("  ❌ TFLite schema version mismatch");
        return false;
    }

    static tflite::MicroInterpreter static_interp(
        tf_model, resolver,
        tensor_arena, ARENA_KB * 1024,
        &error_reporter
    );
    interpreter = &static_interp;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("  ❌ AllocateTensors failed");
        Serial.println("     Increase ARENA_KB in inference.h");
        return false;
    }

    inp_t = interpreter->input(0);
    out_t = interpreter->output(0);

    Serial.printf("  Input  shape: [%d,%d,%d] type=%d\n",
        inp_t->dims->data[0],
        inp_t->dims->data[1],
        inp_t->dims->data[2],
        inp_t->type);  // 9 = int8 ✅

    return true;
}

// ── Run one inference ─────────────────────────────────────────
// Input : preprocessed float window[300][8]
// Output: fall probability 0.0–1.0
static float run_inference(float window[WIN][N_CH]) {
    int8_t* inp = inp_t->data.int8;

    // Quantize float32 → int8
    // int8 = clamp(round(float / scale + zero_point), -128, 127)
    for (int t = 0; t < WIN; t++) {
        for (int ch = 0; ch < N_CH; ch++) {
            float f  = window[t][ch];
            int   qi = (int)roundf(f / INP_SCALE) + INP_ZP;
            if (qi < -128) qi = -128;
            if (qi >  127) qi =  127;
            inp[t * N_CH + ch] = (int8_t)qi;
        }
    }

    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("  ⚠ Invoke failed");
        return 0.0f;
    }

    // Dequantize int8 → float
    // float = (int8 - zero_point) * scale
    int8_t raw  = out_t->data.int8[0];
    float  prob = (float)(raw - OUT_ZP) * OUT_SCALE;
    return prob;
}
