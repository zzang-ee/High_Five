#pragma once

#include <Arduino.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Config.h"

class EmgSensor {
public:
    EmgSensor();

    bool begin();

    // Copies the newest 10 ms feature snapshot into the loop-owned state.
    // Raw ADC acquisition is performed independently by the sampling task.
    bool update(uint32_t nowMs);
    void resetSignalWindow();

    bool isWindowReady() const { return windowReady; }
    bool isSignalQualityGood() const { return signalQualityGood; }
    bool isSamplingHealthy(uint32_t nowMs) const;
    bool consumeSamplingGap();
    bool consumeFeatureGap();
    uint32_t getMissedRawSampleCount() const;
    bool hasValidBatteryReading() const;
    uint8_t getBatteryPercent() const;
    float getBatteryVoltage() const;

    float getExtensorRMS() const { return extensorRMS; }
    float getFlexorRMS() const { return flexorRMS; }

    bool setRestThresholds(float extensorThresholdValue,
                           float flexorThresholdValue);
    void setExtensorMax(float value);
    void setFlexorMax(float value);
    void clearCalibration();

    bool isExtensorActive() const;
    bool isFlexorActive() const;
    float getExtensorActivation() const;
    float getFlexorActivation() const;

    float getExtensorThreshold() const { return extensorThreshold; }
    float getFlexorThreshold() const { return flexorThreshold; }
    float getExtensorMax() const { return extensorMax; }
    float getFlexorMax() const { return flexorMax; }
    float getExtensorMVC() const { return extensorMax; }
    float getFlexorMVC() const { return flexorMax; }
    bool hasValidCalibration() const;

private:
    static constexpr uint16_t ADC_MAX_VALUE = 4095;

    struct BiquadCoefficients {
        float b0;
        float b1;
        float b2;
        float a1;
        float a2;
    };

    struct BiquadState {
        float x1;
        float x2;
        float y1;
        float y2;
    };

    struct ChannelFilterState {
        BiquadState highPass;
        BiquadState notch;
        BiquadState lowPass1;
        BiquadState lowPass2;
        bool primed;
    };

    // Loop-owned feature and calibration state.
    float extensorThreshold;
    float flexorThreshold;
    float extensorMax;
    float flexorMax;
    float extensorRMS;
    float flexorRMS;
    bool windowReady;
    bool signalQualityGood;
    uint32_t lastConsumedSequence;
    bool featureGapDetected;

    // Sampling task/timer lifecycle.
    TaskHandle_t samplingTaskHandle;
    esp_timer_handle_t samplingTimer;
    bool initialized;

    // Sampling-task-owned DSP state.
    BiquadCoefficients highPassCoefficients;
    BiquadCoefficients notchCoefficients;
    BiquadCoefficients lowPassCoefficients1;
    BiquadCoefficients lowPassCoefficients2;
    ChannelFilterState extensorFilter;
    ChannelFilterState flexorFilter;
    float extensorSquaredWindow[EMG_RMS_SAMPLE_COUNT];
    float flexorSquaredWindow[EMG_RMS_SAMPLE_COUNT];
    uint8_t extensorSaturationWindow[EMG_RMS_SAMPLE_COUNT];
    uint8_t flexorSaturationWindow[EMG_RMS_SAMPLE_COUNT];
    double extensorSquaredSum;
    double flexorSquaredSum;
    uint16_t extensorSaturationCount;
    uint16_t flexorSaturationCount;
    uint16_t windowIndex;
    uint16_t validSampleCount;
    uint16_t samplesSinceFeature;
    uint64_t lastRawSampleTimeUs;
    esp_adc_cal_characteristics_t batteryAdcCharacteristics;
    uint64_t lastBatterySampleTimeUs;
    uint32_t batteryRawSum;
    uint8_t batteryRawSampleCount;
    float filteredBatteryVoltage;
    bool batteryFilterInitialized;

    // Cross-core snapshot and commands. Every access is protected by dataMux.
    mutable portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
    bool resetRequested;
    bool samplingGapLatched;
    uint32_t missedRawSamples;
    float publishedExtensorRMS;
    float publishedFlexorRMS;
    bool publishedWindowReady;
    bool publishedSignalQualityGood;
    uint32_t publishedSequence;
    uint32_t publishedLastFeatureTimeMs;
    float publishedBatteryVoltage;
    uint8_t publishedBatteryPercent;
    bool publishedBatteryValid;

    static void samplingTimerCallback(void* argument);
    static void samplingTaskEntry(void* argument);
    void samplingLoop();
    void processRawPair(int extensorRaw, int flexorRaw);
    void sampleBattery(uint64_t nowUs);
    void resetProcessingState();
    void publishFeature();

    float filterSample(float rawValue, ChannelFilterState& state) const;
    static float applyBiquad(float input,
                             const BiquadCoefficients& coefficients,
                             BiquadState& state);
    static BiquadCoefficients makeHighPass(float cutoffHz, float q);
    static BiquadCoefficients makeLowPass(float cutoffHz, float q);
    static BiquadCoefficients makeNotch(float frequencyHz, float q);
    static BiquadCoefficients normalizeBiquad(float b0,
                                               float b1,
                                               float b2,
                                               float a0,
                                               float a1,
                                               float a2);
    static bool isSaturated(int rawValue);
    static uint8_t calculateBatteryPercent(float packVoltage);
    static float normalizeActivation(float rms, float threshold, float maximum);
    static bool isValidCalibrationValue(float value);
};
