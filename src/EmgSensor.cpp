#include "EmgSensor.h"

#include <cmath>
#include <cstring>

#include "driver/adc.h"
#include "esp_err.h"

namespace {
constexpr float PI_VALUE = 3.14159265358979323846f;
constexpr uint32_t ADC_DEFAULT_VREF_MV = 1100;
constexpr adc1_channel_t EXTENSOR_ADC_CHANNEL = ADC1_CHANNEL_6;
constexpr adc1_channel_t FLEXOR_ADC_CHANNEL = ADC1_CHANNEL_7;
constexpr adc1_channel_t BATTERY_ADC_CHANNEL = ADC1_CHANNEL_4;

constexpr float BATTERY_VOLTAGE_POINTS[] = {
    6.60f, 6.80f, 7.00f, 7.20f, 7.40f,
    7.60f, 7.80f, 8.00f, 8.20f, 8.40f};
constexpr uint8_t BATTERY_PERCENT_POINTS[] = {
    0, 5, 10, 20, 35, 50, 65, 80, 90, 100};
constexpr size_t BATTERY_POINT_COUNT =
    sizeof(BATTERY_VOLTAGE_POINTS) / sizeof(BATTERY_VOLTAGE_POINTS[0]);

static_assert(EMG_EXTENSOR_PIN == 34 && EMG_FLEXOR_PIN == 35,
              "Direct ADC1 channel mapping requires EMG pins 34 and 35");
static_assert(BATTERY_ADC_PIN == 32,
              "Direct ADC1 channel mapping requires battery pin 32");
static_assert(BATTERY_POINT_COUNT ==
                  sizeof(BATTERY_PERCENT_POINTS) /
                      sizeof(BATTERY_PERCENT_POINTS[0]),
              "Battery voltage and percentage tables must match");

uint16_t calculateAllowedSaturatedSamples(uint16_t sampleCount) {
    const float allowed =
        static_cast<float>(sampleCount) * EMG_MAX_SATURATION_FRACTION;
    return static_cast<uint16_t>(std::ceil(allowed));
}
}  // namespace

EmgSensor::EmgSensor()
    : extensorThreshold(0.0f),
      flexorThreshold(0.0f),
      extensorMax(0.0f),
      flexorMax(0.0f),
      extensorRMS(0.0f),
      flexorRMS(0.0f),
      windowReady(false),
      signalQualityGood(false),
      lastConsumedSequence(0),
      samplingTaskHandle(nullptr),
      samplingTimer(nullptr),
      initialized(false),
      highPassCoefficients{},
      notchCoefficients{},
      lowPassCoefficients1{},
      lowPassCoefficients2{},
      extensorFilter{},
      flexorFilter{},
      extensorSquaredWindow{},
      flexorSquaredWindow{},
      extensorSaturationWindow{},
      flexorSaturationWindow{},
      extensorSquaredSum(0.0),
      flexorSquaredSum(0.0),
      extensorSaturationCount(0),
      flexorSaturationCount(0),
      windowIndex(0),
      validSampleCount(0),
      samplesSinceFeature(0),
      lastRawSampleTimeUs(0),
      batteryAdcCharacteristics{},
      lastBatterySampleTimeUs(0),
      batteryRawSum(0),
      batteryRawSampleCount(0),
      filteredBatteryVoltage(0.0f),
      batteryFilterInitialized(false),
      resetRequested(false),
      samplingGapLatched(false),
      missedRawSamples(0),
      publishedExtensorRMS(0.0f),
      publishedFlexorRMS(0.0f),
      publishedWindowReady(false),
      publishedSignalQualityGood(false),
      publishedSequence(0),
      publishedLastFeatureTimeMs(0),
      publishedBatteryVoltage(0.0f),
      publishedBatteryPercent(0),
      publishedBatteryValid(false) {}

bool EmgSensor::begin() {
    if (initialized) {
        return true;
    }

    const esp_err_t widthResult = adc1_config_width(ADC_WIDTH_BIT_12);
    const esp_err_t extensorAdcResult =
        adc1_config_channel_atten(EXTENSOR_ADC_CHANNEL, ADC_ATTEN_DB_12);
    const esp_err_t flexorAdcResult =
        adc1_config_channel_atten(FLEXOR_ADC_CHANNEL, ADC_ATTEN_DB_12);
    const esp_err_t batteryAdcResult =
        adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_12);
    if (widthResult != ESP_OK || extensorAdcResult != ESP_OK ||
        flexorAdcResult != ESP_OK || batteryAdcResult != ESP_OK) {
        Serial.printf("ERR:ADC_CONFIG:%d,%d,%d,%d\n",
                      static_cast<int>(widthResult),
                      static_cast<int>(extensorAdcResult),
                      static_cast<int>(flexorAdcResult),
                      static_cast<int>(batteryAdcResult));
        return false;
    }

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12,
                             ADC_DEFAULT_VREF_MV,
                             &batteryAdcCharacteristics);

    highPassCoefficients = makeHighPass(EMG_HIGH_PASS_CUTOFF_HZ,
                                        EMG_FILTER_Q);
    notchCoefficients = makeNotch(EMG_NOTCH_FREQUENCY_HZ,
                                  EMG_NOTCH_Q);
    lowPassCoefficients1 = makeLowPass(EMG_LOW_PASS_CUTOFF_HZ,
                                       EMG_LOW_PASS_Q_1);
    lowPassCoefficients2 = makeLowPass(EMG_LOW_PASS_CUTOFF_HZ,
                                       EMG_LOW_PASS_Q_2);
    resetProcessingState();
    lastRawSampleTimeUs = 0;

    const BaseType_t taskResult = xTaskCreatePinnedToCore(
        samplingTaskEntry,
        "emg_sample",
        EMG_SAMPLING_TASK_STACK,
        this,
        EMG_SAMPLING_TASK_PRIORITY,
        &samplingTaskHandle,
        EMG_SAMPLING_TASK_CORE);
    if (taskResult != pdPASS || samplingTaskHandle == nullptr) {
        samplingTaskHandle = nullptr;
        Serial.println("ERR:EMG_TASK_CREATE");
        return false;
    }

    esp_timer_create_args_t timerArguments = {};
    timerArguments.callback = samplingTimerCallback;
    timerArguments.arg = this;
    timerArguments.dispatch_method = ESP_TIMER_TASK;
    timerArguments.name = "emg_tick";

    const esp_err_t createResult =
        esp_timer_create(&timerArguments, &samplingTimer);
    if (createResult != ESP_OK) {
        vTaskDelete(samplingTaskHandle);
        samplingTaskHandle = nullptr;
        samplingTimer = nullptr;
        Serial.printf("ERR:EMG_TIMER_CREATE:%d\n", static_cast<int>(createResult));
        return false;
    }

    const esp_err_t startResult =
        esp_timer_start_periodic(samplingTimer,
                                 EMG_RAW_SAMPLE_INTERVAL_US);
    if (startResult != ESP_OK) {
        esp_timer_delete(samplingTimer);
        samplingTimer = nullptr;
        vTaskDelete(samplingTaskHandle);
        samplingTaskHandle = nullptr;
        Serial.printf("ERR:EMG_TIMER_START:%d\n", static_cast<int>(startResult));
        return false;
    }

    initialized = true;
    Serial.printf(
        "EMG: raw=%luHz, band=%.0f-%.0fHz, notch=%.0fHz/Q%.0f, "
        "RMS=%ums, feature=%lums\n",
                  static_cast<unsigned long>(EMG_RAW_SAMPLE_RATE_HZ),
                  EMG_HIGH_PASS_CUTOFF_HZ,
                  EMG_LOW_PASS_CUTOFF_HZ,
                  EMG_NOTCH_FREQUENCY_HZ,
                  EMG_NOTCH_Q,
                  static_cast<unsigned int>(EMG_RMS_WINDOW_MS),
                  static_cast<unsigned long>(EMG_FEATURE_INTERVAL_MS));
    return true;
}

bool EmgSensor::update(uint32_t nowMs) {
    (void)nowMs;

    uint32_t sequence = 0;
    float newExtensorRMS = 0.0f;
    float newFlexorRMS = 0.0f;
    bool newWindowReady = false;
    bool newSignalQualityGood = false;

    portENTER_CRITICAL(&dataMux);
    sequence = publishedSequence;
    if (sequence != lastConsumedSequence) {
        newExtensorRMS = publishedExtensorRMS;
        newFlexorRMS = publishedFlexorRMS;
        newWindowReady = publishedWindowReady;
        newSignalQualityGood = publishedSignalQualityGood;
    }
    portEXIT_CRITICAL(&dataMux);

    if (sequence == lastConsumedSequence) {
        return false;
    }

    // The control loop only needs the newest feature. It can legitimately
    // skip intermediate 10 ms snapshots while BLE sends notifications; that
    // is not a raw ADC sampling failure because acquisition runs in its own
    // task.
    extensorRMS = newExtensorRMS;
    flexorRMS = newFlexorRMS;
    windowReady = newWindowReady;
    signalQualityGood = newSignalQualityGood;
    lastConsumedSequence = sequence;
    return true;
}

void EmgSensor::resetSignalWindow() {
    uint32_t invalidatedSequence = 0;

    portENTER_CRITICAL(&dataMux);
    resetRequested = true;
    publishedExtensorRMS = 0.0f;
    publishedFlexorRMS = 0.0f;
    publishedWindowReady = false;
    publishedSignalQualityGood = false;
    ++publishedSequence;
    invalidatedSequence = publishedSequence;
    portEXIT_CRITICAL(&dataMux);

    extensorRMS = 0.0f;
    flexorRMS = 0.0f;
    windowReady = false;
    signalQualityGood = false;
    lastConsumedSequence = invalidatedSequence;
}

bool EmgSensor::consumeSamplingGap() {
    portENTER_CRITICAL(&dataMux);
    const bool detected = samplingGapLatched;
    samplingGapLatched = false;
    portEXIT_CRITICAL(&dataMux);
    return detected;
}

bool EmgSensor::isSamplingHealthy(uint32_t nowMs) const {
    portENTER_CRITICAL(&dataMux);
    const uint32_t lastFeatureTimeMs = publishedLastFeatureTimeMs;
    portEXIT_CRITICAL(&dataMux);

    return initialized && lastFeatureTimeMs != 0 &&
           static_cast<uint32_t>(nowMs - lastFeatureTimeMs) <=
               EMG_FEATURE_HEALTH_TIMEOUT_MS;
}

uint32_t EmgSensor::getMissedRawSampleCount() const {
    portENTER_CRITICAL(&dataMux);
    const uint32_t result = missedRawSamples;
    portEXIT_CRITICAL(&dataMux);
    return result;
}

bool EmgSensor::hasValidBatteryReading() const {
    portENTER_CRITICAL(&dataMux);
    const bool result = publishedBatteryValid;
    portEXIT_CRITICAL(&dataMux);
    return result;
}

uint8_t EmgSensor::getBatteryPercent() const {
    portENTER_CRITICAL(&dataMux);
    const uint8_t result = publishedBatteryPercent;
    portEXIT_CRITICAL(&dataMux);
    return result;
}

float EmgSensor::getBatteryVoltage() const {
    portENTER_CRITICAL(&dataMux);
    const float result = publishedBatteryVoltage;
    portEXIT_CRITICAL(&dataMux);
    return result;
}

bool EmgSensor::setRestThresholds(float extensorThresholdValue,
                                  float flexorThresholdValue) {
    if (!isValidCalibrationValue(extensorThresholdValue) ||
        !isValidCalibrationValue(flexorThresholdValue)) {
        return false;
    }
    extensorThreshold = extensorThresholdValue;
    flexorThreshold = flexorThresholdValue;
    return true;
}

void EmgSensor::setExtensorMax(float value) {
    extensorMax = isValidCalibrationValue(value) ? value : 0.0f;
}

void EmgSensor::setFlexorMax(float value) {
    flexorMax = isValidCalibrationValue(value) ? value : 0.0f;
}

void EmgSensor::clearCalibration() {
    extensorThreshold = 0.0f;
    flexorThreshold = 0.0f;
    extensorMax = 0.0f;
    flexorMax = 0.0f;
}

bool EmgSensor::hasValidCalibration() const {
    return extensorThreshold > 0.0f && flexorThreshold > 0.0f &&
           isValidCalibrationValue(extensorThreshold) &&
           isValidCalibrationValue(flexorThreshold) &&
           isValidCalibrationValue(extensorMax) &&
           isValidCalibrationValue(flexorMax) &&
           extensorMax - extensorThreshold >=
               fmaxf(EMG_EXTENSOR_MIN_CALIBRATION_SPAN,
                     extensorThreshold * EMG_MIN_CALIBRATION_SPAN_RATIO) &&
           flexorMax - flexorThreshold >=
               fmaxf(EMG_FLEXOR_MIN_CALIBRATION_SPAN,
                     flexorThreshold * EMG_MIN_CALIBRATION_SPAN_RATIO);
}

bool EmgSensor::isExtensorActive() const {
    return windowReady && signalQualityGood && extensorThreshold > 0.0f &&
           extensorRMS >= extensorThreshold;
}

bool EmgSensor::isFlexorActive() const {
    return windowReady && signalQualityGood && flexorThreshold > 0.0f &&
           flexorRMS >= flexorThreshold;
}

float EmgSensor::getExtensorActivation() const {
    if (!windowReady || !signalQualityGood) {
        return 0.0f;
    }
    return normalizeActivation(extensorRMS, extensorThreshold, extensorMax,
                               EMG_EXTENSOR_MIN_CALIBRATION_SPAN);
}

float EmgSensor::getFlexorActivation() const {
    if (!windowReady || !signalQualityGood) {
        return 0.0f;
    }
    return normalizeActivation(flexorRMS, flexorThreshold, flexorMax,
                               EMG_FLEXOR_MIN_CALIBRATION_SPAN);
}

void EmgSensor::samplingTimerCallback(void* argument) {
    auto* sensor = static_cast<EmgSensor*>(argument);
    if (sensor != nullptr && sensor->samplingTaskHandle != nullptr) {
        xTaskNotifyGive(sensor->samplingTaskHandle);
    }
}

void EmgSensor::samplingTaskEntry(void* argument) {
    auto* sensor = static_cast<EmgSensor*>(argument);
    if (sensor == nullptr) {
        vTaskDelete(nullptr);
        return;
    }
    sensor->samplingLoop();
}

void EmgSensor::samplingLoop() {
    for (;;) {
        const uint32_t notifications =
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());

        bool requestedReset = false;
        portENTER_CRITICAL(&dataMux);
        requestedReset = resetRequested;
        resetRequested = false;
        portEXIT_CRITICAL(&dataMux);

        uint32_t missedThisCycle = notifications > 1 ? notifications - 1 : 0;
        bool timingGap = false;
        if (lastRawSampleTimeUs != 0) {
            const uint64_t elapsedUs = nowUs - lastRawSampleTimeUs;
            if (elapsedUs > EMG_RAW_SAMPLE_INTERVAL_US) {
                const uint64_t elapsedIntervals =
                    elapsedUs / EMG_RAW_SAMPLE_INTERVAL_US;
                const uint32_t missedFromTime =
                    elapsedIntervals > 1
                        ? static_cast<uint32_t>(elapsedIntervals - 1)
                        : 0;
                if (missedFromTime > missedThisCycle) {
                    missedThisCycle = missedFromTime;
                }
            }
            timingGap = elapsedUs > EMG_RAW_GAP_LIMIT_US;
        }
        // Every notification represents one acquisition instant. If more
        // than one accumulated, at least one raw pair was not measured and
        // the rolling RMS must not bridge that discontinuity.
        timingGap = timingGap || missedThisCycle > 0;
        lastRawSampleTimeUs = nowUs;

        if (requestedReset || timingGap) {
            resetProcessingState();
        }

        if (missedThisCycle > 0 || timingGap) {
            portENTER_CRITICAL(&dataMux);
            missedRawSamples += missedThisCycle;
            if (timingGap) {
                samplingGapLatched = true;
                publishedExtensorRMS = 0.0f;
                publishedFlexorRMS = 0.0f;
                publishedWindowReady = false;
                publishedSignalQualityGood = false;
                ++publishedSequence;
            }
            portEXIT_CRITICAL(&dataMux);
        }

        processRawPair(adc1_get_raw(EXTENSOR_ADC_CHANNEL),
                       adc1_get_raw(FLEXOR_ADC_CHANNEL));
        // GPIO32 is read by this same task so battery monitoring can never
        // race the 2 kHz EMG accesses to the shared ADC1 peripheral.
        sampleBattery(nowUs);
    }
}

void EmgSensor::processRawPair(int extensorRaw, int flexorRaw) {
    if (extensorRaw < 0 || extensorRaw > ADC_MAX_VALUE || flexorRaw < 0 ||
        flexorRaw > ADC_MAX_VALUE) {
        resetProcessingState();
        portENTER_CRITICAL(&dataMux);
        samplingGapLatched = true;
        ++missedRawSamples;
        publishedExtensorRMS = 0.0f;
        publishedFlexorRMS = 0.0f;
        publishedWindowReady = false;
        publishedSignalQualityGood = false;
        ++publishedSequence;
        portEXIT_CRITICAL(&dataMux);
        return;
    }

    const float filteredExtensor =
        filterSample(static_cast<float>(extensorRaw), extensorFilter);
    const float filteredFlexor =
        filterSample(static_cast<float>(flexorRaw), flexorFilter);

    const float extensorSquared = filteredExtensor * filteredExtensor;
    const float flexorSquared = filteredFlexor * filteredFlexor;
    const uint8_t extensorSaturated = isSaturated(extensorRaw) ? 1U : 0U;
    const uint8_t flexorSaturated = isSaturated(flexorRaw) ? 1U : 0U;

    if (validSampleCount == EMG_RMS_SAMPLE_COUNT) {
        extensorSquaredSum -= extensorSquaredWindow[windowIndex];
        flexorSquaredSum -= flexorSquaredWindow[windowIndex];
        extensorSaturationCount -= extensorSaturationWindow[windowIndex];
        flexorSaturationCount -= flexorSaturationWindow[windowIndex];
    } else {
        ++validSampleCount;
    }

    extensorSquaredWindow[windowIndex] = extensorSquared;
    flexorSquaredWindow[windowIndex] = flexorSquared;
    extensorSaturationWindow[windowIndex] = extensorSaturated;
    flexorSaturationWindow[windowIndex] = flexorSaturated;
    extensorSquaredSum += extensorSquared;
    flexorSquaredSum += flexorSquared;
    extensorSaturationCount += extensorSaturated;
    flexorSaturationCount += flexorSaturated;

    windowIndex = static_cast<uint16_t>(
        (windowIndex + 1U) % EMG_RMS_SAMPLE_COUNT);

    constexpr uint16_t samplesPerFeature = static_cast<uint16_t>(
        (EMG_RAW_SAMPLE_RATE_HZ * EMG_FEATURE_INTERVAL_MS) /
        1000UL);
    ++samplesSinceFeature;
    if (samplesSinceFeature >= samplesPerFeature) {
        samplesSinceFeature = 0;
        publishFeature();
    }
}

void EmgSensor::sampleBattery(uint64_t nowUs) {
    if (lastBatterySampleTimeUs != 0 &&
        nowUs - lastBatterySampleTimeUs < BATTERY_SUBSAMPLE_INTERVAL_US) {
        return;
    }
    lastBatterySampleTimeUs = nowUs;

    const int raw = adc1_get_raw(BATTERY_ADC_CHANNEL);
    if (raw < 0 || raw > ADC_MAX_VALUE) {
        batteryRawSum = 0;
        batteryRawSampleCount = 0;
        batteryFilterInitialized = false;
        portENTER_CRITICAL(&dataMux);
        publishedBatteryValid = false;
        publishedBatteryVoltage = 0.0f;
        publishedBatteryPercent = 0;
        portEXIT_CRITICAL(&dataMux);
        return;
    }

    batteryRawSum += static_cast<uint32_t>(raw);
    ++batteryRawSampleCount;
    if (batteryRawSampleCount < BATTERY_AVERAGE_SAMPLE_COUNT) {
        return;
    }

    const uint32_t averagedRaw =
        batteryRawSum / static_cast<uint32_t>(batteryRawSampleCount);
    batteryRawSum = 0;
    batteryRawSampleCount = 0;

    const uint32_t dividerMillivolts =
        esp_adc_cal_raw_to_voltage(averagedRaw, &batteryAdcCharacteristics);
    const float dividerRatio =
        (BATTERY_DIVIDER_TOP_OHMS + BATTERY_DIVIDER_BOTTOM_OHMS) /
        BATTERY_DIVIDER_BOTTOM_OHMS;
    const float measuredPackVoltage =
        (static_cast<float>(dividerMillivolts) / 1000.0f) * dividerRatio;
    const bool valid =
        std::isfinite(measuredPackVoltage) &&
        measuredPackVoltage >= BATTERY_VALID_MIN_VOLTAGE &&
        measuredPackVoltage <= BATTERY_VALID_MAX_VOLTAGE;

    if (!valid) {
        batteryFilterInitialized = false;
        portENTER_CRITICAL(&dataMux);
        publishedBatteryValid = false;
        publishedBatteryVoltage = 0.0f;
        publishedBatteryPercent = 0;
        portEXIT_CRITICAL(&dataMux);
        return;
    }

    if (!batteryFilterInitialized) {
        filteredBatteryVoltage = measuredPackVoltage;
        batteryFilterInitialized = true;
    } else {
        filteredBatteryVoltage =
            BATTERY_FILTER_ALPHA * measuredPackVoltage +
            (1.0f - BATTERY_FILTER_ALPHA) * filteredBatteryVoltage;
    }

    const uint8_t percent = calculateBatteryPercent(filteredBatteryVoltage);
    portENTER_CRITICAL(&dataMux);
    publishedBatteryVoltage = filteredBatteryVoltage;
    publishedBatteryPercent = percent;
    publishedBatteryValid = true;
    portEXIT_CRITICAL(&dataMux);
}

void EmgSensor::resetProcessingState() {
    extensorFilter = {};
    flexorFilter = {};
    std::memset(extensorSquaredWindow, 0, sizeof(extensorSquaredWindow));
    std::memset(flexorSquaredWindow, 0, sizeof(flexorSquaredWindow));
    std::memset(extensorSaturationWindow,
                0,
                sizeof(extensorSaturationWindow));
    std::memset(flexorSaturationWindow,
                0,
                sizeof(flexorSaturationWindow));
    extensorSquaredSum = 0.0;
    flexorSquaredSum = 0.0;
    extensorSaturationCount = 0;
    flexorSaturationCount = 0;
    windowIndex = 0;
    validSampleCount = 0;
    samplesSinceFeature = 0;
}

void EmgSensor::publishFeature() {
    if (validSampleCount == 0) {
        return;
    }

    const bool ready = validSampleCount == EMG_RMS_SAMPLE_COUNT;
    const float sampleDivisor = static_cast<float>(validSampleCount);
    const float newExtensorRMS = static_cast<float>(
        std::sqrt(fmax(0.0, extensorSquaredSum / sampleDivisor)));
    const float newFlexorRMS = static_cast<float>(
        std::sqrt(fmax(0.0, flexorSquaredSum / sampleDivisor)));
    const uint16_t allowedSaturatedSamples =
        calculateAllowedSaturatedSamples(validSampleCount);
    const bool qualityGood =
        ready && extensorSaturationCount <= allowedSaturatedSamples &&
        flexorSaturationCount <= allowedSaturatedSamples &&
        std::isfinite(newExtensorRMS) && std::isfinite(newFlexorRMS);
    const uint32_t featureTimeMs =
        static_cast<uint32_t>(esp_timer_get_time() / 1000LL);

    portENTER_CRITICAL(&dataMux);
    // resetSignalWindow() may have invalidated the current task-side window.
    // Do not publish it before the task consumes that reset request.
    if (!resetRequested) {
        publishedExtensorRMS = newExtensorRMS;
        publishedFlexorRMS = newFlexorRMS;
        publishedWindowReady = ready;
        publishedSignalQualityGood = qualityGood;
        publishedLastFeatureTimeMs = featureTimeMs;
        ++publishedSequence;
    }
    portEXIT_CRITICAL(&dataMux);
}

float EmgSensor::filterSample(float rawValue,
                              ChannelFilterState& state) const {
    if (!state.primed) {
        // Start the high-pass section at the present ADC bias so the nominal
        // 1.5 V sensor offset does not create a false contraction transient.
        state.highPass.x1 = rawValue;
        state.highPass.x2 = rawValue;
        state.highPass.y1 = 0.0f;
        state.highPass.y2 = 0.0f;
        state.primed = true;
        return 0.0f;
    }

    const float highPassed =
        applyBiquad(rawValue, highPassCoefficients, state.highPass);
    const float notched =
        applyBiquad(highPassed, notchCoefficients, state.notch);
    const float lowPassed =
        applyBiquad(notched, lowPassCoefficients1, state.lowPass1);
    return applyBiquad(lowPassed, lowPassCoefficients2, state.lowPass2);
}

float EmgSensor::applyBiquad(float input,
                             const BiquadCoefficients& coefficients,
                             BiquadState& state) {
    const float output = coefficients.b0 * input +
                         coefficients.b1 * state.x1 +
                         coefficients.b2 * state.x2 -
                         coefficients.a1 * state.y1 -
                         coefficients.a2 * state.y2;
    state.x2 = state.x1;
    state.x1 = input;
    state.y2 = state.y1;
    state.y1 = output;
    return output;
}

EmgSensor::BiquadCoefficients EmgSensor::makeHighPass(float cutoffHz,
                                                       float q) {
    const float omega =
        2.0f * PI_VALUE * cutoffHz / EMG_RAW_SAMPLE_RATE_HZ;
    const float cosine = std::cos(omega);
    const float alpha = std::sin(omega) / (2.0f * q);
    const float b0 = (1.0f + cosine) * 0.5f;
    const float b1 = -(1.0f + cosine);
    const float b2 = b0;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cosine;
    const float a2 = 1.0f - alpha;
    return normalizeBiquad(b0, b1, b2, a0, a1, a2);
}

EmgSensor::BiquadCoefficients EmgSensor::makeLowPass(float cutoffHz,
                                                      float q) {
    const float omega =
        2.0f * PI_VALUE * cutoffHz / EMG_RAW_SAMPLE_RATE_HZ;
    const float cosine = std::cos(omega);
    const float alpha = std::sin(omega) / (2.0f * q);
    const float b0 = (1.0f - cosine) * 0.5f;
    const float b1 = 1.0f - cosine;
    const float b2 = b0;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cosine;
    const float a2 = 1.0f - alpha;
    return normalizeBiquad(b0, b1, b2, a0, a1, a2);
}

EmgSensor::BiquadCoefficients EmgSensor::makeNotch(float frequencyHz,
                                                    float q) {
    const float omega =
        2.0f * PI_VALUE * frequencyHz / EMG_RAW_SAMPLE_RATE_HZ;
    const float cosine = std::cos(omega);
    const float alpha = std::sin(omega) / (2.0f * q);
    const float b0 = 1.0f;
    const float b1 = -2.0f * cosine;
    const float b2 = 1.0f;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cosine;
    const float a2 = 1.0f - alpha;
    return normalizeBiquad(b0, b1, b2, a0, a1, a2);
}

EmgSensor::BiquadCoefficients EmgSensor::normalizeBiquad(float b0,
                                                         float b1,
                                                         float b2,
                                                         float a0,
                                                         float a1,
                                                         float a2) {
    return {b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0};
}

bool EmgSensor::isSaturated(int rawValue) {
    return rawValue <= EMG_ADC_RAIL_MARGIN_COUNTS ||
           rawValue >=
               static_cast<int>(ADC_MAX_VALUE -
                                EMG_ADC_RAIL_MARGIN_COUNTS);
}

uint8_t EmgSensor::calculateBatteryPercent(float packVoltage) {
    if (!std::isfinite(packVoltage) ||
        packVoltage <= BATTERY_VOLTAGE_POINTS[0]) {
        return 0;
    }
    if (packVoltage >= BATTERY_VOLTAGE_POINTS[BATTERY_POINT_COUNT - 1U]) {
        return 100;
    }

    for (size_t i = 1; i < BATTERY_POINT_COUNT; ++i) {
        if (packVoltage <= BATTERY_VOLTAGE_POINTS[i]) {
            const float lowerVoltage = BATTERY_VOLTAGE_POINTS[i - 1U];
            const float upperVoltage = BATTERY_VOLTAGE_POINTS[i];
            const float fraction =
                (packVoltage - lowerVoltage) /
                (upperVoltage - lowerVoltage);
            const float lowerPercent = BATTERY_PERCENT_POINTS[i - 1U];
            const float upperPercent = BATTERY_PERCENT_POINTS[i];
            return static_cast<uint8_t>(lroundf(
                lowerPercent + fraction * (upperPercent - lowerPercent)));
        }
    }
    return 100;
}

float EmgSensor::normalizeActivation(float rms,
                                     float threshold,
                                     float maximum,
                                     float minimumSpan) {
    const float span = maximum - threshold;
    const float requiredSpan =
        fmaxf(minimumSpan,
              threshold * EMG_MIN_CALIBRATION_SPAN_RATIO);
    if (!std::isfinite(rms) || !std::isfinite(span) ||
        span < requiredSpan) {
        return 0.0f;
    }
    return constrain((rms - threshold) / span, 0.0f, 1.0f);
}

bool EmgSensor::isValidCalibrationValue(float value) {
    return std::isfinite(value) && value >= 0.0f &&
           value <= static_cast<float>(ADC_MAX_VALUE);
}
