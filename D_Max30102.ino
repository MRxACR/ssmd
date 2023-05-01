#include <MAX3010x.h>
#include "filters.h"

// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

void decOxy() {

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    Serial.println("{\"log\" : \"Oxymètre : Sensor initialized\" }");
  }
  else {
    Serial.println("{\"log\" : \"Oxymètre : Sensor not found\" }");
  }
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void TaskOxy() {

  sensor.begin();
  sensor.setSamplingRate(kSamplingRate);

  while (1) {

    auto sample = sensor.readSample(1000);
    float current_value_red = sample.red;
    float current_value_ir = sample.ir;

    if (sample.red > kFingerThreshold) {
      finger_detected = true;

    }
    else {
      finger_detected = false;
    }


    if (finger_detected && act_oxy) {

      current_value_red = low_pass_filter_red.process(current_value_red);
      current_value_ir = low_pass_filter_ir.process(current_value_ir);
      stat_red.process(current_value_red);
      stat_ir.process(current_value_ir);

      float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
      float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
      float r = rred / rir;
      float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

      Serial.println("{\"oxy\" : " + String( spo2 ) + "}");
      
      if (spo2 < 80) {
        message_alert = "Manque d'oxygène, SpO2 :" + String(spo2) + "%";
        alert = 1;
      }

    }
    else {
      // Reset values if the finger is removed
      differentiator.reset();
      averager_bpm.reset();
      averager_r.reset();
      averager_spo2.reset();
      low_pass_filter_red.reset();
      low_pass_filter_ir.reset();
      high_pass_filter.reset();
      stat_red.reset();
      stat_ir.reset();
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS); //  2s
  }

}
