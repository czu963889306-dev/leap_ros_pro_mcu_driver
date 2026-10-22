#ifndef BATTERY_ADC_H_
#define BATTERY_ADC_H_

#include <stdint.h>

#include "esp_err.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

struct BatteryAdcReading {
  float adc_voltage_v = 0.0f;
  float battery_voltage_v = 0.0f;
  uint8_t percentage = 0;
  int raw = 0;
  bool valid = false;
};

class BatteryAdc {
 public:
  BatteryAdc(adc_unit_t unit, adc_channel_t channel, float divider_ratio,
             float empty_voltage_v, float full_voltage_v);

  esp_err_t Init();
  BatteryAdcReading Read();

 private:
  float EstimatePercentage(float battery_voltage_v) const;

  adc_unit_t unit_;
  adc_channel_t channel_;
  float divider_ratio_;
  float empty_voltage_v_;
  float full_voltage_v_;
  adc_oneshot_unit_handle_t unit_handle_;
  adc_cali_handle_t cali_handle_;
  bool initialized_;
};

#endif  // BATTERY_ADC_H_
