#include "batt.h"
#include "agv_config.h"
#include "adc.h"

uint32_t Batt_ReadMv(void)
{
  uint32_t sum = 0, n = 0;
  for (uint32_t i = 0; i < BATT_SAMPLES; i++) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
      sum += HAL_ADC_GetValue(&hadc1);
      n++;
    }
  }
  HAL_ADC_Stop(&hadc1);
  if (n == 0) return 0;
  uint32_t mv = (sum / n) * BATT_VREF_MV * BATT_DIV_NUM / (4095U * BATT_DIV_DEN);
  return mv + BATT_DIODE_MV;
}
