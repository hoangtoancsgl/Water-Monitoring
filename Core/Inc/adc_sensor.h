#ifndef _ADC_H
#define _ADC_H

uint16_t adc1_get_raw(char sensor);
uint16_t adc_read_ph_sensor_voltage();

int read_ph_sensor(uint16_t ph_voltage, uint16_t Voltage_686, uint16_t Voltage_401);

#endif
