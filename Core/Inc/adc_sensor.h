#ifndef _ADC_H
#define _ADC_H

enum sensor_type {PH_sensor = 1, ORP_sensor = 2};

uint16_t adc1_get_raw(char sensor);
uint16_t adc_read_sensor_voltage(char sensor);

int read_ph_sensor(uint16_t ph_voltage, uint16_t Voltage_686, uint16_t Voltage_401);
int read_orp_sensor(uint16_t orp_voltage);
#endif
