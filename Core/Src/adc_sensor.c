#include "main.h"
#include "adc_sensor.h"

#define NO_OF_SAMPLES 32 

ADC_HandleTypeDef hadc1;

static uint16_t adc_buff[NO_OF_SAMPLES];
static uint16_t adc_buff1[NO_OF_SAMPLES];

static uint16_t getMedianNum(uint16_t bArray[], int iFilterLen) 
{
    uint16_t bTab[iFilterLen];
    for (int i = 0; i<iFilterLen; i++) bTab[i] = bArray[i];

    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) 
    {
        for (i = 0; i < iFilterLen - j - 1; i++) 
        {
            if (bTab[i] > bTab[i + 1]) 
            {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0) bTemp = bTab[(iFilterLen - 1) / 2];
    else bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

static float adc_cal_raw_to_voltage(int voltage_value)
{
    return (float)voltage_value/4.095*3.3;
}


uint16_t adc1_get_raw(char sensor)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t tem_val=0;
    if(sensor == PH_sensor)
    {
        sConfig.Channel = ADC_CHANNEL_4;
        sConfig.Rank = ADC_REGULAR_RANK_2;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        tem_val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return tem_val;
    }
    else if(sensor == ORP_sensor)
    {
        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank = ADC_REGULAR_RANK_2;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        tem_val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return tem_val;
    }
}

uint16_t adc_read_sensor_voltage(char sensor)
{
    uint32_t sensor_voltage = 0;
    //Multisampling
    for (int j = 0; j < NO_OF_SAMPLES; j++) 
    {
        for (int i = 0; i < NO_OF_SAMPLES; i++) 
        {
            adc_buff[i]= adc1_get_raw(sensor);
        }
        vTaskDelay(1);
        adc_buff1[j] = getMedianNum(adc_buff, NO_OF_SAMPLES);
    }
        
    sensor_voltage = getMedianNum(adc_buff1, NO_OF_SAMPLES);
    // Convert ADC value to voltage in mV
    return adc_cal_raw_to_voltage(sensor_voltage);
}

int read_ph_sensor(uint16_t ph_voltage, uint16_t Voltage_686, uint16_t Voltage_401)
{
    float slope = (6.86-4.01)/((Voltage_686-1500.0)/3.0 - (Voltage_401-1500.0)/3.0);  // two point: (_neutralVoltage,6.86),(_acidVoltage,4.01)
    float intercept =  6.86 - slope*(Voltage_686-1500.0)/3.0;

    return 10*(slope*(ph_voltage-1500.0)/3.0+intercept);  //y = k*x + b
}

int read_orp_sensor(uint16_t orp_voltage)
{
    return orp_voltage/10;
}
