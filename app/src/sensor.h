#ifndef _BME280_SENSOR_H
#define _BME280_SENSOR_H

struct bme280_measurements {
    double temp;
    double press;
    double humidity;
};

extern struct bme280_measurements sensor_readings;

void sample_sensor();
void configure_bme280_sensor();
#endif
