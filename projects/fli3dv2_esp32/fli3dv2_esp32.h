#pragma once

void setup_gpio();
void setup_separation();
void setup_power();
void setup_buzzer();
void setup_timer();
bool setup_icm20948();
bool setup_bmp388();
bool setup_neo6mv2();
bool acquire_IMU();
bool acquire_BMP();
bool acquire_gps();

void separation_publish();