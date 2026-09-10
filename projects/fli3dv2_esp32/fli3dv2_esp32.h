#pragma once

void setup_gpio();
void setup_separation();
void setup_power();
void setup_buzzer();
void setup_timer();
bool setup_icm20948();
bool setup_bmp388();
bool setup_bmp280();
bool setup_neo6mv2();
bool acquire_icm20948();
bool acquire_bmp280();
bool acquire_bmp388();
bool acquire_neo6mv2();
bool zero_bmp280();
bool zero_bmp388();

void separation_publish();