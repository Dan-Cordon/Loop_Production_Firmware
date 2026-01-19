#pragma once

void can_rx_task(void* pv);
void sensor_task(void* pv);
void valve_control_task(void* pv);
void main_control_task(void* pv);
void tank_fill_task(void* pv);
void spray_valve_task(void* pv);