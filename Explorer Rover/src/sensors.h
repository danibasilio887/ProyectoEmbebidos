#ifndef SENSORS_H
#define SENSORS_H

void sensors_init(); 
void sensors_update(); 

// Getters
float get_temp();
float get_humidity();
long  get_dist_front();
long  get_dist_left();
long  get_dist_right();
float get_angle_roll();
float get_angle_pitch();
float get_battery_voltage();

#endif // SENSORS_H