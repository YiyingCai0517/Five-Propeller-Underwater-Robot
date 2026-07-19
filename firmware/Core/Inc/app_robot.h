//
// Created by DS on 2026/3/9.
//

#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include "main.h"

void StartControlTask(void *argument);
void StartDepthTask(void *argument);
void StartCommTask(void *argument);

#endif //APP_ROBOT_H