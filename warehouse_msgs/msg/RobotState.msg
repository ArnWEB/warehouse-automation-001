# Robot State Message
# Published by Robot State Monitor

int32 robot_id
float32 x
float32 y
float32 theta      # orientation
bool busy          # true if executing task
int32 current_task  # -1 if idle
float32 battery_level
---
