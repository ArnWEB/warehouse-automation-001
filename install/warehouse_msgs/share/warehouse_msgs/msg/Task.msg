# Task Message
# Represents a warehouse task (pickup/delivery)

int32 task_id
float32 pick_x      # Pickup location x
float32 pick_y      # Pickup location y
float32 drop_x      # Drop location x
float32 drop_y      # Drop location y
int32 priority      # 0=low, 1=medium, 2=high
int32 quantity      # Number of items
float32 time_window_start
float32 time_window_end
string task_type    # "pickup_delivery", "transport", etc.
---
