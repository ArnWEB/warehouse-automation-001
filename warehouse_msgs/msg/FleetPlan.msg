# Fleet Plan Message
# Published by Fleet Manager after cuOpt optimization

int32 robot_id
int32[] tasks          # Task indices in order
int32 plan_id          # Unique plan ID to prevent old plans being reused
float32 total_cost     # Total cost of this plan
float32 solve_time    # Time taken by cuOpt to solve
---
