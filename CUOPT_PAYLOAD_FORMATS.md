# CuOpt Payload Formats - Ideal vs Current

## Current Issue
Error: `ValueError: cuOpt Error: Bad Request - 400: Length of weights array must be equal to edges array`

This happens with the CSR (Compressed Sparse Row) waypoint graph format.

---

## Option 1: Simple Distance Matrix Format (RECOMMENDED)

**Most maintainable and straightforward:**

```json
{
  "vehicles": [
    {
      "id": "amr1",
      "capacity": [10],
      "start_location": 0,
      "end_location": 0
    }
  ],
  "jobs": [
    {"id": 10, "location": 10, "service_time": 0},
    {"id": 20, "location": 20, "service_time": 0},
    {"id": 30, "location": 30, "service_time": 0}
  ],
  "vehicle_locations": [[5, 35], [12.25, 53.34], [3.51, 45.9], ...],
  "distance_matrix": {
    "type": "symmetric",
    "dimension": 90,
    "data": [
      [0, 1.5, 2.3, ...],
      [1.5, 0, 1.8, ...],
      ...
    ]
  },
  "options": {
    "traffic_engine": "OSRM"
  }
}
```

**Pros:**
- Simple symmetric distance matrix
- Clear vehicle/job structure  
- Works with all CuOpt versions
- Easy to validate dimensions

---

## Option 2: CSR (Compressed Sparse Row) Waypoint Graph Format

**Current approach - requires fixing weights/edges mismatch:**

```json
{
  "cost_waypoint_graph_data": {
    "waypoint_graph": {
      "0": {
        "offsets": [0, 3, 5, 10, ...],    // CSR row offsets
        "edges": [1, 2, 4, ...],           // Target node indices
        "distances": [1.5, 2.3, 0.8, ...]  // Must match edge count!
      }
    }
  },
  "fleet_data": {
    "vehicle_locations": [[5, 36], [0, 0], ...],
    "capacities": [[10, 10]],
    "vehicle_time_windows": null
  },
  "task_data": {
    "task_locations": [10, 20, 30],
    "demand": [[1, 1, 1]],
    "task_time_windows": null,
    "service_times": null
  },
  "solver_config": {
    "time_limit": 0.01
  }
}
```

**Issue:** Must ensure `len(distances) == len(edges)` for every waypoint

---

## Option 3: High-Level VRP Format (Recommended for Simplicity)

```json
{
  "orders": [
    {"id": 10, "location": [12.25, 53.34], "demand": [1]},
    {"id": 20, "location": [3.51, 45.9], "demand": [1]},
    {"id": 30, "location": [5.2, 40.1], "demand": [1]}
  ],
  "vehicles": [
    {
      "id": "amr1",
      "start_location": [5.07, 36.92],
      "end_location": [5.07, 36.92],
      "capacity": [10],
      "earliest_start": 0,
      "latest_return": 3600
    },
    {
      "id": "amr2",
      "start_location": [1.62, 18.82],
      "end_location": [1.62, 18.82],
      "capacity": [10],
      "earliest_start": 0,
      "latest_return": 3600
    },
    {
      "id": "amr3",
      "start_location": [0.0, 0.0],
      "end_location": [0.0, 0.0],
      "capacity": [10],
      "earliest_start": 0,
      "latest_return": 3600
    }
  ],
  "travel_matrix": {
    "distances": [
      [0.0, 1.5, 2.3, ...],
      [1.5, 0.0, 1.8, ...],
      ...
    ],
    "times": [
      [0.0, 1.0, 1.5, ...],
      [1.0, 0.0, 0.9, ...],
      ...
    ]
  }
}
```

---

## Recommended Fix Strategy

**Best approach:** Use Option 1 (Simple Distance Matrix) because:

1. ✅ **No CSR complexity** - All waypoints must have matching edge/distance counts
2. ✅ **Clearer structure** - Vehicle and job definitions are explicit
3. ✅ **Easier validation** - Simple 2D matrix, one size for all
4. ✅ **Better debuggability** - Error messages more informative
5. ✅ **Warehouse compatible** - 90 waypoints × 90 = 8,100 matrix values

---

## Current Warehouse Setup

- **Robots**: amr1, amr2, amr3
- **Waypoints**: 90 locations (WAREHOUSE_LOCATIONS dictionary)
- **Depot**: Waypoint 0 (origin)
- **Distance calc**: Euclidean from coordinates

---

## Implementation Notes

In `cuopt_client.py`:

1. Replace `_build_waypoint_graph()` with distance matrix generation
2. Change payload structure from CSR to simple format
3. Update `_solve_cuopt()` to send simplified payload
4. Keep mock fallback for robustness

Example distance matrix generation:
```python
def _build_distance_matrix(self):
    """Build symmetric distance matrix from warehouse locations."""
    n = len(WAREHOUSE_LOCATIONS)
    matrix = [[0.0] * n for _ in range(n)]
    
    for i in range(n):
        for j in range(n):
            if i == j:
                matrix[i][j] = 0.0
            else:
                loc_i = WAREHOUSE_LOCATIONS[i]
                loc_j = WAREHOUSE_LOCATIONS[j]
                dist = math.sqrt(
                    (loc_i["x"] - loc_j["x"])**2 + 
                    (loc_i["y"] - loc_j["y"])**2
                )
                matrix[i][j] = dist
    
    return matrix
```
