# Future Plan 001: Production Warehouse Automation System

## Vision

Transform the current warehouse automation system from a simple demo to a production-ready system capable of handling real-time task management, dynamic orders, and robust fleet orchestration.

---

## Current State (Working)

### What's Implemented:
- **Order Flow**: order_generator → cuopt_bridge → cuopt_client → optimized plan
- **Fleet Flow**: fleet_task_generator → fleet_manager → cuopt_client → optimized plan
- **Real CuOpt Integration**: Uses NVIDIA CuOpt API with 90-waypoint Isaac Sim warehouse
- **Multi-Robot Support**: 3 AMRs (amr1, amr2, amr3)
- **Visualization**: Fleet dashboard shows task execution

### Current Limitations:
- Hardcoded task sequences in cuopt_bridge
- No task batching (each order = separate optimization)
- No add/cancel task functionality
- No real-time task modification
- No priority handling
- No failure recovery

---

## Phase 1: Task Management (Near Term)

### Goal
Enable real-time task management with add/cancel/re-plan capabilities.

### Features

#### 1. Task Queue System
- Accumulate multiple tasks before optimization
- Track task states (pending → assigned → executing → completed)
- Prevent duplicate task execution

#### 2. Add Task
- Allow adding new tasks via ROS topic/service
- Validate task parameters
- Add to pending queue

#### 3. Cancel Task
- Allow cancelling pending tasks
- Skip cancelled tasks in optimization
- Handle cancellation during execution

#### 4. Re-plan Logic
- Trigger re-optimization when tasks change
- Re-assign tasks to robots
- Handle partial plan updates

### Implementation

#### New Topics:
```bash
/fleet/task_add        # Add new task
/fleet/task_cancel     # Cancel task by ID  
/fleet/replan          # Force re-plan
```

#### New Services:
```bash
fleet_manager/add_task     # Add task (request/response)
fleet_manager/cancel_task # Cancel task (request/response)
fleet_manager/get_queue  # Get pending tasks
fleet_manager/replan     # Trigger re-plan
```

#### Task Structure:
```python
Task = {
    "task_id": "TASK-001",      # Unique ID
    "type": "pickup",            # pickup, deliver, move, stack, retrieve
    "from_location": "palletizer",  # optional
    "to_location": "staging",    # required for move/deliver
    "payload_kg": 10,
    "priority": 3,               # 1=highest, 5=lowest
    "status": "pending"          # pending, assigned, executing, completed, cancelled, failed
}
```

### Files to Modify:
- `cuopt_bridge/cuopt_bridge/cuopt_bridge.py` - Add task queue and handlers
- `orchestrator/orchestrator/fleet_manager.py` - Add services

---

## Phase 2: Order Enhancement (Short Term)

### Goal
Make orders specify their own tasks instead of hardcoded sequence.

### Features

#### 1. Dynamic Task Specification
Orders can include custom task sequences:
```python
order = {
    "order_id": "ORD-001",
    "priority": 3,
    "tasks": [
        {"type": "pickup", "from": "palletizer"},
        {"type": "deliver", "to": "staging_area"}
    ]
}
```

#### 2. Task Validation
- Validate locations exist in warehouse
- Validate task types are supported

#### 3. Default Fallback
If order has no tasks, use sensible defaults.

### Files to Modify:
- `order_system/order_system/order_generator.py` - Support custom tasks
- `cuopt_bridge/cuopt_bridge/cuopt_bridge.py` - Parse dynamic tasks

---

## Phase 3: Priority & Deadline (Medium Term)

### Goal
Handle task priority and deadlines for production use.

### Features

#### 1. Priority Queue
- Sort tasks by priority (1=highest)
- High priority tasks optimized first

#### 2. Deadline Handling
- Track task deadlines
- Warn if deadline cannot be met
- Prioritize urgent tasks

#### 3. Priority Override
- Allow real-time priority changes
- Trigger re-plan on priority change

### Priority Levels:
| Priority | Use Case |
|----------|----------|
| 1 | Urgent (rush order) |
| 2 | High (same-day) |
| 3 | Normal (standard) |
| 4 | Low (batch) |
| 5 | Bulk (restocking) |

---

## Phase 4: Production Robustness (Long Term)

### Goal
Make system production-ready with failure handling and monitoring.

### Features

#### 1. Failure Handling
- Detect robot failure
- Re-assign failed tasks
- Notify operators

#### 2. Recovery Logic
- Resume from checkpoint
- Handle partial completion
- Graceful degradation

#### 3. Monitoring Dashboard
- Real-time task status
- Robot health
- Performance metrics

#### 4. Audit Logging
- Log all task operations
- Track robot activity
- Compliance logging

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    PRODUCTION WAREHOUSE SYSTEM                            │
└─────────────────────────────────────────────────────────────────────────────┘

                           ┌─────────────────┐
                           │   ERP / WMS     │  ← External systems
                           │  (Future)       │
                           └────────┬────────┘
                                    │
                           ┌────────▼────────┐
                           │  ORDER MANAGER  │  ← Central coordinator
                           │                  │
                           │ - Order queue    │
                           │ - Task states    │
                           │ - Priority queue │
                           └────────┬────────┘
                                    │
        ┌─────────────────────────────┼─────────────────────────────┐
        │                             │                             │
┌──────▼──────┐              ┌──────▼──────┐              ┌──────▼──────┐
│ USER INPUT  │              │ ROBOT STATUS │              │ TASK STATUS │
│  (Tablet)   │              │  MONITOR     │              │  MONITOR    │
└──────┬──────┘              └──────┬──────┘              └──────┬──────┘
       │                            │                            │
       └────────────────────────────┼────────────────────────────┘
                                    │
                           ┌────────▼────────┐
                           │  CUOPT BRIDGE   │  ← Task management
                           │                  │
                           │ - Task queue     │
                           │ - Add/cancel     │
                           │ - Re-plan logic  │
                           └────────┬────────┘
                                    │
                           ┌────────▼────────┐
                           │   CUOPT CLIENT  │  ← Optimization
                           │   (Real API)    │
                           └────────┬────────┘
                                    │
                           ┌────────▼────────┐
                           │   EXECUTORS     │  ← Per robot
                           │   amr1, amr2    │
                           └─────────────────┘
```

---

## Task States

```
┌─────────┐     ┌──────────┐     ┌───────────┐     ┌────────────┐
│ PENDING │ ──▶ │ASSIGNED  │ ──▶ │EXECUTING  │ ──▶ │COMPLETED  │
└─────────┘     └──────────┘     └───────────┘     └────────────┘
      │                                  │
      │              ┌──────────────────┘
      ▼              ▼
┌────────────┐  ┌────────────┐
│  CANCELLED │  │   FAILED   │
└────────────┘  └────────────┘
```

---

## Re-Planning Triggers

| Trigger | Action |
|---------|--------|
| New task added | Re-optimize all pending tasks |
| Task cancelled | Remove & re-optimize remaining |
| Task failed | Re-assign to available robot |
| Robot completes task | Check pending → assign next |
| High priority task | Re-prioritize & re-optimize |
| Robot battery low | Reassign tasks, send to charging |

---

## Implementation Checklist

### Phase 1: Task Management
- [ ] Add task queue in cuopt_bridge
- [ ] Implement /fleet/task_add topic
- [ ] Implement /fleet/task_cancel topic
- [ ] Implement /fleet/replan topic
- [ ] Add ROS services for request/response

### Phase 2: Order Enhancement
- [ ] Modify order_generator to support custom tasks
- [ ] Update cuopt_bridge to parse dynamic tasks
- [ ] Add task validation

### Phase 3: Priority & Deadline
- [ ] Implement priority queue
- [ ] Add deadline handling
- [ ] Priority override functionality

### Phase 4: Production Robustness
- [ ] Failure detection
- [ ] Recovery logic
- [ ] Monitoring dashboard
- [ ] Audit logging

---

## Notes

- Current system uses real NVIDIA CuOpt API at `43.201.55.122:5000`
- Warehouse has 90 waypoints from Isaac Sim
- Coordinates range: X (1.48-34.42), Y (4.16-61.49)
- Default robots: amr1 (charging), amr2 (staging), amr3 (asrs_output)

---

## Timeline

| Phase | Effort | Priority |
|-------|--------|----------|
| Phase 1: Task Management | Medium | High |
| Phase 2: Order Enhancement | Low | High |
| Phase 3: Priority & Deadline | Medium | Medium |
| Phase 4: Production Robustness | High | Low |

---

*Document Version: 1.0*
*Created: 2026-02-27*
