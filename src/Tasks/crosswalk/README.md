# Crosswalk action

Crosswalk node - node activated when crosswalk is detected. It manages detection of pedestrians and waiting for them.

Server uses `crosswalk.action` to communicate with client.

```mermaid
stateDiagram-v2
    [*] --> DISABLED
    DISABLED --> IDLE
    IDLE --> APPROACHING_TO_EMPTY_CROSSWALK: Goal received
    APPROACHING_TO_EMPTY_CROSSWALK --> IDLE: Passed crosswalk
    APPROACHING_TO_EMPTY_CROSSWALK --> APPROACHING_TO_NOT_EMPTY_CROSSWALK: Detected pedestrian
    APPROACHING_TO_NOT_EMPTY_CROSSWALK --> APPROACHING_TO_EMPTY_CROSSWALK: Pedestrian left
    APPROACHING_TO_NOT_EMPTY_CROSSWALK --> WAITING_FOR_PEDESTRIANS: Approached to crosswalk
    WAITING_FOR_PEDESTRIANS --> IDLE: Pedestrian left
```

## Topics

### Action name

- `task/crosswalk`

### Subscribed topics

- `/obstacles` ([custom_msgs/Box2DArray](./../../Shared/custom_msgs/msg/Box2DArray.msg))
- `/road_lines`

### Published topics

## Parameters

- `roi_min_x`, `roi_max_x`, `roi_min_y`, `roi_max_y` (_float_)
  - describing area of interest
