### Subscribed topics
  - point cloud published by LIDAR sensor
  - 2D Camera Image
### Published topics
- `/obstacles` ([custom_msgs/Box3DArray](./../../Shared/custom_msgs/msg/Box3DArray.msg))
  - Polygon Array contains vertexes of cuboid of every obstacle in local XYZ
- `/probable_signs` ([custom_msgs/ProbableSignArray](./../../Shared/custom_msgs/msg/ProbableSignArray.msg))
  - Polygon Array contains vertexes of cuboid of every obstacle that can be a sign in local XYZ with position on 2D image
