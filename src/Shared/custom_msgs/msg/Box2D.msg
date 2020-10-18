Header header

# Box2D apex position
#
#             ^x
#      tl  tr |  tl  tr
#       ---   |   ---
#       | |   |   | |
#       ---   |   ---
#      bl  br |  bl  br
#             |
#  <y_________|___________
#             |
#      tl  tr |  tl  tr
#       ---   |   ---  
#       | |   |   | |
#       ---   |   ---
#      bl  br |  bl  br
#             |

# apex points
# t - top
# b - back
# r - right
# l - left
geometry_msgs/Point tr
geometry_msgs/Point tl
geometry_msgs/Point br
geometry_msgs/Point bl

# center point
geometry_msgs/Point point_centroid

# dimentions
float32 width
float32 length