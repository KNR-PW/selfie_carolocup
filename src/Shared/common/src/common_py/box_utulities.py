from geometry_msgs.msg import Point

from custom_msgs.msg import Box2D


#https://stackoverflow.com/questions/8721406/how-to-determine-if-a-point-is-inside-a-2d-convex-polygon
# result = False
# for (i = 0, j = points.length - 1; i < points.length; j = i++) {
# if (points[i].y > test.y) != (points[j].y > test.y) &&
#    (test.x < (points[j].x - points[i].x) * (test.y - points[i].y) /
#       (points[j].y-points[i].y) + points[i].x)
# {
#  result = !result;
# }
def is_point_inside_box(box: Box2D, point: Point):

    return True  # TODO implement properly


def is_box_center_inside(area_of_interest: Box2D, box: Box2D) -> bool:
    center = box.point_centroid
    return is_point_inside_box(center)


def filter_boxes(area_of_interest: Box2D, boxes_list: list) -> list:
    result = []
    for box in boxes_list:
        pass
    return result
