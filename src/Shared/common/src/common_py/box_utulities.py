from std_msgs.msg import Header
from geometry_msgs.msg import Point
from custom_msgs.msg import Box2D

from shapely.geometry import Point as PointShapely
from shapely.geometry.polygon import Polygon as PolygonShapely


def is_point_inside_box(box: Box2D, point: Point) -> bool:

    tl = (Box2D.tl.x, Box2D.tl.y)
    tr = (Box2D.tr.x, Box2D.tr.y)
    br = (Box2D.br.x, Box2D.br.y)
    bl = (Box2D.bl.x, Box2D.bl.y)
    polygon = PolygonShapely([tl, tr, br, bl])

    point_x = Point.x
    point_y = Point.y
    point = PointShapely(point_x, point_y)

    if polygon.contains(point) or point.touches(polygon):
        return True
    return False


def is_box_center_inside(area_of_interest: Box2D, box: Box2D) -> bool:
    center = box.point_centroid
    return is_point_inside_box(area_of_interest, center)


def filter_boxes(area_of_interest: Box2D, boxes_list: list) -> list:
    """
    if 4 or 5 box's points (4 corners and center) are inside area_of_interest, box is accepted
    """
    result = []
    for box in boxes_list:

        points_in_area_of_interest = 0

        tl_point = box.tl
        tr_point = box.tr
        br_point = box.br
        bl_point = box.bl
        center = box.point_centroid
        points = [tl_point, tr_point, br_point, bl_point, center]
        for point in points:
            if is_point_inside_box(area_of_interest, point):
                points_in_area_of_interest += 1
        if points_in_area_of_interest >= 4:
            result.append(box)
    return result


def create_square_box(x_min, y_min, x_max, y_max) -> Box2D:
    box = Box2D(header=Header())
    box.tl = Point(x=x_max, y=y_max, z=0)
    box.tr = Point(x=x_max, y=y_min, z=0)
    box.bl = Point(x=x_min, y=y_max, z=0)
    box.br = Point(x=x_min, y=y_min, z=0)
    return box
