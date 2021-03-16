from enum import IntEnum
import math
 
class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y
 
    def __str__(self):
        return f'<{self.x}, {self.y}>'
 
# Positive means move into positive X region
class CLEANING_DIR(IntEnum):
    NEGATIVE = -1
    POSITIVE = 1
    
class TURN_DIR(IntEnum):
    RIGHT = -1 
    LEFT = 1
 
# Right hand coordinate system such that
# x-axis - along the net
# y-axis - perpendiculat to the net
# (0, 0) - center of the court
 
 
DEFAULT_NET_OVERLAP = 0.3
 
def gen_zig_zag(init_point: Point, init_turn_dir: TURN_DIR, max_dist_from_net: float, net_width: float, net_overlap: float):
    cleaning_dir = CLEANING_DIR(math.copysign(1, -init_point.x))
    turn_width = net_width - net_overlap
    return gen_zig_zag_helper(init_point, cleaning_dir, init_turn_dir, turn_width, max_dist_from_net,)
 
def gen_zig_zag_helper(init_point: Point, cleaning_dir: CLEANING_DIR, turn_dir: TURN_DIR, turn_width: float, max_dist_from_net: float):
    # return empty list if too far from the net
    if abs(init_point.y) > max_dist_from_net:
        return []
 
    # calculate the point of the next segment on the opposite side
    next_segment_init_point = Point(-init_point.x, init_point.y + cleaning_dir.value * turn_dir.value * turn_width)
    
    # recursively calculate all the points
    return [init_point, Point(-init_point.x, init_point.y)] + gen_zig_zag_helper(next_segment_init_point, CLEANING_DIR(-cleaning_dir.value), TURN_DIR(-turn_dir.value), turn_width, max_dist_from_net)
 
def gen_circular(init_point: Point, turn_dir: TURN_DIR, max_dist_from_net: float, net_width: float, net_overlap: float):
    cleaning_dir = CLEANING_DIR(math.copysign(1, -init_point.x))
    turn_width = 3 / 2 * net_width - 2 * net_overlap
    return gen_circular_helper(init_point, 0, cleaning_dir, turn_dir, turn_width, max_dist_from_net)
 
def gen_circular_helper(init_point: Point, phase: int, cleaning_dir: CLEANING_DIR, turn_dir: TURN_DIR, turn_width: float, max_dist_from_net: float):
    # return empty list if too far from the net
    if abs(init_point.y) > max_dist_from_net:
        return []
 
    # calculate the point of the next segment on the opposite side
    y_displacement =  cleaning_dir.value * turn_dir.value * turn_width 
    if phase == 1:
        y_displacement /= 2
    next_segment_init_point = Point(-init_point.x, init_point.y + y_displacement )
 
    # recursively calculate all the points
    return [init_point, Point(-init_point.x, init_point.y)] + gen_circular_helper(
        next_segment_init_point, 
        (phase + 1) % 2, 
        CLEANING_DIR(-cleaning_dir.value), 
        turn_dir, turn_width, max_dist_from_net
    )
 
 
if __name__ == '__main__':
    init_point = Point(-3.0, 1.0)
 
    print('ZIG-ZAG:')
    zigzag_pattern_points = gen_zig_zag(init_point=init_point, init_turn_dir=TURN_DIR.LEFT, max_dist_from_net=10, net_width=2, net_overlap=DEFAULT_NET_OVERLAP)
    for point in zigzag_pattern_points:
        print(point)
 
    print('CIRCULAR:')
    zigzag_pattern_points = gen_circular(init_point=init_point, turn_dir=TURN_DIR.RIGHT, max_dist_from_net=10, net_width=2, net_overlap=DEFAULT_NET_OVERLAP)
    for point in zigzag_pattern_points:
        print(point)