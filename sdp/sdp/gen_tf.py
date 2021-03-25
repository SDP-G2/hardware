import sys

def gen_tf(n: int, x: float, y: float, z: float, yaw: float, roll: float):
    return f"""
    aruco_{n}_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_{n}_transform",
        arguments=["{x:.2f}", "{y:.2f}", "{z:.2f}", "{yaw:.4f}", "0", "{roll:.4f}", "world", "aruco_{n}"]
    )
"""

if __name__ == '__main__':
    startId, numId, start_x, start_y, z, yaw, roll = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
    with open('temp.tf', 'w') as f:
        for i in range(numId):
            y = start_y - 2 * i
            x = start_x
            f.write(gen_tf(startId + i, x, y, z, yaw, roll))