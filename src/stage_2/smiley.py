import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import math, time

rclpy.init()
node = Node('smiley')
pen_cl = node.create_client(SetPen, '/turtle1/set_pen')
tel_cl = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
rst_cl = node.create_client(Empty, '/reset')
pub    = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

for cl in [pen_cl, tel_cl, rst_cl]:
    cl.wait_for_service()

def spin():
    rclpy.spin_once(node, timeout_sec=0.05)

def reset():
    f = rst_cl.call_async(Empty.Request())
    while not f.done(): spin()
    time.sleep(1)

def set_pen(r, g, b, w, off):
    req = SetPen.Request()
    req.r, req.g, req.b, req.width, req.off = r, g, b, w, off
    f = pen_cl.call_async(req)
    while not f.done(): spin()
    time.sleep(0.2)

def pen_off():
    set_pen(0, 0, 0, 1, 1)
    time.sleep(0.3)

def teleport(x, y, th):
    req = TeleportAbsolute.Request()
    req.x, req.y, req.theta = float(x), float(y), float(th)
    f = tel_cl.call_async(req)
    while not f.done(): spin()
    time.sleep(0.2)

def go_to(x, y, th):
    pen_off()
    teleport(x, y, th)

def draw_arc(cx, cy, radius, a_start, a_end, r, g, b, w):
    sx = cx + radius * math.cos(a_start)
    sy = cy + radius * math.sin(a_start)
    go_to(sx, sy, a_start + math.pi/2)

    set_pen(r, g, b, w, 0)
    time.sleep(0.2)

    total_angle = a_end - a_start
    total_time  = abs(total_angle) * 1.5
    omega = total_angle / total_time
    v     = radius * abs(omega)

    twist = Twist()
    twist.linear.x  = v
    twist.angular.z = omega

    end_time = time.time() + total_time
    while time.time() < end_time:
        pub.publish(twist)
        spin()
        time.sleep(0.05)

    stop = Twist()
    pub.publish(stop)
    time.sleep(0.2)
    pen_off()

# ── RESET ────────────────────────────────────────────
reset()

# ── FACE (yellow) ────────────────────────────────────
print("Drawing face...")
draw_arc(5.5, 5.5, 3.2, 0, 2*math.pi, 255, 220, 0, 6)

# ── LEFT EYE (cyan) ──────────────────────────────────
print("Drawing left eye...")
draw_arc(4.2, 7.0, 0.4, 0, 2*math.pi, 0, 220, 255, 4)

# ── RIGHT EYE (green) ────────────────────────────────
print("Drawing right eye...")
draw_arc(6.8, 7.0, 0.4, 0, 2*math.pi, 0, 255, 100, 4)

# ── SMILE (red arc only) ─────────────────────────────
print("Drawing smile...")
draw_arc(5.5, 5.0, 1.2, math.radians(-150), math.radians(-30), 255, 80, 80, 5)

print("Done!")
node.destroy_node()
rclpy.shutdown()