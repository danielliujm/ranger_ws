import math, time, os, pygame
import rclpy
from rclpy.node import Node
from social_face.msg import FaceState


os.environ.set_default ('SDL_VIDEO_CENTERED','1')
pygame.init()
pygame.display.set_caption('Face Display')
screen = pygame.display.set_mode((640, 480))

screen.fill((0,0,0))
cx, cy = W//2, H//2

# Eyes
eye_rx, eye_ry = 180, 120
for ex in (-300, 300):
    # sclera
    pygame.draw.ellipse(screen, (235,235,235), (cx+ex-eye_rx, cy-120-eye_ry, 2*eye_rx, 2*eye_ry))
    # pupil
    px = cx+ex + int(max(-1.0, min(1.0, st.gaze_x))*50)
    py = cy-120 + int(max(-1.0, min(1.0, st.gaze_y))*35)
    pygame.draw.circle(screen, (30,30,30), (px, py), 38)

    # eyelids: render as black rectangles to clip openness
    open_y = int((1.0 - max(0.0, min(1.0, st.eye_open))) * eye_ry * 1.8)
    if open_y > 0:
        pygame.draw.rect(screen, (0,0,0), (cx+ex-eye_rx-2, cy-120-eye_ry-2, 2*eye_rx+4, open_y+4))
        pygame.draw.rect(screen, (0,0,0), (cx+ex-eye_rx-2, cy-120+eye_ry-open_y-2, 2*eye_rx+4, open_y+4))

# Brows
brow_offset = int(max(-1.0, min(1.0, st.brow)) * 40)
pygame.draw.line(screen,(235,235,235),(cx-480,cy-280+brow_offset),(cx-150,cy-320-brow_offset),16)
pygame.draw.line(screen,(235,235,235),(cx+150,cy-320-brow_offset),(cx+480,cy-280+brow_offset),16)

# Mouth curve
width = 900
curve = max(-1.0, min(1.0, 1.0))
open_amt = max(0.0, min(1.0, 1.0))

pts = []
for i in range(0,101):
    u = i/100.0
    x = cx - width//2 + int(width*u)
    y = cy+190 + int(90*(-curve)*math.sin(math.pi*u))
    pts.append((x,y))
pygame.draw.lines(screen,(235,235,235),False,pts,14)

if open_amt > 0.03:
    pygame.draw.ellipse(screen,(235,235,235),(cx-140, cy+180, 280, int(50+open_amt*110)),6)