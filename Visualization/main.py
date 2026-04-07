import serial
import time
from vpython import *


# --- 1. Serial Port Setup ---
# IMPORTANT: Update this to match your STM32's port on your Mac!
# It will likely look something like '/dev/cu.usbmodem14101' or '/dev/cu.usbserial-XXXX'
SERIAL_PORT = '/dev/cu.usbmodem102'
BAUD_RATE = 115200


try:
   print(f"Connecting to {SERIAL_PORT}...")
   ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
   time.sleep(2) # Give the serial connection a moment to initialize
   print("Connected!")
except Exception as e:
   print(f"Failed to connect: {e}")
   print("Running simulation without live data (waiting for connection).")
   ser = None


# while True:
#     # print(f"Bytes waiting: {ser.in_waiting}")
#     # time.sleep(0.5)
#     if ser.in_waiting > 0:
#         try:
#             line = ser.readline().decode('utf-8').strip()
#             if line.startswith(">Roll:"):
#                 roll_deg = float(line[6:])
#                 print(f"Roll: {roll_deg:.2f}°")
#             else:
#                 # Print unrecognized lines raw so you can see what's actually arriving
#                 print(f"[raw] {repr(line)}")
#         except ValueError as e:
#             print(f"[parse error] {repr(line)} -> {e}")
#         except Exception as e:
#             print(f"[error] {e}")


# --- 2. VPython Scene Setup (From previous step) ---
scene = canvas(title="Real-Time Missile Orientation", width=800, height=600, background=color.black)
scene.up = vector(0, 0, 1)
scene.camera.pos = vector(4, 4, 3.5)
scene.camera.axis = vector(-4, -4, -3)


axis_length = 3
shaft_thickness = 0.05
arrow(pos=vector(0,0,0), axis=vector(axis_length,0,0), color=color.red, shaftwidth=shaft_thickness)
arrow(pos=vector(0,0,0), axis=vector(0,axis_length,0), color=color.green, shaftwidth=shaft_thickness)
arrow(pos=vector(0,0,0), axis=vector(0,0,axis_length), color=color.cyan, shaftwidth=shaft_thickness)


roll_deg = 0.0


scaling_factor = 0.02
nosecone_length = 26
forward_length = 126.3
aft_length = 141.8
radius = 13.8/2
fin_height = 2 * (15.0 + radius)
fin_length = 50.0
fin_thickness = 1
canard_height = 2 * (10.0 + radius)
canard_length = 10.0


nosecone = ellipsoid(
   pos=vector(0, 0, (forward_length) * scaling_factor),
   width=nosecone_length * 2 * scaling_factor,
   length=radius * 2 * scaling_factor,
   height=radius * 2 * scaling_factor,
   color=color.gray(0.8),
)
forward_body = cylinder(
   pos=vector(0, 0, 0),
   axis=vector(0, 0, forward_length * scaling_factor),
   radius=radius * scaling_factor,
   color=color.gray(0.8),
)
canards_1_and_3 = box(
   pos=vector(0, 0, scaling_factor * (forward_length/2)),
   size=vector(fin_thickness * scaling_factor, canard_height * scaling_factor, canard_length * scaling_factor),
   color=color.gray(0.8),
)
canards_2_and_4 = box(
   pos=vector(0, 0, scaling_factor * (forward_length/2)),
   size=vector(canard_height * scaling_factor, fin_thickness * scaling_factor, canard_length * scaling_factor),
   color=color.gray(0.8),
)
aft_body = cylinder(
   pos=vector(0, 0, -aft_length * scaling_factor),
   axis=vector(0, 0, aft_length * scaling_factor),
   radius=radius * scaling_factor,
   color=color.gray(0.8),
)
fins_1_and_3 = box(
   pos=vector(0, 0, scaling_factor * (-aft_length + fin_length/2)),
   size=vector(fin_thickness * scaling_factor, fin_height * scaling_factor, fin_length * scaling_factor),
   color=color.gray(0.8),
)
fins_2_and_4 = box(
   pos=vector(0, 0, scaling_factor * (-aft_length + fin_length/2)),
   size=vector(fin_height * scaling_factor, fin_thickness * scaling_factor, fin_length * scaling_factor),
   color=color.gray(0.8),
)


missile = compound([nosecone, forward_body, canards_1_and_3, canards_2_and_4, aft_body, fins_1_and_3, fins_2_and_4])


# Create a HUD label
# pixel_pos=True makes the position relative to the window (in pixels)
# rather than 3D space. (0,0) is the bottom-left corner.
roll_label = label(
   pixel_pos=True,
   pos=vector(20, 550, 0), # Adjust the 550 based on your canvas height
   align='left',
   text='Roll: 0.00°',
   box=False,          # Removes the default bounding box
   color=color.white,
   height=20           # Font size
)


# --- 3. The Real-Time Animation Loop ---
print("Starting simulation loop...")


while True:
   rate(100)
  
   if ser and ser.in_waiting > 0:
       try:
           line = ser.readline().decode('utf-8').strip()
           if line.startswith(">Roll:"):
               roll_deg = float(line[6:])
               print(f"Roll: {roll_deg:.2f}°")
               roll_label.text = f"Roll: {roll_deg:.2f}°"
           else:
               print(f"[raw] {repr(line)}")
       except ValueError as e:
           print(f"[parse error] {repr(line)} -> {e}")
       except Exception as e:
           print(f"[error] {e}")


   # Always update the visual, using the last known good roll value
   missile.axis = vector(forward_length, 0, 0)
   missile.up = vector(0, 1, 0)
   missile.rotate(angle=radians(roll_deg), axis=vector(0, 0, 1))

