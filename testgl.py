import cv2
import numpy as np
from utils import *
from realsense import Camera
import pyglet
from pyglet.gl import *
from pyglet.window import key
from OpenGL.GL import *
# ArUco marker detection parameters
arucodetecter = cv2.aruco.ArucoDetector()
cam = Camera()
# Camera parameters
camera_matrix =  cam.cameraMatrix()  # Replace with actual camera matrix
dist_coeffs = np.array([0, 0, 0, 0, 0])  # Replace with actual distortion coefficients

# Set window dimensions
window_width, window_height = 800, 600

# Create window
window = pyglet.window.Window(window_width, window_height, resizable=True)

# Enable depth testing
glEnable(GL_DEPTH_TEST)

@window.event
def on_draw():
    # Clear the window
    window.clear()

    # Set the projection matrix
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, window_width / window_height, 0.1, 100.0)

    # Set the modelview matrix
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0)

    # Draw camera
    glColor3f(1, 0, 0)  # Red color for camera
    glPushMatrix()
    glTranslatef(0, 0, -1)
    glutWireCube(0.2)
    glPopMatrix()

    # Draw ArUco markers
    glColor3f(0, 0, 1)  # Blue color for markers
    for marker_pos in aruco_marker_positions:
        glPushMatrix()
        glTranslatef(*marker_pos)
        glutWireCube(0.2)
        glPopMatrix()

def update(dt):
    # Read frame from camera
    _,frame,_ = cam.get_frame_stream()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = arucodetecter.detectMarkers(gray)
    
    
    if ids is not None:
        # Estimate pose of detected markers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        # Update marker positions
        aruco_marker_positions = [tvec[0] for tvec in tvecs]
    
pyglet.clock.schedule_interval(update, 1 / 30.0)  # Update at 30 FPS


# Run the application
pyglet.app.run()


cv2.destroyAllWindows()
