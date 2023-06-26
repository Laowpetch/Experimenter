import pyrealsense2 as rs
import numpy as np

class Camera:
    def __init__(self):
        print("Loading Intel Realsense Camera")
        config = rs.config()
        print(config)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
        
        self.pipeline = rs.pipeline()
        self.profile_ = self.pipeline.start(config) ##
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def get_frame_stream(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None

        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return True, color_image, depth_image
    
    def release(self):
        self.pipeline.stop()
    
    def profile(self):
        return self.profile_
    
    def cameraMatrix(self):
        profile = self.pipeline.get_active_profile()
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        # Construct the camera matrix
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy

        camera_matrix = np.array([[fx, 0, cx],
                                [0, fy, cy],
                                [0, 0, 1]])
        print(type(camera_matrix))
        return camera_matrix