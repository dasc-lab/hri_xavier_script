import pyrealsense2 as rs
import numpy as np
import cv2


pipe = rs.pipeline()
cfg = rs.config()
align_to = rs.stream.color
align = rs.align(align_to)

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

pipe.start(cfg)
#output = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MPEG'), 30, (640,480))


while True:
    frame = pipe.wait_for_frames()
    aligned_frames = align.process(frame)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()
    aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    if :
    	color_image = cv2.circle(color_image, (200, 160), 4, (255,0,0),-1)
    cv2.imshow('rgb', color_image)
    cv2.imshow('depth', depth_image)
    cv2.imshow('aligned image',aligned_depth_image)
    #output.write(color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
pipe.stop()
#output.release()
