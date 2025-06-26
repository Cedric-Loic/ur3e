import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

W = 640
H = 480


# Initialize Intel RealSense pipeline
config = rs.config()
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

pipeline = rs.pipeline()
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# Load YOLO model
model_directory = '/home/normanthia/Documents/demonstration/best.pt'
model = YOLO(model_directory)

while True:
   
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    results = model(color_image)[0]
    annotated_image = results.plot()

    if results.obb is not None and len(results.obb) > 0:
            for i, box in enumerate(results.obb):
                conf = float(results.conf[i]) if hasattr(results, 'conf') else float(box.conf)
                if conf < 0.750:
                    continue

                # Extract bounding box parameters (x, y, width, height, angle)
                x, y, w, h, angle = box.xywhr[0]  # Get bounding box details
                center_x = int(x)
                center_y = int(y)
                angle = float(angle)

                # Ensure height is the major axis
                if h > w:
                    w, h = h, w
                    angle += np.pi / 2  # Adjust angle if swapped

                # Compute segment endpoints along the height
                half_h = h / 2
                beg_x = int(center_x - half_h * np.sin(angle))
                beg_y = int(center_y + half_h * np.cos(angle))
                end_x = int(center_x + half_h * np.sin(angle))
                end_y = int(center_y - half_h * np.cos(angle))

                # Draw center point
                cv2.circle(annotated_image, (center_x, center_y), 5, (0, 0, 255), -1)  # Red dot

                # Draw green segment
                cv2.line(annotated_image, (beg_x, beg_y), (end_x, end_y), (0, 255, 0), 2)

                # Display angle for debugging
                cv2.putText(annotated_image, f"{int(np.degrees(angle))} deg", (center_x, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        print("No objects detected in the current frame.")

    # Show the annotated image
    cv2.imshow('YOLOv8 Inference', annotated_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
pipeline.stop()
cv2.destroyAllWindows()
