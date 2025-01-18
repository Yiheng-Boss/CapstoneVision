import cv2
import numpy as np
import time
from picamera2 import Picamera2
from ultralytics import YOLO          

# Load YOLO model
model_path = ("/home/pi/Desktop/M2G15/best_m2g15.pt")  # Replace with your YOLO model file path
model = YOLO(model_path)

# Initialize Raspberry Pi Camera
camera = Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', "size": (640, 640)}) #addition format use to capture rgb image
camera.configure(config)
camera.start()
time.sleep(0.1)  # Allow camera to warm up

def detect_objects(frame):
    # Convert the frame to RGB format (YOLO expects RGB images)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)  # Use COLOR_BGR2RGB if your frame is in BGR format

    # Run YOLO detection
    results = model.predict(frame_rgb, verbose=False)
    print("Frame shape:", frame_rgb.shape)

    # Process detected objects
    for result in results[0].boxes:  # Access detected bounding boxes
        cls = int(result.cls)  # Class ID
        confidence = result.conf.item()
        x_min, y_min, x_max, y_max = map(int, result.xyxy[0])  # Bounding box

        label = model.names[cls]  # Get the class name
        print(f"Detected: {label} with confidence {confidence:.2f}")

        # Draw the detection on the frame
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {confidence:.2f}", (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame

def main():
    try:
        while True:
            # Capture a frame
            frame = camera.capture_array()

            # Detect objects
            frame = detect_objects(frame)

            # Display the frame
            cv2.imshow("Object Detection", frame)

            # Exit on 'Esc' key
            if cv2.waitKey(1) & 0xFF == 27:  # ASCII code for 'Esc'
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        # Cleanup
        cv2.destroyAllWindows()
        camera.stop()

if name == "main":
    main()