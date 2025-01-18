import cv2
import numpy as np
import time
import lgpio
from picamera2 import Picamera2
from ultralytics import YOLO   

# GPIO setup
INPUT_PIN = 18           # Pin to receive start signal
OUTPUT_PIN_PLANT1 = 23   # Pin to signal Spider Plant detection
OUTPUT_PIN_PLANT2 = 25   # Pin to signal Purple Heart detection
OUTPUT_PIN_FAILURE = 24  # Pin to signal detection failure

# Initialize GPIO
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, INPUT_PIN)
lgpio.gpio_claim_output(h, OUTPUT_PIN_PLANT1)
lgpio.gpio_claim_output(h, OUTPUT_PIN_PLANT2)
lgpio.gpio_claim_output(h, OUTPUT_PIN_FAILURE)

# Load YOLO model
model_path = "/home/pi/Desktop/M2G15/best_m2g15.pt"  # Replace with your YOLO model file path
model = YOLO(model_path)

# Initialize Raspberry Pi Camera
camera = Picamera2()
config = camera.create_preview_configuration(main={'format': 'RGB888', "size": (640, 640)})
camera.configure(config)

def detect_objects(frame):
    # Convert the frame to RGB format (YOLO expects RGB images)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

    # Run YOLO detection
    results = model.predict(frame_rgb, verbose=False)
    plant1_detected = False
    plant2_detected = False

    # Process detected objects
    for result in results[0].boxes:
        cls = int(result.cls)  # Class ID
        confidence = result.conf.item()
        x_min, y_min, x_max, y_max = map(int, result.xyxy[0])  # Bounding box
        label = model.names[cls]  # Get the class name
        print(f"Detected: {label} with confidence {confidence:.2f}")

        # Check for Plant 1
        if label == "Spider Plant" and confidence > 0.65:
            plant1_detected = True

        # Check for Plant 2
        elif label == "Purple Heart" and confidence > 0.65:
            plant2_detected = True
            
        # Draw the detection on the frame
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {confidence:.2f}", (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame, plant1_detected, plant2_detected

def main():
    try:
        while True:
            # Wait for the input pin to go HIGH
            if lgpio.gpio_read(h, INPUT_PIN) == 1:
                print("Input received. Starting camera...")
                camera.start()
                time.sleep(0.1)  # Allow camera to warm up

                start_time = time.time()
                plant1_start = None
                plant2_start = None
                plant1_confirmed = False
                plant2_confirmed = False

                while time.time() - start_time < 20:  # Monitor for 20 seconds
                    frame = camera.capture_array()
                    frame, plant1_detected, plant2_detected = detect_objects(frame)
                    cv2.imshow("Object Detection", frame)
                    
                    # Check if Plant 1 is continuously detected for 5 seconds
                    if plant1_detected:
                        if not plant1_start:
                            plant1_start = time.time()
                        elif time.time() - plant1_start >= 5:
                            plant1_confirmed = True
                            print("Spider Plant detected for 5 seconds.")
                            lgpio.gpio_write(h, OUTPUT_PIN_PLANT1, 1)
                            time.sleep(2)
                            lgpio.gpio_write(h, OUTPUT_PIN_PLANT1, 0)
                            break
                    else:
                        plant1_start = None

                    # Check if Plant 2 is continuously detected for 5 seconds
                    if plant2_detected:
                        if not plant2_start:
                            plant2_start = time.time()
                        elif time.time() - plant2_start >= 5:
                            plant2_confirmed = True
                            print("Purple Heart detected for 5 seconds.")
                            lgpio.gpio_write(h, OUTPUT_PIN_PLANT2, 1)
                            time.sleep(2)
                            lgpio.gpio_write(h, OUTPUT_PIN_PLANT2, 0)
                            break
                    else:
                        plant2_start = None

                    # Exit loop if 'Esc' is pressed
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

                # If no plant detected in 20 seconds
                if not (plant1_confirmed or plant2_confirmed):
                    print("No plant detected within 20 seconds. Sending failure signal.")
                    lgpio.gpio_write(h, OUTPUT_PIN_FAILURE, 1)
                    time.sleep(2)
                    lgpio.gpio_write(h, OUTPUT_PIN_FAILURE, 0)

                camera.stop()

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        camera.stop()
        lgpio.gpiochip_close(h)
        print("GPIO resources released.")

if __name__ == "__main__":
    main()
