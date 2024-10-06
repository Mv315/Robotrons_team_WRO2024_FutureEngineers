import cv2
import numpy as np
import subprocess
import io
from PIL import Image
import time

def get_frame(camera_process):
    mjpeg_buffer = b''
    while True:
        chunk = camera_process.stdout.read(1024)
        if not chunk:
            break
        mjpeg_buffer += chunk
        a = mjpeg_buffer.find(b'\xff\xd8')
        b = mjpeg_buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = mjpeg_buffer[a:b+2]
            mjpeg_buffer = mjpeg_buffer[b+2:]
            return cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

def calibrate():
    # Initialize libcamera-vid
    libcamera_cmd = ['libcamera-vid', '--inline', '-t', '0', '--width', '640', '--height', '480', '--codec', 'mjpeg', '-o', '-']
    camera_process = subprocess.Popen(libcamera_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Known object size in cm
    real_size = float(input("Enter the real size of the object in cm: "))

    pixel_sizes = []
    distances = []

    print("Move the object to different distances and press 'c' to capture, 'q' to quit.")

    while True:
        frame = get_frame(camera_process)
        cv2.imshow('Frame', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow('Detected Object', frame)

                pixel_size = max(w, h)
                pixel_sizes.append(pixel_size)
                
                distance = float(input("Enter the distance of the object from the camera in cm: "))
                distances.append(distance)

                print(f"Captured: Distance = {distance} cm, Pixel Size = {pixel_size}")

        elif key == ord('q'):
            break

    camera_process.terminate()
    cv2.destroyAllWindows()

    # Calculate the relation
    pixel_sizes = np.array(pixel_sizes)
    distances = np.array(distances)
    
    # Fit a curve: pixel_size = a * (distance^b)
    log_distances = np.log(distances)
    log_pixel_sizes = np.log(pixel_sizes)
    coeffs = np.polyfit(log_distances, log_pixel_sizes, 1)
    a = np.exp(coeffs[1])
    b = coeffs[0]

    print(f"\nCalibration complete.")
    print(f"Relation: pixel_size = {a:.4f} * (distance^{b:.4f})")
    print(f"To get distance from pixel size: distance = (pixel_size / {a:.4f})^(1/{b:.4f})")
    
    return a, b, real_size

def estimate_distance(pixel_size, a, b, real_size):
    # Estimate distance based on pixel size
    estimated_distance = (pixel_size / a) ** (1/b)
    
    # Calculate real-world size at this distance
    real_world_size = (real_size * pixel_size) / (a * (estimated_distance ** b))
    
    return estimated_distance, real_world_size

if __name__ == "__main__":
    a, b, real_size = calibrate()

    while True:
        pixel_size = float(input("Enter pixel size (or -1 to quit): "))
        if pixel_size == -1:
            break
        distance, size = estimate_distance(pixel_size, a, b, real_size)
        print(f"Estimated distance: {distance:.2f} cm")
        print(f"Estimated real-world size: {size:.2f} cm")
