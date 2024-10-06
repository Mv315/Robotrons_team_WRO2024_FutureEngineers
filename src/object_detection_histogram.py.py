import cv2
import numpy as np
import concurrent.futures
import time
import subprocess
def detect_red_histogram(image):
    
    #image = cv2.medianBlur(image, 5)
    
    #advanced preprocessing with CLAHE approach
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    enhanced_lab = cv2.merge((cl,a,b))
    enhanced_image = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)
    
    hsv = cv2.cvtColor(enhanced_image, cv2.COLOR_BGR2HSV)
    
    kernel = np.ones((5,5), np.uint8)
    hsv = cv2.morphologyEx(hsv, cv2.MORPH_CLOSE, kernel)
    hsv = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
    lower_red1 = np.array([0, 40, 40])      # Reduced saturation and value
    upper_red1 = np.array([10, 255, 255])   # Kept the same
    lower_red2 = np.array([170, 40, 40])    # Reduced saturation and value
    upper_red2 = np.array([180, 255, 255])
    time1 = time.time()
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2
    
    hist = cv2.calcHist([hsv], [0, 1], mask, [180, 256], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    
    backproj = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    
    _, thresh = cv2.threshold(backproj, 50, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    time2 = time.time()
    print(time2-time1)
    if contours and max(cv2.contourArea(c) for c in contours) > 500:
        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        return True, (x, y, w, h)
    return False, None

def detect_green_histogram(image):
    
    #image = cv2.medianBlur(image,5)
    kernel = np.ones((5,5),np.uint8)
    
    #advanced preprocessing with CLAHE approach
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    enhanced_lab = cv2.merge((cl,a,b))
    enhanced_image = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)
    
    hsv = cv2.cvtColor(enhanced_image, cv2.COLOR_BGR2HSV)

    
    #hsv = cv2.morphologyEx(hsv, cv2.MORPH_CLOSE, kernel)
    #hsv = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
    # Two ranges for green
    lower_green1 = np.array([40, 40, 40])    # Reduced saturation and value
    upper_green1 = np.array([80, 255, 255])  # Kept the same
    lower_green2 = np.array([35, 40, 40])    # Reduced saturation and value
    upper_green2 = np.array([45, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_green1, upper_green1)
    mask2 = cv2.inRange(hsv, lower_green2, upper_green2)
    mask = cv2.bitwise_or(mask1,mask2)
    
    hist = cv2.calcHist([hsv], [0, 1], mask, [180, 256], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    
    backproj = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    
    _, thresh = cv2.threshold(backproj, 5, 255, cv2.THRESH_BINARY)#threshold=40(original),0worksonlyforsleepingone
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours and max(cv2.contourArea(c) for c in contours) > 500:
        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        return True, (x, y, w, h)
    return False, None
def main():
    libcamera_cmd = ['libcamera-vid', '--inline', '-t', '0', '--width', '640', '--height', '480', '--codec', 'mjpeg', '-o', '-']
    camera_process = subprocess.Popen(libcamera_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    mjpeg_buffer = b""
    while True:
        buffer = camera_process.stdout.read(1024)
        if not buffer:
                break
        mjpeg_buffer += buffer
        start = mjpeg_buffer.find(b'\xff\xd8')
        end = mjpeg_buffer.find(b'\xff\xd9')
        if start != -1 and end != -1 and start < end:
                jpg_data = mjpeg_buffer[start:end+2]
                mjpeg_buffer = mjpeg_buffer[end+2:]
                image = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future_red = executor.submit(detect_red_histogram, image)
                future_green = executor.submit(detect_green_histogram, image)
        
                red_detected, red_box = future_red.result()
                green_detected, green_box = future_green.result()
    
            if red_detected and not green_detected:
                print("Red is detected")
                cv2.rectangle(image, (red_box[0], red_box[1]), (red_box[0] + red_box[2], red_box[1] + red_box[3]), (0, 0, 255), 2)
                cv2.putText(image, "Red", (red_box[0], red_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            elif green_detected and not red_detected:
                print("Green is detected")
                cv2.rectangle(image, (green_box[0], green_box[1]), (green_box[0] + green_box[2], green_box[1] + green_box[3]), (0, 255, 0), 2)
                cv2.putText(image, "Green", (green_box[0], green_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            elif red_detected and green_detected:
                print("Both red and green are detected")
                cv2.rectangle(image, (red_box[0], red_box[1]), (red_box[0] + red_box[2], red_box[1] + red_box[3]), (0, 0, 255), 2)
                cv2.putText(image, "Red", (red_box[0], red_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                cv2.rectangle(image, (green_box[0], green_box[1]), (green_box[0] + green_box[2], green_box[1] + green_box[3]), (0, 255, 0), 2)
                cv2.putText(image, "Green", (green_box[0], green_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            else:
                print("Neither red nor green is detected")
    
            cv2.imshow('Detected Objects', image)
            cv2.waitKey(0) #press a key to move on with sequential execution
            cv2.destroyAllWindows()

if __name__ == "__main__":
    
    main()
