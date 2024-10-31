import os
import cv2
from ultralytics import YOLO

# Load the pre-trained YOLOv5 model
model = YOLO('yolov5su.pt')  # Replace with your YOLOv5 model path

# Global variables
select_person = False
frames = []  # To store the frames loaded

def get_person_from_click(event, x, y, flags, param):
    global select_person
    if event == cv2.EVENT_LBUTTONDOWN:
        select_person = True
        detect_and_draw_person(x, y)

def detect_and_draw_person(x, y):
    global frames
    # Run detection on the current frame
    results = model(frames[0])  # Perform inference on the first frame
    detections = results[0].boxes

    # Loop through all detected boxes
    for box in detections:
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Get the bounding box coordinates
        cls = int(box.cls[0])  # Class ID

        if cls == 0:  # Assuming '0' is the class ID for persons
            # Check if the click (x, y) is inside the bounding box
            if x_min <= x <= x_max and y_min <= y <= y_max:
                # Draw a rectangle around the detected person
                cv2.rectangle(frames[0], (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.imshow('Selected Person', frames[0])  # Show the updated frame

                # Optionally, save the cropped image of the detected person
                cropped_image = frames[0][y_min:y_max, x_min:x_max]
                cv2.imwrite('selected_person.jpg', cropped_image)  # Save the cropped image
                print("Cropped image saved as 'selected_person.jpg'.")
                break



def main(data_dir):
    global frames
    # Load frames from the specified directory
    frames = [cv2.imread(os.path.join(data_dir, f'f_{i}.jpeg')) for i in range(1, 3)]
    
    if not frames:
        print("No frames to select perosn!")
        return

    # Process the first frame for user selection
    cv2.namedWindow('Select Person', cv2.WINDOW_NORMAL)


    while not select_person: 

        # Detect persons in the first frame
        first_frame_results = model(frames[0])  # Perform inference on the first frame
        detections = first_frame_results[0].boxes

        # Draw bounding boxes for detected persons
        for box in detections:
            x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Bounding box coordinates
            cls = int(box.cls[0])

            if cls == 0:  # Assuming '0' is the class ID for persons
                cv2.rectangle(frames[0], (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

        cv2.imshow('Select Person', frames[0])

        # Set mouse callback for clicking on the person
        cv2.setMouseCallback('Select Person', get_person_from_click)

        if cv2.waitKey(1000) & 0xFF == ord('q'):
            break
        if (select_person) :
            break

    cv2.destroyWindow('Select Person')

if __name__ == "__main__":
    data_directory = 'data'  # Change to your data directory containing the frames
    main(data_directory)
