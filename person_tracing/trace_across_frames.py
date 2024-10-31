import cv2
import numpy as np
import os
from ultralytics import YOLO
from keras.applications import MobileNetV2
from keras.applications.mobilenet_v2 import preprocess_input
from keras.models import Model

# Load the pre-trained YOLOv5 model
model = YOLO('yolov5su.pt')  # Replace with your YOLOv5 model path
os.environ["QT_QPA_PLATFORM"] = "xcb"


# Load MobileNetV2 model for feature extraction
base_model = MobileNetV2(weights='imagenet', include_top=False, pooling='avg')
feature_extractor = Model(inputs=base_model.input, outputs=base_model.output)

def extract_features(image):
    """Extract features from the given image using MobileNetV2."""
    image_resized = cv2.resize(image, (224, 224))  # Resize to match model input
    image_preprocessed = preprocess_input(image_resized)  # Preprocess for MobileNetV2
    features = feature_extractor.predict(np.expand_dims(image_preprocessed, axis=0))
    return features.flatten()  # Flatten the feature vector

def calculate_similarity(cropped_image, frame_image):
    """Calculate similarity using cosine similarity between two feature vectors."""
    features_cropped = extract_features(cropped_image)  # Extract features for cropped image
    features_frame = extract_features(frame_image)      # Extract features for frame image

    # Calculate cosine similarity
    similarity_score = np.dot(features_cropped, features_frame) / (np.linalg.norm(features_cropped) * np.linalg.norm(features_frame))
    return similarity_score

def detect_and_draw_person_boxes(frame, selected_person_image):
    # Perform inference on the given frame
    results = model(frame)  # Perform inference on the current frame
    detections = results[0].boxes

    max_similarity = 0
    max_sim_box_coords = None

    # Draw bounding boxes for detected persons
    for box in detections:
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Bounding box coordinates
        cls = int(box.cls[0])

        if cls == 0:  # Assuming '0' is the class ID for persons
            # Crop the image of the detected person for similarity calculation
            cropped_image = frame[y_min:y_max, x_min:x_max]

            # Calculate similarity with the selected person image
            similarity = calculate_similarity(selected_person_image, cropped_image)
            if similarity > max_similarity:
                max_similarity = similarity
                max_sim_box_coords = (x_min, y_min, x_max, y_max)

    # Draw the rectangle around the person with maximum similarity
    if max_sim_box_coords:
        x_min, y_min, x_max, y_max = max_sim_box_coords
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
        cv2.putText(frame, f"Similarity: {max_similarity:.2f}", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return frame

def main(data_dir):
    # Load frames from the specified directory
    frames = [cv2.imread(os.path.join(data_dir, f'f_{i}.jpeg')) for i in range(1, 3)]
    if not frames:
        print("No frames found.")
        return

    # Load the saved image of the selected person (assuming it is saved as 'selected_person.jpg')
    selected_person_image = cv2.imread('selected_person.jpg')
    if selected_person_image is None:
        print("Selected person image not found.")
        return

    # Process each frame and draw rectangle on the detected person with the highest similarity
    for i, frame in enumerate(frames):
        frame_with_box = detect_and_draw_person_boxes(frame, selected_person_image)

        # Display the frame with the drawn box
        cv2.imshow(f"Frame {i+1}", frame_with_box)
        cv2.waitKey(1000)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    data_directory = 'data'  # Change to your data directory containing the frames
    main(data_directory)
