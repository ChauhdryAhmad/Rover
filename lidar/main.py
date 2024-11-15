import cv2
import numpy as np
import os

def detect_laser_line(image):
    # Step 1: Convert image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Step 2: Define range for red color (both light and dark red)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Step 3: Create masks for red color and combine them
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Step 4: Threshold to create a binary image for line detection
    binary = cv2.threshold(red_mask, 50, 255, cv2.THRESH_BINARY)[1]

    # Step 5: Use Hough Line Transform to detect lines
    # (Adjust parameters for your image resolution and line visibility)
    lines = cv2.HoughLinesP(binary, rho=1, theta=np.pi / 180, threshold=1, minLineLength=5, maxLineGap=100)

    # Step 6: Draw detected lines that meet the red color criterion
    points = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Draw the line on the original image
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green line for visualization
            a_point = (x1, y1)
            b_point = (x2, y2)
            points.append((a_point, b_point))


    return image, points



def complete_vertical_line(image, segments, y_threshold=10):
    # Sort line segments based on the minimum y-coordinate of each segment's endpoints
    segments = sorted(segments, key=lambda segment: min(segment[0][1], segment[1][1]))

    # Initialize a list to store x-coordinates where the vertical line should be drawn
    complete_x_coords = [min(segments[0][0][0], segments[0][1][0])]

    # Iterate over sorted segments to find and connect close y-coordinates
    for i in range(1, len(segments)):
        # Get the x-coordinate of the segment closest to the y-axis condition
        current_x = min(segments[i][0][0], segments[i][1][0])
        
        # Check if the y-coordinate is close enough to the previous y-coordinate
        if abs(complete_x_coords[-1] - current_x) <= y_threshold:
            # Add the x-coordinate to the list if it's within the threshold
            complete_x_coords.append(current_x)

    # Calculate the average x-coordinate for the line position
    if complete_x_coords:
        average_x = int(sum(complete_x_coords) / len(complete_x_coords))
    else:
        average_x = 0  # Default in case no coordinates found within threshold

    # Draw a vertical line at the calculated average x position
    image_height = image.shape[0]
    cv2.line(image, (average_x, 0), (average_x, image_height), (0, 0, 255), 2)  # Red vertical line

    return average_x

# funtion for complete horizontal line 

def complete_horizontal_line(image, segments, x_threshold=10):
    # Sort line segments based on the minimum x-coordinate of each segment's endpoints
    segments = sorted(segments, key=lambda segment: min(segment[0][0], segment[1][0]))

    # Initialize a list to store y-coordinates where the horizontal line should be drawn
    complete_y_coords = [min(segments[0][0][1], segments[0][1][1])]

    # Iterate over sorted segments to find and connect close x-coordinates
    for i in range(1, len(segments)):
        # Get the y-coordinate of the segment closest to the x-axis condition
        current_y = min(segments[i][0][1], segments[i][1][1])
        
        # Check if the x-coordinate is close enough to the previous x-coordinate
        if abs(complete_y_coords[-1] - current_y) <= x_threshold:
            # Add the y-coordinate to the list if it's within the threshold
            complete_y_coords.append(current_y)

    # Calculate the average y-coordinate for the line position
    if complete_y_coords:
        average_y = int(sum(complete_y_coords) / len(complete_y_coords))
    else:
        average_y = 0  # Default in case no coordinates found within threshold

    # Draw a horizontal line at the calculated average y position
    image_width = image.shape[1]
    cv2.line(image, (0, average_y), (image_width, average_y), (0, 0, 255), 2)  # Red horizontal line

    return average_y

# Directories
to_process_dir = './images'
laser_detected_dir = './laser_detected'
coordinates_dir = './coordinates_detected'

# Create output directories if they don't exist
os.makedirs(laser_detected_dir, exist_ok=True)
os.makedirs(coordinates_dir, exist_ok=True)


def predictDistanceAccTo_x_Cordinates(x):
    return (-2.449667700795079e-05 * x**3 
            + 0.07483786812947929 * x**2 
            - 76.28786215573622 * x 
            + 25978.577127861463)

def predictDistanceAccTo_y_Cordinates(y):
    return (2.8138839924614784e-05 * y**3 
            - 0.025410657787418665 * y**2 
            + 7.595435158317794 * y 
            - 705.4154291595778)

def process_images():
    for filename in os.listdir(to_process_dir):
        # Ensure we only process image files (skip directories or other file types)
        if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
            # Construct full path for the image
            image_path = os.path.join(to_process_dir, filename)
            image = cv2.imread(image_path)

            if image is None:
                print(f"Could not read image {filename}. Skipping.")
                continue

            # Detect laser line
            result_image, points = detect_laser_line(image)
            
            # Save the result after laser line detection
            laser_detected_path = os.path.join(laser_detected_dir, filename)
            cv2.imwrite(laser_detected_path, result_image)

            # Draw complete horizontal and vertical lines
            y_axis = complete_horizontal_line(result_image, points)
            x_axis = complete_vertical_line(result_image, points)

            print(filename, "x:", x_axis, ", y:", y_axis)

            # Calculate distances based on x and y coordinates
            if x_axis is not None and y_axis is not None:
                dist_x = predictDistanceAccTo_x_Cordinates(x_axis)
                dist_y = predictDistanceAccTo_y_Cordinates(y_axis)
                print(filename, "x_dist :", dist_x, ", y_dist :", dist_y)


                # Draw a bold red dot at the (x_axis, y_axis) coordinates
                cv2.circle(result_image, (x_axis, y_axis), 5, (0, 0, 255), -1)  # Red dot

                # Write the x and y coordinates along with distances near the dot
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(result_image, f"({x_axis}, {y_axis})", (x_axis + 10, y_axis - 10),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                # Write dist_x and dist_y in the top-right corner of the image
                text_x = result_image.shape[1] - 300  # Adjust based on text width if needed
                text_y_start = 30  # Starting y-position for the first line of text

                cv2.putText(result_image, f"dist_x: {dist_x:.2f}", (text_x, text_y_start),
                            font, 1, (0, 255, 0), 1, cv2.LINE_AA)  # Green text for dist_x
                cv2.putText(result_image, f"dist_y: {dist_y:.2f}", (text_x, text_y_start + 50),
                            font, 1, (255, 0, 0), 1, cv2.LINE_AA)  # Blue text for dist_y

            # Save the final image with completed lines and distances
            coordinates_detected_path = os.path.join(coordinates_dir, filename)
            cv2.imwrite(coordinates_detected_path, result_image)

            cv2.imshow("Completed Lines with Distances", result_image)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

            print(f"Processed and saved: {filename} \n\n")

process_images()
