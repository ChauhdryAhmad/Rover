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


## function for partail horizontal line 

# def complete_line(image, segments, x_threshold=10):

#     # Sort line segments based on the minimum x-coordinate of each segment's endpoints
#     segments = sorted(segments, key=lambda segment: min(segment[0][0], segment[1][0]))

#     # List to hold points for complete line
#     complete_line_points = [segments[0][0], segments[0][1]]

#     # Iterate over sorted segments to find and connect close segments
#     for i in range(1, len(segments)):
#         # Get previous end point and current segment's start and end points
#         prev_point = complete_line_points[-1]
#         start_point, end_point = segments[i]

#         # Check if the start point is close enough to the previous endpoint
#         if abs(prev_point[0] - start_point[0]) <= x_threshold:
#             # Extend the line by connecting this segment
#             complete_line_points.append(start_point)
#             complete_line_points.append(end_point)

#     # Draw the complete line by connecting all points
#     for i in range(len(complete_line_points) - 1):
#         cv2.line(image, complete_line_points[i], complete_line_points[i+1], (0, 0, 255), 2)  # Red line

#     # Display or save the result
#     cv2.imshow("Completed Line", image)
#     cv2.waitKey(1000)
#     cv2.destroyAllWindows()

# Directories
to_process_dir = './images'
laser_detected_dir = './laser_detected'
coordinates_dir = './coordinates_detected'

# Create output directories if they don't exist
os.makedirs(laser_detected_dir, exist_ok=True)
os.makedirs(coordinates_dir, exist_ok=True)


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

            print(filename,"x : ", x_axis, " , y : ", y_axis)

                        # Draw a bold red dot at the (x_axis, y_axis) coordinates and label them
            if y_axis is not None and x_axis is not None:
                # Draw a bold red dot at the intersection point
                cv2.circle(result_image, (x_axis, y_axis), 5, (0, 0, 255), -1)  # Red dot

                # Write the x and y coordinates near the dot
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(result_image, f"({x_axis}, {y_axis})", (x_axis + 10, y_axis - 10),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)


            # Save the final image with completed lines
            coordinates_detected_path = os.path.join(coordinates_dir, filename)
            cv2.imwrite(coordinates_detected_path, result_image)

            cv2.imshow("Completed Lines", result_image)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

            print(f"Processed and saved: {filename}")


process_images()