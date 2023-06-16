import cv2
import numpy as np
import time
from collections import Counter 

vid = cv2.VideoCapture(0)

def update_continuous_balls(circle, continuous_balls, position_error_margin, size_error_margin, counter_threshold=10, same_pos_counter_threshold=20):
    x, y, r = circle
    updated_continuous_balls = []
    found_match = False
    for prev_x, prev_y, prev_r, count, same_pos_count in continuous_balls:
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        size_diff = abs(r - prev_r)
        if position_diff <= position_error_margin and size_diff <= size_error_margin:
            if not found_match:
                updated_continuous_balls.append((x, y, r, 0, same_pos_count+1 if position_diff == 0 else 0))
                found_match = True
            else:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count, same_pos_count))
        else:
            if count < counter_threshold and same_pos_count < same_pos_counter_threshold:
                updated_continuous_balls.append((prev_x, prev_y, prev_r, count + 1, same_pos_count))
    if not found_match:
        updated_continuous_balls.append((x, y, r, 0, 0))
    return updated_continuous_balls


def most_frequent_points(point_list, error_margin):
    counter = Counter(point_list)
    frequent_points = [point for point, freq in counter.items() if freq > error_margin]
    return frequent_points


def update_continuous_corners(corner, continuous_corners, position_error_margin, counter_threshold=20, same_pos_counter_threshold=20):
    x, y = corner
    updated_continuous_corners = []
    found_match= False
    for (prev_x, prev_y, count, same_pos_count) in continuous_corners:
        position_diff = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        if position_diff <= position_error_margin:
            if not found_match:
                updated_continuous_corners.append((x,y,0,same_pos_count+1 if position_diff == 0 else 0))
                found_match = True
            else:
                updated_continuous_corners.append((prev_x,prev_y,count,same_pos_count))
        else:
            if count < counter_threshold and same_pos_count<same_pos_counter_threshold:
                updated_continuous_corners.append((prev_x, prev_y, count + 1, same_pos_count))

    if not found_match:
        updated_continuous_corners.append((x, y, 0, 0))

    most_frequent = most_frequent_points([(x, y) for x, y, _, _ in updated_continuous_corners], position_error_margin)
    
    return updated_continuous_corners, most_frequent if most_frequent else (x, y)



def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    det1 = (x1 - x2) * (y3 - y4)
    det2 = (y1 - y2) * (x3 - x4)
    d = det1 - det2

    if d != 0:  
        px = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / d
        py = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / d
        return (px, py)
    else:
        return None


def detect_walls(frame, continuous_corners, position_error_margin):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, minLineLength=100, maxLineGap=10)
    walls = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            walls.append(((x1, y1), (x2, y2)))
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    intersections = []

    if lines is not None:
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                intersection = line_intersection(lines[i][0], lines[j][0])
                if intersection is not None:
                    intersection = (round(intersection[0]), round(intersection[1]))  # Round the pixel values here
                    if 0 <= intersection[0] < frame.shape[1] and 0 <= intersection[1] < frame.shape[0]:
                        intersections.append(intersection)

    corners = []

    nw, ne, sw, se = (0, 0), (0, 0), (0, 0), (0, 0) 

    most_frequent_nw = most_frequent_ne = most_frequent_sw = most_frequent_se = None

    if intersections:
        nw = min(intersections, key=lambda p: p[0] + p[1])
        ne = max(intersections, key=lambda p: p[0] - p[1])
        sw = max(intersections, key=lambda p: p[1] - p[0])
        se = max(intersections, key=lambda p: p[0] + p[1])

        continuous_corners["nw"], most_frequent_nw = update_continuous_corners(nw, continuous_corners["nw"], position_error_margin)
        continuous_corners["ne"], most_frequent_ne = update_continuous_corners(ne, continuous_corners["ne"], position_error_margin)
        continuous_corners["sw"], most_frequent_sw = update_continuous_corners(sw, continuous_corners["sw"], position_error_margin)
        continuous_corners["se"], most_frequent_se = update_continuous_corners(se, continuous_corners["se"], position_error_margin)

    return frame, walls, continuous_corners, most_frequent_nw, most_frequent_ne, most_frequent_sw, most_frequent_se



def camera():
    continuous_balls = []

    continuous_corners = {
        "nw": [], 
        "ne": [], 
        "sw": [], 
        "se": []
    }

    position_error_margin = 2
    size_error_margin = 1
    frame_counter = 0
    warm_up_period = 30
    while True:
        ret, frame = vid.read()
        assert ret, "Failed to read frame from video capture."
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        gray = cv2.GaussianBlur(gray, (5, 5), 0) 

        frame, walls, continuous_corners, most_frequent_nw, most_frequent_ne, most_frequent_sw, most_frequent_se = detect_walls(frame, continuous_corners, position_error_margin)
        
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=10, minRadius=5, maxRadius=10)
                        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                if r > 0: 
                    roi = frame[y-r:y+r, x-r:x+r]
                    if roi is not None and roi.shape[0] > 0 and roi.shape[1] > 0:
                        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        thresh = cv2.adaptiveThreshold(roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                        white_pixels = cv2.countNonZero(thresh)
                        circle_area = np.pi * r**2
                        ratio = white_pixels / circle_area
                        roundness_threshold = 0.8
                        area_threshold = 0.8
                        if ratio > roundness_threshold and circle_area > area_threshold:
                            continuous_balls = update_continuous_balls((x, y, r), continuous_balls, position_error_margin, size_error_margin)
                            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

        print("Most frequent corners:")
        print("NW:", most_frequent_nw)
        print("NE:", most_frequent_ne)
        print("SW:", most_frequent_sw)
        print("SE:", most_frequent_se)
        
        cv2.imshow('frame', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # print(continuous_corners)
            print("NW:", most_frequent_nw)
            print("NE:", most_frequent_ne)
            print("SW:", most_frequent_sw)
            print("SE:", most_frequent_se)
            break

        frame_counter += 1
    vid.release()
    cv2.destroyAllWindows()


camera()
