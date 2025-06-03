import cv2
from cv2 import aruco
import numpy as np
import os

CHECKERBOARD_ROWS = 7
CHECKERBOARD_COLS = 6
CHARUCO_BOARD_COLS = 15
CHARUCO_BOARD_ROWS = 9

SQUARE_SIZE = 0.108
MARKER_SIZE = 0.01884

objp = np.zeros((CHECKERBOARD_ROWS * CHECKERBOARD_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_COLS, 0:CHECKERBOARD_ROWS].T.reshape(-1, 2)
objp *= SQUARE_SIZE



aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

charuco_board = cv2.aruco.CharucoBoard(
    (CHARUCO_BOARD_COLS, CHARUCO_BOARD_ROWS),
    SQUARE_SIZE,
    MARKER_SIZE,
    aruco_dict
)
def create_board():
    board_size = (3600, 5400)  # Output image size in pixels (width, height)
    charuco_board_image = charuco_board.generateImage(board_size)

    bordered_board = cv2.copyMakeBorder(
    charuco_board_image,
    top=100, bottom=100, left=100, right=100,
    borderType=cv2.BORDER_CONSTANT,
    value=255  # White color
    )

    # Save the board image to a file
    cv2.imwrite("charuco_board.png", bordered_board)

    # Display the board image
    cv2.imshow("Charuco Board", bordered_board)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

CharucoDetector = cv2.aruco.CharucoDetector(charuco_board)

charuco_corners = None
charuco_ids = None
marker_corners = None
marker_ids = None
all_charuco_corners = []
all_charuco_ids = []
image_size = (3840,2160)

def create_output():
        os.makedirs('./processed/', exist_ok=True)




def charuco_read(images):
    i = 1
    for image in images:
        print(f"Processing image {i}/{len(images)} {format(image)}")
        i = i + 1
        image_data = cv2.imread(image)
        if image_data is None:
            exit("no image")
        charuco_corners, charuco_ids, marker_corners, marker_ids = CharucoDetector.detectBoard(image_data)
        if charuco_corners is not None and charuco_ids is not None:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)
            
            #simpky for visualization purposes
            #red squares for aruco markers
            #blue squares for checkerboard corners
            #only internal checkboard corners count
            cv2.aruco.drawDetectedMarkers(image_data, marker_corners, marker_ids)

            #Draw center of marker based on average corners
            x = (marker_corners[i-1][0][0][0] + marker_corners[i-1][0][1][0] + marker_corners[i-1][0][2][0] + marker_corners[i-1][0][3][0]) / 4
            y = (marker_corners[i-1][0][0][1] + marker_corners[i-1][0][1][1] + marker_corners[i-1][0][2][1] + marker_corners[i-1][0][3][1]) / 4
            cv2.circle(image_data, (int(x), int(y)), 5, (0, 255, 0), -1)  


            #cv2.aruco.drawDetectedCornersCharuco(image_data, charuco_corners, charuco_ids)
            for corner, charuco_id in zip(charuco_corners, charuco_ids):
                for point in corner:
                    x, y = point.ravel()
                    cv2.circle(image_data, (int(x), int(y)), 5, (0, 0, 255), -1)  
                    
        else:
            print("NULL corners")
        
        processed_path = f"./processed/{os.path.basename(image)}"
        cv2.imwrite(processed_path, image_data)
    
    

    
def charuco_calibrate():
        object_points = []  #3D points in Charuco board coordinate space
        image_points = []   #Corresponding 2D points in image coordinate space

        
        for charuco_corners, charuco_ids in zip(all_charuco_corners, all_charuco_ids):
            if charuco_corners is not None and charuco_ids is not None:
                obj_pts, img_pts = charuco_board.matchImagePoints(charuco_corners, charuco_ids)

                object_points.append(obj_pts)
                image_points.append(img_pts)

        if len(object_points) == 0 or len(image_points) == 0:
            print("No object points")
            exit()

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,  # 3D points
        image_points,   # 2D points
        image_size,     # (width, height)
        None,           # Initial camera matrix
        None            # Initial distortion coefficients
        )

     

        #Reprojection images Debugging only
        #This shows the calibration array of image points ontop of detected image points
        #Pretty much the finished camera calibration's interpretation of the edges vs the actual detected edges
        #Comes with some nuance, because how do we know the detected edges are completely accurate
        #The more centered the blue is on the green dot, the closer the calbiratio is to aruco detection levels
        for i, (image_path, obj_pts, img_pts) in enumerate(zip(images, object_points, image_points)):
            print(f"Reprojection image {i + 1}/{len(images)}: {image_path}")
            image_data = cv2.imread(image_path)
            if image_data is None:
                print(f"Failed to load image {image_path}. Skipping...")
                continue

            #Reproject the 3D object points into the 2D image plane
            reprojected_points, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)

            #detected points (green)
            for point in img_pts:
                x, y = point.ravel()
                cv2.circle(image_data, (int(x), int(y)), 5, (0, 255, 0), -1)  

            for detected_point, reprojected_point in zip(img_pts, reprojected_points):
                x1, y1 = detected_point.ravel()  # Detected point
                x2, y2 = reprojected_point.ravel()  # Reprojected point

                # Apply scaling factor to the line vector
                x2_scaled = x1 + (x2 - x1) * 10
                y2_scaled = y1 + (y2 - y1) * 10

                # Draw the reprojected point
                cv2.circle(image_data, (int(x2), int(y2)), 3, (255, 0, 0), -1)  # Blue for reprojected points

                # Draw a vector line indicating the scaled offset
                cv2.line(image_data, (int(x1), int(y1)), (int(x2_scaled), int(y2_scaled)), (0, 0, 255), 1)  # Red line for vector


            processed_path = f"./processed/reprojection_{os.path.basename(image_path)}"
            cv2.imwrite(processed_path, image_data)

        # Output results
        print("---------------------------------------")
        print("Calibration Successful! \nRMSE: ", ret)
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
        np.savez('calibration_data_live.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)



object_points = []
image_points = []

def checkerboard_read(images):
    for i, image in enumerate(images):
        print(f"Processing image {i+1}/{len(images)}: {image}")
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (6, 7), None)

        if ret:
            #refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

            image_points.append(corners2)
            object_points.append(objp)

            #Save
            cv2.drawChessboardCorners(img, (CHECKERBOARD_COLS, CHECKERBOARD_ROWS), corners2, ret)
            processed_path = f"./processed/{os.path.basename(image)}"
            cv2.imwrite(processed_path, img)
        else:
            print("Checkerboard not detected")

    
def checkerboard_calibrate():

        if len(object_points) == 0 or len(image_points) == 0:
            print("No object points")
            exit()

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None
        )
     

        #Reprojection images Debugging only
        #This shows the calibration array of image points ontop of detected image points
        #Pretty much the finished camera calibration's interpretation of the edges vs the actual detected edges
        #Comes with some nuance, because how do we know the detected edges are completely accurate
        #The more centered the blue is on the green dot, the closer the calbiratio is to aruco detection levels
        for i, (image_path, obj_pts, img_pts) in enumerate(zip(images, object_points, image_points)):
            print(f"Reprojection image {i + 1}/{len(images)}: {image_path}")
            image_data = cv2.imread(image_path)
            if image_data is None:
                print(f"Failed to load image {image_path}. Skipping...")
                continue

            #Reproject the 3D object points into the 2D image plane
            reprojected_points, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)

            #detected points (green)
            for point in img_pts:
                x, y = point.ravel()
                cv2.circle(image_data, (int(x), int(y)), 5, (0, 255, 0), -1)  

            for detected_point, reprojected_point in zip(img_pts, reprojected_points):
                x1, y1 = detected_point.ravel()  # Detected point
                x2, y2 = reprojected_point.ravel()  # Reprojected point

                # Apply scaling factor to the line vector
                x2_scaled = x1 + (x2 - x1) * 10
                y2_scaled = y1 + (y2 - y1) * 10

                # Draw the reprojected point
                cv2.circle(image_data, (int(x2), int(y2)), 3, (255, 0, 0), -1)  # Blue for reprojected points

                # Draw a vector line indicating the scaled offset
                cv2.line(image_data, (int(x1), int(y1)), (int(x2_scaled), int(y2_scaled)), (0, 0, 255), 1)  # Red line for vector


            processed_path = f"./processed/reprojection_{os.path.basename(image_path)}"
            cv2.imwrite(processed_path, image_data)

        # Output results
        print("---------------------------------------")
        print("Calibration Successful! \nRMSE: ", ret)
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
        np.savez('calibration_data_live.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)



datadir = "./output/"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png") ])
order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
images = images[order]
#create_board()
#create_output()
#charuco_read(images)
#charuco_calibrate()

checkerboard_read(images)
checkerboard_calibrate()