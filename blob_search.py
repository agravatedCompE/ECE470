
#!/usr/bin/env python
import cv2
import numpy as np

# Params for camera calibration
theta = np.pi/180*(178.66-180) #negative if the board needs to rotate clockwise (which it does) in degrees
beta = 0.1 * 77 #77 pixels for every 10cm

tx = (83-65)/beta
ty = 307/beta
max_x = 480/beta
max_y = 640/beta

M = np.array([  [np.cos(theta), -np.sin(theta), 0, max_x/2+tx   ],
                [np.sin(theta), np.cos(theta),  0, max_y/2-ty   ],
                [0,             0,              1, 0            ],
                [0,             0,              0, 1            ]])

# Function that converts image coord to world coord
# ( blob_image_center[i][0], blob_image_center[i][1] )
def IMG2W(col, row):
    #TODO transformations
    #convert to origin from corner to center of image
    centeredCol = col - 320
    centeredRow = row - 240

    #convert origin to meters
    centeredColMeters = centeredCol / beta
    centerdRowMeters = centeredRow / beta

    #convert to
    BaseInCameraCenterFrameMeters_x = centerdRowMeters + 0.320
    BaseInCameraCenterFrameMeters_y = centeredColMeters + 0.010

    R = np.array( [ [np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]  ] )
    
    point = np.array( [ BaseInCameraCenterFrameMeters_x, BaseInCameraCenterFrameMeters_y ] )
    ans = np.matmul(R, point) /100

    ans[0] = ans[0] + 0.320
    ans[1] = ans[1] + 0.010
    #point = np.array([row, col, 0, 1])
    #ans = np.matmul(M, point)
    return ans


def blob_search(image_raw, color):
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Filter by Color
    params.filterByColor = False
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    # Filter by Circularity
    params.filterByCircularity = False
    # Filter by Inerita
    params.filterByInertia = False
    # Filter by Convexity
    params.filterByConvexity = False
    #TODO
    
    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
    #blueLower = (110,50,50) # blue lower
    #blueUpper = (130,255,255) # blue upper
    if (color == "green"):
        #RGB ~~ 115 235 120
        #HSV ~~ 60, 1, 0.5
        lower = (50,100,255*0.4) # green lower
        upper = (70,255,255) # green upper
    elif (color == "yellow"):
        #RGB ~~ 255 225 50
        #HSV ~~ 15 0.75, 1
        lower = (5,255*0.6,255*0.75) # yellow lower
        upper = (25,255*0.8,255) # yellow upper
    elif (color == "orange"):
        #HSV ~~ 30, 1, 1
        lower = (10,100,100) # orange lower
        upper = (20,255,255) # orange upper
    else:
        return 0
    
    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    keypoints = detector.detect(mask_image)
    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((round(keypoints[i].pt[0], 0), round(keypoints[i].pt[1], 0)))
    # Draw the keypoints on the detected block
    #im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, ???)
    #distance = np.sqrt( (blob_image_center[0][0] - blob_image_center[1][0])**2 + (blob_image_center[0][1] - blob_image_center[1][1])**2 )
    #print ("distance: ", distance)
    #angle = np.arctan2(blob_image_center[0][1] - blob_image_center[1][1],blob_image_center[0][0] - blob_image_center[1][0])
    #print ("angle: ", 180*angle/np.pi)
    #
    #TODO
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints,0,(255,0,0),flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
    #print(blob_image_center, round(xw_yw[0][0], 3), round(xw_yw[0][1], 3))
    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)
    cv2.waitKey(2)
    return xw_yw