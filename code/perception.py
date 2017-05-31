import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def yellow_detection(img, thresh=(100,50)):
    binary_output = np.zeros_like(img[:,:,0])
    r = img[:, :, 0]
    g = img[:, :, 1]
    b = img[:, :, 2]
    binary_output[(r > thresh[0]) & (g > thresh[0]) & (b < thresh[1])] = 1
    return binary_output

def obstacle_detection(img, region, thresh = 50):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    binary_output = np.zeros_like(gray)
    binary_output[(gray < thresh) & (region == 1)] = 1
    return binary_output

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    x_world = np.int_(xpos + (xpix_rot / scale))
    y_world = np.int_(ypos + (ypix_rot / scale))
    world_size = 200
    xpix_translated = np.clip(x_world, 0, world_size - 1)
    ypix_translated = np.clip(y_world, 0, world_size - 1) 
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    if Rover.picking_up == 0:
	    # 1) Define source and destination points for perspective transform
	    print(Rover.pos)
	    xpos, ypos = Rover.pos
	    yaw = Rover.yaw
	    world_size = Rover.worldmap.shape[0]
	    scale = 10

	    dst_size = 5 
	    bottom_offset = 6
	    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
	    img = Rover.img
	    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
	                              [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
	                              [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
	                              [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
	                              ])
	    # 2) Apply perspective transform
	    warped = perspect_transform(img, source, destination)
	    region_of_interest = perspect_transform(np.ones_like(img[:,:,0]), source, destination)
	    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
	    color_select = color_thresh(warped, rgb_thresh=(150, 150, 150))
	    rock_select = yellow_detection(warped)
	    obstacle_select = obstacle_detection(warped, region_of_interest, thresh=100)
	    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
	        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
	        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
	        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
	    Rover.vision_image[:,:,0] = obstacle_select * 255
	    Rover.vision_image[:,:,1] = rock_select * 255
	    Rover.vision_image[:,:,2] = color_select * 255
	    # 5) Convert map image pixel values to rover-centric coords
	    xpix, ypix = rover_coords(color_select)
	    xpix_rock, ypix_rock = rover_coords(rock_select)
	    xpix_obstacle, ypix_obstacle = rover_coords(obstacle_select)
	    # 6) Convert rover-centric pixel values to world coordinates
	    x_world_navigate, y_world_navigate = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
	    x_world_rock, y_world_rock = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, yaw, world_size, scale)
	    x_world_obstacle, y_world_obstacle = pix_to_world(xpix_obstacle, ypix_obstacle, xpos, ypos, yaw, world_size, scale)
	    # 7) Update Rover worldmap (to be displayed on right side of screen)
	        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
	        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
	        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
	    if ((Rover.pitch < 1) | (Rover.pitch > 359 )) & ((Rover.roll < 1) | (Rover.roll > 359)):
	        Rover.worldmap[y_world_navigate, x_world_navigate, 2] += 10
	        Rover.worldmap[y_world_navigate, x_world_navigate, 1] -= 10
	        Rover.worldmap[y_world_navigate, x_world_navigate, 0] -= 10
	        Rover.worldmap[y_world_rock, x_world_rock, 1] += 10
	        Rover.worldmap[y_world_rock, x_world_rock, 0] -= 10
	        Rover.worldmap[y_world_rock, x_world_rock, 2] -= 10
	        Rover.worldmap[y_world_obstacle, x_world_obstacle, 0] += 1
	        Rover.worldmap[y_world_obstacle, x_world_obstacle, 1] -= 1
	        Rover.worldmap[y_world_obstacle, x_world_obstacle, 2] -= 1
	    
	    Rover.worldmap = Rover.worldmap.clip(0, 255)
	    # 8) Convert rover-centric pixel positions to polar coordinates
	    # Update Rover pixel distances and angles
	        # Rover.nav_dists = rover_centric_pixel_distances
	        # Rover.nav_angles = rover_centric_angles
	    dists, angles = to_polar_coords(xpix, ypix)
	    # only count those with clear view
	    # try:
	    # 	dists, angles = zip(*((dist, angle) for dist, angle in zip(dists, angles) if dist > np.mean(dists)))
	    # except:
	    # 	print("ERROR!!!!!!!!!!!")
	    # 	print(dists, angles)
	    dists = np.asarray(dists)
	    angles = np.asarray(angles)
	    try:
	    	Rover.nav_dists, Rover.nav_angles = zip(*((dist, angle) for dist, angle in zip(dists, angles) if (angle <= 0.17) & (angle >= -0.17) ))
	    except:
	    	Rover.nav_dists, Rover.nav_angles = dists, angles
	    # if (angles < - 0.3).sum() < 100:
	    # 	temp_angles = angles[angles <= 0.78]
	    # 	if len(temp_angles) > 4000 :
	    # 		angles = temp_angles
	    #print((angles < - 0.3).sum(), len(angles))
	    # if (angles < - 0.3).sum() < 200:
	    # 	Rover.to_wall = True
	    # else:
	    	# Rover.to_wall = False
	    #angles = angles[angles <= 0.78]
	    #print(type(angles))
	    #Rover.nav_dists, Rover.nav_angles = dists, angles
	    #print("!!!!!!!!!!!!!!!: " + str(len(Rover.nav_angles)))
	    #print(angles)
	    #print(dists, angles)
	    #Rover.nav_dists, Rover.nav_angles = zip(*((dist, angle) for dist, angle in zip(dists, angles) if (angle <= 0) | (angle >= 0.78) ))
	    #print("!!!!!!!!!!!!!")
	    #print(Rover.nav_dists, Rover.nav_angles)
	    # 9) If rocks are nearby, store its location
	
    return Rover