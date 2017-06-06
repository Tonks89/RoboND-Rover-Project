import numpy as np
import cv2


# Detection (thresholding)
def color_thresh(img, rgb_thresh=(160, 160, 160), b_thresh=(165,255)):
    
    # ----> Navigable terrain  
    # Create an array of zeros same xy size as img, but single channel
    navigable_select = np.zeros_like(img[:,:,0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    navigable_select[above_thresh] = 1
    
    
    # ----> Obstacles
    obstacle_select = np.zeros_like(img[:,:,0])
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[0]) \
                & (img[:,:,2] < rgb_thresh[0]) \
                & (img[:,:,0] > 0)\
                & (img[:,:,1] > 0)\
                & (img[:,:,2] > 0)
    obstacle_select[below_thresh] = 1
    
    
    # ----> Rock sample threshold
    # convert from RGB to LAB
    lab = cv2.cvtColor(img, cv2.COLOR_RGB2Lab) 
    # take b_channel (yellow)
    b_channel = lab[:,:,2] 
    # create binary image same size as original image
    rock_binary = np.zeros_like(b_channel)
    # set pixels containing yellow to 1
    rock_binary[(b_channel >= b_thresh[0]) & (b_channel <=  b_thresh[1])] = 1


    # Return the binary images      
    return navigable_select,obstacle_select,rock_binary




# Function to reduce distortion caused by perspective transform
def distortion_reduction(thresh_img, xorigin, yorigin, radius):
    
    x_thresh, y_thresh = thresh_img.nonzero()
    pix_thresh = zip(x_thresh, y_thresh)
    
    # Determine if navigable pixel is distortion or not
    for x_pixel, y_pixel in pix_thresh:
        dist = np.sqrt((x_pixel - xorigin)**2 + (y_pixel - yorigin)**2)
        
        if dist <= radius:
            # keep value
            pass
        else:
            # dismiss as distortion
            thresh_img[x_pixel][y_pixel] = 0
    
    return thresh_img


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
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * (np.pi/180)
    # Apply a rotation
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = xpos + (xpix_rot/scale)
    ypix_translated = ypos + (ypix_rot/scale)
    # Return the result  
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
    img = Rover.img

    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    # rover position is not bottom of image, but a bit offset
    bottom_offset = 6 
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable1, obstacles, rocks = color_thresh(warped)
   
    xorigin = navigable1.shape[1]/2
    yorigin = navigable1.shape[0]
    radius = 70
    navigable = distortion_reduction(navigable1, xorigin, yorigin, radius)
   

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:,:,0] = obstacles*255
    Rover.vision_image[:,:,1] = rocks*255
    Rover.vision_image[:,:,2] = navigable*255
    

    # 5) Convert map image pixel values to rover-centric coords
    navigable_xpix, navigable_ypix = rover_coords(navigable)
    obstacles_xpix, obstacles_ypix = rover_coords(obstacles)
    rocks_xpix, rocks_ypix = rover_coords(rocks)
    

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0];
    scale = 10
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix, xpos, ypos, yaw, world_size, scale)
    obstacles_x_world, obstacles_y_world = pix_to_world(obstacles_xpix, obstacles_ypix, xpos, ypos, yaw, world_size, scale)
    rocks_x_world, rocks_y_world = pix_to_world(rocks_xpix, rocks_ypix, xpos, ypos, yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1 
    Rover.worldmap[rocks_y_world, rocks_x_world, 1] += 1 
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1 


    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(navigable_xpix, navigable_ypix)
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    
 
    
    
    return Rover
