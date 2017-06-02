import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    min_ahead = 25 ###*
    #Rover.stop_forward = 200 ###*  
    
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        
        # Number of navigable pixels in steering direction
        angles_list = list(Rover.nav_angles) ###*
        navigable_indir = angles_list.count(0) ###*
        
        
        # Navigable distance in boundaries of field of view
        #dist_list = list(Rover.nav_dists)
        
        #b1_angle = min(angles_list)
        #b2_angle = max(angles_list)
        #b1_idx = angles_list.index(b1_angle)
        #b2_idx = angles_list.index(b2_angle)
         
        
        #b1_dist = dist_list[b1_idx]
        #b2_dist = dist_list[b2_idx]
          
            
     
        
       
        # Number of navigable, continuos pixels in steering direction
        
        
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            
            # Check the extent of navigable terrain
            # If there is enough navigable terrain and no obstacles ahead
            if len(Rover.nav_angles) >= Rover.stop_forward and navigable_indir >= min_ahead:  ###*
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode (not enough area to move, 
            # not enough distance ahead).
            elif len(Rover.nav_angles) < Rover.stop_forward  or navigable_indir < min_ahead: ###*
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                # If there's a lack of navigable terrain turn!
                if len(Rover.nav_angles) < Rover.go_forward or navigable_indir < min_ahead: ###*
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #Rover.steer = -15 # Could be more clever here about which way to turn*****
                    
                    
                    # Turning angle: angle w/ highest num. navigable pix.
                    # Note if several dirs with = num pixels it will turn toward first in scanned_angles
                    angles_int = np.rint(Rover.nav_angles) # convert float to int
                    angles_int_list = list(angles_int.astype(int))
                    
    
                    
                    b1_angle = min(angles_int_list)
                    b2_angle = max(angles_int_list)
       
                    npix_all_angles = []
                    scanned_angles = list(range(b1_angle, b2_angle, 1))
                    
                    if 0 in scanned_angles:
                        scanned_angles.remove(0) # otherwise it wont turn and be stuck

                    for ang in scanned_angles: # every 5 degrees check number of navig pixel
                        npix_angle = angles_int_list.count(ang)
                        npix_all_angles.append(npix_angle)
                        
                    idx_turn = npix_all_angles.index(max(npix_all_angles)) #get idx of angle with most pix
                    dir_turn = scanned_angles[idx_turn] #get corresponding angle
                    Rover.steer = dir_turn
                    
                    # Decide best direction to steer
                    #if b1_dist > b2_dist: # Terrain looks better in b1 direction
                        
                        #if b1_angle >= -15 and b1_angle <= 15:
                            #Rover.steer = b1_angle
                        #elif b1_angle < -15:
                            #Rover.steer = -15
                        #else:
                            #Rover.steer = 15
                                                        
                    #elif b2_dist > b1_dist: # Terrain looks better in b2 direction
                        #if b2_angle >= -15 and b2_angle <= 15:
                            #Rover.steer = b2_angle
                        #elif b2_angle < -15:
                            #Rover.steer = -15
                        #else:
                            #Rover.steer = 15  
                            
                    #else:
                            #Rover.steer = -15
                        
                        #pass # if dist are same check again if we can forward
                        
                    
                    
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward and navigable_indir >= min_ahead: ###*
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

