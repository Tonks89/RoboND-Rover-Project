import numpy as np


def decision_step(Rover):

    # Thresholds
    min_ahead = 25     # minimum distance ahead
    stop_areaLR = 100  # minimal num. of pixels on left and right side of robot frame to stop
    go_areaLR = 250    # minimal num. of pixels on left and right side of robot frame to start moving

    
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        
        # Count number of navigable pixels in steering direction
        angles_list = list(Rover.nav_angles) 
        navigable_indir = angles_list.count(0) 
        # Count number of pixels left & right
        navigable_L = sum(ang > 0 for ang in angles_list)
        navigable_R = sum(ang < 0 for ang in angles_list)

        # Check for Rover.mode status
        # ------------------------------> If we're in "forward" mode
        if Rover.mode == 'forward':             
            # Check the extent of navigable terrain
            # If there is enough navigable terrain and no obstacles ahead
            if navigable_L >= stop_areaLR and navigable_R >= stop_areaLR and navigable_indir >= min_ahead:  
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
            # or not enough distance ahead).
            elif navigable_L < stop_areaLR or navigable_R < stop_areaLR or navigable_indir < min_ahead: 
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # -------------------------> If we're already in "stop" mode 
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
                if navigable_L < go_areaLR or navigable_R < go_areaLR or navigable_indir < min_ahead: 
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn                  
                # If we're stopped but see sufficient navigable terrain in front then go!
                if navigable_L >= go_areaLR and navigable_R >= go_areaLR and navigable_indir >= min_ahead: 
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

