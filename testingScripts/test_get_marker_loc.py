import time
import math
from threading_aruco_library import ArucoSingleTracker, load_calibration_data
from dronekit import LocationGlobalRelative, connect
import argparse
import threading

# ------------------- CONFIGURATION -------------------
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
the_connection = connect(connection_string, wait_ready=True)

# Calibration file path
calib_path = 'calibration_minicam.npz'

# -----------------------------------------------------

def camera_to_uav(x_cam, y_cam):
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east  = x_uav * s + y_uav * c
    return north, east

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return newlat, newlon

def get_current_gps(vehicle):
    location = vehicle.location.global_relative_frame
    return location.lat, location.lon, location.alt

# Load camera calibration
camera_matrix, dist_coeffs = load_calibration_data(calib_path)

# Initialize ArUco tracker (no video)
aruco_tracker = ArucoSingleTracker(
    id_to_find=3,
    marker_size=0.254,  # meters
    show_video=False,
    camera_matrix=camera_matrix,
    camera_distortion=dist_coeffs
)
        
def main():
    det_thread = threading.Thread(
        target=aruco_tracker.track,
        kwargs={'loop': True, 'show_video': False},
        daemon=True
    )
    det_thread.start()
    
    print("Starting search mission – press Ctrl+C to exit")
    
    last_seen = False
    poll_delay = 0.02   # 50 Hz polling

    try:
        while True:
            found = aruco_tracker.latest.get('found', False)
            if found and not last_seen:
                # we’ve just seen the marker
                last_seen = True
                
                # grab the camera coords
                x_cam, y_cam, _ = aruco_tracker.latest['tvec']
                
                # convert to UAV frame
                x_uav, y_uav = camera_to_uav(x_cam, y_cam)
                
                # get our current vehicle pose
                uav_loc = the_connection.location.global_relative_frame
                yaw    = the_connection.attitude.yaw
                
                # rotate into north/east
                north, east = uav_to_ne(x_uav, y_uav, yaw)
                
                # get geo coords
                lat, lon = get_location_metres(uav_loc, north, east)
                
                print(f"[Marker Detected]  Lat = {lat:.7f}, Lon = {lon:.7f}")
            
            elif not found and last_seen:
                # marker was lost – reset if you want to re-print next time it appears
                last_seen = False
            
            time.sleep(poll_delay)
    except KeyboardInterrupt:
        print("\nExiting – stopping tracker")
        aruco_tracker.stop()

if __name__ == "__main__":
    main()
