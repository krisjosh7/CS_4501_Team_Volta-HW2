import cv2
import numpy as np
import yaml
import csv
import math
import sys
from scipy import interpolate

# --- CONFIGURATION ---
# Optimization Parameters
LOOKAHEAD = 5        # Smoothing window (increase for smoother paths)
MAX_VELOCITY = 7.0   # m/s
MIN_VELOCITY = 2.0   # m/s
MAX_LATERAL_ACCEL = 3.0 # m/s^2 (Friction limit)

def load_map(map_name):
    # Load YAML
    with open(f"{map_name}.yaml", 'r') as f:
        map_data = yaml.safe_load(f)
    
    # Load Image
    image_path = map_data['image']
    resolution = map_data['resolution']
    origin = map_data['origin'] # [x, y, z]
    
    # Read image in grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Threshold: Free space is usually 254/255, Occupied is 0
    # We want the track to be white (255), walls black (0)
    _, thresh = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
    
    return thresh, resolution, origin

def extract_centerline(img):
    # 1. Distance Transform (find distance to nearest wall for every pixel)
    dist_transform = cv2.distanceTransform(img, cv2.DIST_L2, 5)
    
    # 2. Skeletonize (This is a simplified approach using thinning)
    # We threshold the distance map to keep only the "deepest" parts of the track
    # Ideally use skimage.morphology.skeletonize, but here is a cv2 fallback:
    # A robust way for racing tracks is to threshold the distance transform
    # at a high value, then thin it.
    
    # normalize to 0-255
    norm_dist = cv2.normalize(dist_transform, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    # Initial skeleton: threshold top 90% of distance (center of track)
    # Note: This depends on track width consistency. 
    _, skeleton_rough = cv2.threshold(norm_dist, 150, 255, cv2.THRESH_BINARY)
    
    # Thinning to get 1-pixel wide line
    skeleton = cv2.ximgproc.thinning(skeleton_rough) if hasattr(cv2, 'ximgproc') else skeleton_rough
    
    return skeleton

def sort_points_to_path(skeleton_img, resolution, origin):
    # Extract (y, x) coordinates of white pixels
    pixels = np.column_stack(np.where(skeleton_img > 0))
    
    if len(pixels) == 0:
        print("Error: No path found. Check map thresholding.")
        return []

    # Nearest Neighbor Sort to create a line from scattered pixels
    path_pixels = [pixels[0]]
    pixels = np.delete(pixels, 0, axis=0)
    
    while len(pixels) > 0:
        last_pt = path_pixels[-1]
        # Find closest remaining pixel
        dists = np.sum((pixels - last_pt)**2, axis=1)
        nearest_idx = np.argmin(dists)
        
        # If jump is too big, we might have hit a different part of the track 
        # (handle loops or disconnected skeletons)
        if dists[nearest_idx] > 50: # 50 pixels jump threshold
             break
             
        path_pixels.append(pixels[nearest_idx])
        pixels = np.delete(pixels, nearest_idx, axis=0)

    # Convert Pixels to Meters (World Coordinates)
    # Map origin is usually bottom-left. Image is top-left.
    # origin: [x_origin, y_origin, theta]
    
    world_path = []
    height, _ = skeleton_img.shape
    
    for p in path_pixels:
        r, c = p[0], p[1] # row (y), col (x) in image
        
        # Image Frame -> Map Frame transformation
        # x_map = c * res + origin_x
        # y_map = (height - 1 - r) * res + origin_y  (Flip Y because image y goes down)
        
        x_world = c * resolution + origin[0]
        y_world = (height - 1 - r) * resolution + origin[1]
        
        world_path.append([x_world, y_world])
        
    return np.array(world_path)

def optimize_velocity(path):
    # Calculate curvature and velocity for each point
    new_path = []
    
    for i in range(len(path)):
        p_curr = path[i]
        p_prev = path[i-LOOKAHEAD] # Wrap around handled by python negative indexing
        p_next = path[(i+LOOKAHEAD) % len(path)]
        
        # 1. Calculate Curvature (Menger Curvature) of the triangle formed by 3 points
        # Triangle area = 0.5 * |x1(y2-y3) + x2(y3-y1) + x3(y1-y2)|
        x1, y1 = p_prev
        x2, y2 = p_curr
        x3, y3 = p_next
        
        area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
        
        # Side lengths
        a = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        b = math.sqrt((x2-x3)**2 + (y2-y3)**2)
        c = math.sqrt((x3-x1)**2 + (y3-y1)**2)
        
        if area == 0:
            curvature = 0
        else:
            curvature = (4 * area) / (a * b * c)
            
        # 2. Calculate Max Velocity allowed by Friction Circle
        # V_max = sqrt( F_lat / curvature )
        if curvature < 0.01:
            v_target = MAX_VELOCITY
        else:
            v_target = math.sqrt(MAX_LATERAL_ACCEL / curvature)
            
        v_target = max(MIN_VELOCITY, min(MAX_VELOCITY, v_target))
        
        new_path.append([x2, y2, v_target])
        
    return new_path

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 generate_raceline.py <map_name_no_extension>")
        print("Example: python3 generate_raceline.py my_track")
        sys.exit(1)
        
    map_name = sys.argv[1]
    print(f"Processing map: {map_name}...")
    
    # 1. Load Map
    thresh_img, res, origin = load_map(map_name)
    
    # 2. Extract Centerline
    print("Extracting centerline...")
    skeleton = extract_centerline(thresh_img)
    
    # 3. Convert to World Path
    path = sort_points_to_path(skeleton, res, origin)
    if len(path) < 10:
        print("Failed to generate valid path.")
        sys.exit(1)
        
    # 4. Optimize Velocity Profile
    print("Optimizing velocity profile...")
    final_trajectory = optimize_velocity(path)
    
    # 5. Save to CSV
    output_file = f"{map_name}_raceline.csv"
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(final_trajectory)
        
    print(f"Success! Saved {len(final_trajectory)} waypoints to {output_file}")
    print("Format: [x, y, velocity]")