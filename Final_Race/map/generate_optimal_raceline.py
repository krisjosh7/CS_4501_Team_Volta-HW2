import cv2
import numpy as np
import yaml
import csv
import math
import sys
from scipy.interpolate import splprep, splev

# --- CONFIGURATION ---
LOOKAHEAD_IDX = 5       
MAX_VELOCITY = 50.0      
MIN_VELOCITY = 10.0      
MAX_LATERAL_ACCEL = 20.0 
INTERPOLATION_POINTS = 500 # How many points in the final path

# Global variables for mouse callback
ref_points = []
img_display = None

def load_map(map_name):
    try:
        with open(f"{map_name}.yaml", 'r') as f:
            map_data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: Could not find {map_name}.yaml")
        sys.exit(1)
    
    image_path = map_data['image']
    resolution = map_data['resolution']
    origin = map_data['origin']
    
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        img = cv2.imread(f"{map_name}.pgm", cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not load image {image_path}")
        sys.exit(1)

    return img, resolution, origin

def click_event(event, x, y, flags, param):
    global ref_points, img_display
    
    if event == cv2.EVENT_LBUTTONDOWN:
        ref_points.append((x, y))
        # Draw a circle where user clicked
        cv2.circle(img_display, (x, y), 5, (0, 0, 0), -1) 
        cv2.imshow("Map - Click Points (Press 'd' when done)", img_display)

def get_user_waypoints(img):
    global img_display
    img_display = img.copy()
    
    # Make it color for better visibility of clicks
    img_display = cv2.cvtColor(img_display, cv2.COLOR_GRAY2BGR)
    
    cv2.namedWindow("Map - Click Points (Press 'd' when done)")
    cv2.setMouseCallback("Map - Click Points (Press 'd' when done)", click_event)
    
    print(">>> INSTRUCTIONS: ")
    print("1. Click points along your desired racing line (Start -> Corners -> End).")
    print("2. Click roughly 5-10 points to define the shape.")
    print("3. Press 'd' on your keyboard when finished.")
    
    while True:
        cv2.imshow("Map - Click Points (Press 'd' when done)", img_display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('d'):
            break
            
    cv2.destroyAllWindows()
    return ref_points

def generate_spline(points, resolution, origin, height):
    if len(points) < 3:
        print("Error: Need at least 3 points to generate a loop.")
        sys.exit(1)
        
    # Separate x and y
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    
    # Close the loop (connect last point to first)
    x.append(x[0])
    y.append(y[0])
    
    # Fit Spline (per_bc_type='periodic' ensures smooth closure)
    try:
        tck, u = splprep([x, y], s=0, per=True) 
    except TypeError:
        # Fallback for older scipy versions
        tck, u = splprep([x, y], s=0)

    # Generate smooth path
    u_new = np.linspace(u.min(), u.max(), INTERPOLATION_POINTS)
    x_new, y_new = splev(u_new, tck)
    
    # Convert pixels to World Coordinates
    world_path = []
    for i in range(len(x_new)):
        c, r = x_new[i], y_new[i] # x=col, y=row
        
        x_world = c * resolution + origin[0]
        y_world = (height - 1 - r) * resolution + origin[1]
        
        world_path.append([x_world, y_world])
        
    return np.array(world_path)

def optimize_velocity(path):
    # (Same velocity logic as before)
    new_path = []
    num_points = len(path)
    
    for i in range(num_points):
        p_prev = path[(i - LOOKAHEAD_IDX) % num_points]
        p_curr = path[i]
        p_next = path[(i + LOOKAHEAD_IDX) % num_points]
        
        x1, y1 = p_prev
        x2, y2 = p_curr
        x3, y3 = p_next
        area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
        a = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        b = math.sqrt((x2-x3)**2 + (y2-y3)**2)
        c = math.sqrt((x3-x1)**2 + (y3-y1)**2)
        
        curvature = 0 if (area == 0 or (a*b*c) == 0) else (4 * area) / (a * b * c)
        
        if curvature < 0.05:
            v_target = MAX_VELOCITY
        else:
            v_target = math.sqrt(MAX_LATERAL_ACCEL / curvature)
        
        v_target = max(MIN_VELOCITY, min(MAX_VELOCITY, v_target))
        new_path.append([p_curr[0], p_curr[1], v_target])
        
    return new_path

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 generate_raceline_interactive.py <map_name_no_extension>")
        sys.exit(1)
        
    map_name = sys.argv[1]
    
    print(f"Loading map: {map_name}...")
    img, res, origin = load_map(map_name)
    
    # 1. Get User Input
    points = get_user_waypoints(img)
    print(f"Captured {len(points)} points.")
    
    # 2. Generate Spline
    print("Interpolating smooth path...")
    path = generate_spline(points, res, origin, img.shape[0])
    
    # 3. Optimize Velocity
    print("Optimizing velocity...")
    final_path = optimize_velocity(path)
    
    # 4. Save
    output_file = f"{map_name}_raceline.csv"
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(final_path)
        
    print(f"Success! Saved to {output_file}")
    
    # Optional: Quick debug visualization of the result
    debug_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # Project world points back to pixels for display
    for p in final_path:
        wx, wy = p[0], p[1]
        px = int((wx - origin[0]) / res)
        py = int(img.shape[0] - 1 - (wy - origin[1]) / res)
        cv2.circle(debug_img, (px, py), 1, (0, 0, 255), -1)
        
    cv2.imwrite("debug_interactive_result.png", debug_img)
    print("Saved 'debug_interactive_result.png' for verification.")