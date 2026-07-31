import json
import math
import sys

def latlon_to_xy(lat, lon, lat0):
    # Approximation for small distances
    x = lon * 111320 * math.cos(math.radians(lat0))
    y = lat * 111320
    return x, y

def point_to_segment_dist(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    
    qx = x1 + t * dx
    qy = y1 + t * dy
    return math.hypot(px - qx, py - qy)

def analyze():
    route_file = '/opt/projects/robotour/pilot_waypoints/waypoints/_route.json'
    log_file = '/opt/projects/robotour/pilot_waypoints/testing/logger-13-14-29.dat'
    
    with open(route_file, 'r') as f:
        route = json.load(f)
        
    waypoints = route['waypoints']
    if not waypoints:
        print("No waypoints found in route.")
        return
        
    lat0 = waypoints[0]['lat']
    pts = [latlon_to_xy(wp['lat'], wp['lon'], lat0) for wp in waypoints]
    
    max_dev = 0
    sum_dev = 0
    count = 0
    
    trajectory = []
    
    with open(log_file, 'r') as f:
        for line in f:
            if 'ipc:///tmp/robot-fusion SOLUTION/' in line:
                try:
                    # Line format: timestamp topic JSON
                    json_str = line.split('ipc:///tmp/robot-fusion SOLUTION/')[1].strip()
                    data = json.loads(json_str)
                    if data.get('speed', 0) > 10: # Only count when moving
                        px, py = latlon_to_xy(data['lat'], data['lon'], lat0)
                        
                        # Find closest segment
                        min_d = float('inf')
                        for i in range(len(pts)-1):
                            d = point_to_segment_dist(px, py, pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
                            if d < min_d:
                                min_d = d
                                
                        max_dev = max(max_dev, min_d)
                        sum_dev += min_d
                        count += 1
                except Exception as e:
                    pass
                    
    if count == 0:
        print("No movement found in the log.")
    else:
        avg_dev = sum_dev / count
        print(f"Analysis complete.")
        print(f"Evaluated {count} moving points.")
        print(f"Maximum deviation from path: {max_dev:.3f} meters")
        print(f"Average deviation from path: {avg_dev:.3f} meters")

if __name__ == '__main__':
    analyze()
