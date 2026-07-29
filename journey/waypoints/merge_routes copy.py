import json
import math

def distance(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2)

def find_closest(p, pts):
    min_dist = float('inf')
    min_idx = -1
    for i, pt in enumerate(pts):
        d = distance(p['lat'], p['lon'], pt['lat'], pt['lon'])
        if d < min_dist:
            min_dist = d
            min_idx = i
    return min_idx

with open('/opt/projects/robotour/journey/waypoints/_route-03-zacatek-long.json', 'r') as f:
    r03 = json.load(f)
with open('/opt/projects/robotour/journey/waypoints/_route-04-zadni-rovinka.json', 'r') as f:
    r04 = json.load(f)

w03 = r03['waypoints']
w04 = r04['waypoints']

# Start of 04
start_04 = w04[0]
# End of 04
end_04 = w04[-1]

# To avoid mismatching sides of the road, search first half for connection out, and second half for connection back
half = len(w03) // 2
idx_tam = find_closest(start_04, w03[:half])
idx_zpet = find_closest(end_04, w03[half:]) + half

print(f"Closest to start of 04 is 03 at index {idx_tam}")
print(f"Closest to end of 04 is 03 at index {idx_zpet}")

# The new route:
new_waypoints = w03[:idx_tam] + w04 + w03[idx_zpet:]
r05 = {"version": r03.get("version", 1), "waypoints": new_waypoints}

with open('/opt/projects/robotour/journey/waypoints/_route-05-okruh.json', 'w') as f:
    json.dump(r05, f, indent=2)

# Generate GPX
gpx_content = []
gpx_content.append('<?xml version="1.0" encoding="UTF-8"?>')
gpx_content.append('<gpx version="1.1" creator="MergeScript">')
gpx_content.append('  <trk>')
gpx_content.append('    <name>05-Okruh</name>')
gpx_content.append('    <trkseg>')
for w in new_waypoints:
    gpx_content.append(f'      <trkpt lat="{w["lat"]}" lon="{w["lon"]}"></trkpt>')
gpx_content.append('    </trkseg>')
gpx_content.append('  </trk>')
gpx_content.append('</gpx>')

with open('/opt/projects/robotour/journey/waypoints/_route-05-okruh.gpx', 'w') as f:
    f.write('\n'.join(gpx_content))

print("Created _route-05-okruh.json and .gpx")
