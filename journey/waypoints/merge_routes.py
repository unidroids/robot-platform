import json
import math

def distance(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2)

with open('/opt/projects/robotour/journey/waypoints/_route-03-zacatek-long.json', 'r') as f:
    r03 = json.load(f)
with open('/opt/projects/robotour/journey/waypoints/_route-04-zadni-rovinka.json', 'r') as f:
    r04 = json.load(f)

w03 = r03['waypoints']
w04 = r04['waypoints']

# Find turnaround of 03 (point furthest from start)
dists_from_start = [distance(w03[0]['lat'], w03[0]['lon'], p['lat'], p['lon']) for p in w03]
turnaround_03 = dists_from_start.index(max(dists_from_start))
print(f"Turnaround of 03 is at index {turnaround_03}")

start_04 = w04[0]
end_04 = w04[-1]

# Cesta tam (before turnaround)
w03_tam = w03[:turnaround_03]
dists_tam = [distance(start_04['lat'], start_04['lon'], p['lat'], p['lon']) for p in w03_tam]
idx_tam = dists_tam.index(min(dists_tam))

# Cesta zpet (after turnaround)
w03_zpet = w03[turnaround_03:]
dists_zpet = [distance(end_04['lat'], end_04['lon'], p['lat'], p['lon']) for p in w03_zpet]
idx_zpet = dists_zpet.index(min(dists_zpet)) + turnaround_03

print(f"Closest to start of 04 is 03 at index {idx_tam} (cesta tam)")
print(f"Closest to end of 04 is 03 at index {idx_zpet} (cesta zpet)")

new_waypoints = w03[:idx_tam] + w04 + w03[idx_zpet:]
r05 = {"version": r03.get("version", 1), "waypoints": new_waypoints}

with open('/opt/projects/robotour/journey/waypoints/_route-05-okruh.json', 'w') as f:
    json.dump(r05, f, indent=2)

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
