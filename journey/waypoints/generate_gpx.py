import json
import os

files = [
    "_route_2026-05-23-08-00-32.json",
    "_route_2026-05-23-18-04-08.json",
    "_route_2026-05-23-20-36-26.json",
    "_route_2026-06-13-14-36-27.json",
    "_route_2026-06-13-14-36-28.json"
]

base_dir = "/opt/projects/robotour/journey/waypoints/"

for filename in files:
    filepath = os.path.join(base_dir, filename)
    if not os.path.exists(filepath):
        print(f"Nenalezen: {filepath}")
        continue
        
    with open(filepath, 'r') as f:
        data = json.load(f)
        
    waypoints = data.get('waypoints', [])
    if not waypoints:
        print(f"Zadne waypointy v: {filename}")
        continue
        
    gpx_filename = filename.replace('.json', '.gpx')
    gpx_filepath = os.path.join(base_dir, gpx_filename)
    
    gpx_content = []
    gpx_content.append('<?xml version="1.0" encoding="UTF-8"?>')
    gpx_content.append('<gpx version="1.1" creator="Robotour Script">')
    gpx_content.append('  <trk>')
    gpx_content.append(f'    <name>{gpx_filename}</name>')
    gpx_content.append('    <trkseg>')
    for w in waypoints:
        gpx_content.append(f'      <trkpt lat="{w["lat"]}" lon="{w["lon"]}"></trkpt>')
    gpx_content.append('    </trkseg>')
    gpx_content.append('  </trk>')
    gpx_content.append('</gpx>')

    with open(gpx_filepath, 'w') as f:
        f.write('\n'.join(gpx_content))
        
    print(f"Created {gpx_filename}")
