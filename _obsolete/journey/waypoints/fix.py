import json
import math

def dist(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2)

r03=json.load(open('_route-03-zacatek-long.json'))['waypoints']
r04=json.load(open('_route-04-zadni-rovinka.json'))['waypoints']

s4 = r04[0]
e4 = r04[-1]

dists_s4 = [dist(s4['lat'], s4['lon'], p['lat'], p['lon']) for p in r03]
idx_tam = dists_s4.index(min(dists_s4))

dists_e4 = [dist(e4['lat'], e4['lon'], p['lat'], p['lon']) for p in r03]
idx_zpet = dists_e4.index(min(dists_e4))

print("Tam:", idx_tam)
print("Zpet:", idx_zpet)
