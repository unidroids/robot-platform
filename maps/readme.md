# Mapová služba MAPS (Port 9040)

Mapová služba spravuje mapové podklady robota, topologický graf cest a poskytuje byznys logiku pro vyhledávání tras (`FIND_ROUTE`) s napojením na komunikaci a automatickým posunem do pravého jízdního pruhu.

## 🗺️ Mapové podklady
- Výchozí mapa je uložena offline u služby: `maps/defaut_map.json`
- Ve složce `vzory/` jsou uloženy originální mapové podklady různých oblastí (např. Kramolín, Stromovka).
- Při spuštění služby (příkaz `START`) se automaticky vytvoří instance (kopie) mapy do složky:  
  `/data/robot/maps/<yyyy-mm-dd>/<HH-MM-SS>/defaut_map.json`

---

## 🔌 Síťový protokol (TCP Port 9040)

Služba naslouchá na TCP portu `9040` (výchozí bind `0.0.0.0:9040`).

### Standardní řídicí příkazy:
| Příkaz | Odpověď | Popis |
| :--- | :--- | :--- |
| `PING` | `PONG MAPS` | Test dostupnosti služby |
| `START` | `OK` / `OK ALREADY_RUNNING` | Spuštění služby a vytvoření kopie mapy do `/data/robot/maps/...` |
| `STOP` | `OK` / `OK WAS NOT RUNNING` | Zastavení služby |
| `RESTART`| `OK` | Restartování služby a znovunačtení mapy |
| `STATUS` | `READY {"service": "MAPS", ...}` | Stav služby, počet uzlů, hran, čas spuštění a diagnostika |
| `EXIT` | `OK-MAPS-BYE` | Uzavření klientského TCP spojení |
| `SHUTDOWN` | `OK SHUTDOWN` | Ukončení celého procesu služby |

---

### Byznys příkaz: `FIND_ROUTE`
Syntaxe:
```text
FIND_ROUTE <start_lat>, <start_lon>, <cil_lat>, <cil_lon>
```
*(Oddělovačem parametrů mohou být čárky i mezery).*

#### 1. Kontrola napojení (5m pravidlo):
- Algoritmus spočte nejbližší bod na celé mapě (prohledá všechny uzly i úsečky hran).
- Pokud je start nebo cíl dále než **5 metrů** od mapy, trasa se nevypočte a služba vrátí:
  - `metadata.search_result`: `"cesta nenalezena"`
  - `metadata.reason`: detailní popis vzdálenosti a azimutu k nejbližšímu místu (např. `"Start je dále než 5m od nejbližšího místa na mapě. Nejbližší místo je 8.32 metrů s azimutem 142.5° daleko."`)
  - `nodes`: `[]`, `edges`: `[]`

#### 2. Úspěšné nalezení trasy:
Pokud jsou oba body $\le 5\,\text{m}$ od komunikace:
- Vytvoří se spojovací úseky k nejbližším bodům mapy.
- Dijkstra nalezne optimální posloupnost uzlů a hran.
- **Pravostranný offset:** Úseky a body se přepočítají pro jízdu vpravo od středu cesty podle její šířky $W$:
  - $W < 2.0\,\text{m} \implies \text{offset} = 0.0\,\text{m}$ (střed cesty)
  - $2.0 \le W < 3.0\,\text{m} \implies \text{offset} = 0.5\,\text{m}$
  - $3.0 \le W < 4.0\,\text{m} \implies \text{offset} = 0.75\,\text{m}$
  - $W \ge 4.0\,\text{m} \implies \text{offset} = 1.0\,\text{m}$
- Spojovací úsek od startu a do cíle plynule navazuje na tento posunutý pravý pruh.

Odpověď je jednořádkový JSON zakončený `\n`:
```json
{
  "metadata": {
    "area_name": "Kramolín 2026-07-29",
    "type": "route_result",
    "service_start_time": "2026-09-17T07:20:00.000000",
    "search_result": "found",
    "route_length_m": 210.78,
    "start_query": {"lat": 49.554131, "lon": 12.741158},
    "goal_query": {"lat": 49.555201, "lon": 12.743162},
    "start_distance_to_map_m": 0.23,
    "goal_distance_to_map_m": 0.45,
    "reason": ""
  },
  "nodes": [
    {"id": "route_node_1_route_start", "lat": 49.554131, "lon": 12.741158, "label": "Start trasy"},
    {"id": "route_node_2_osm_node_1", "lat": 49.554139, "lon": 12.741165, "label": "OSM Bod 1"},
    ...
  ],
  "edges": [
    {
      "id": "route_edge_1",
      "from": "route_node_1_route_start",
      "to": "route_node_2_osm_node_1",
      "name": "Přístup na trasu",
      "width_m": 4.5,
      "offset_m": 1.0,
      "length_m": 1.25,
      "controlPoint": null,
      "coords": []
    },
    ...
  ]
}
```

---

## 🚀 Spuštění a testy

### Spuštění služby:
```bash
python3 maps/main.py [--port 9040] [--map maps/defaut_map.json]
```

### Spuštění testů:
```bash
python3 -m unittest maps.test_maps
```
*(Zahrnuje 16 jednotkových a integračních testů geometrie, grafu, 5m limitu, offsetu i TCP soketové komunikace).*
