# KrakenSDR Triangulator Web App

A professional **laptop-side** web application designed for verifying KrakenSDR bearing estimates and performing advanced signal-source triangulation. This tooling is built specifically for Commander laptops during NGCP UAV signal-hunting operations.

> **Note:** This runs on the ground station laptop — not the Raspberry Pi 5. It is self-contained and operates independently of the Pi-side GCS telemetry pipeline.

---

## What This Does

1. **Ingests Bearing Estimates:** Receives live data from KrakenSDR receiver networks or replays localized historical `.json` files.
2. **Advanced Triangulation Math:** Computes the estimated target origin using two distinct mathematical modes:
   - **Bayesian Spatial Heatmap:** A probability-density field generated from accumulated intersection uncertainties.
   - **Least Squares Angle-of-Arrival (LS-AoA):** A rigid matrix-solver designed for clean, intersecting geometries.
3. **Interactive Spatial Filtering:** Features UI-driven spatial masking tools ("Draw Area" and "Draw Polygon") to dynamically filter out rogue bearings, reflections, or corrupted data physically tracking outside a designated zone.
4. **Professional GUI:** Displays real-time results on an interactive map (Leaflet.js) sporting a dark, premium, utilitarian aesthetic with Playback console controls, a bearing log, and customizable mapping layers.

---

## Quick Start Guide

Follow these step-by-step commands to get the application running on your local machine using the built-in playback data environment.

### 1. Clone the Repository
Open your terminal or command prompt and clone the main project repository:
```bash
git clone https://github.com/JanPastor/NGCP-Kraken-Triangulator-App.git
cd NGCP-Kraken-Triangulator-App/kraken-triangulator
```

### 2. Install Backend Dependencies
The application relies on a lightweight Python server (Flask) to handle spatial mathematics and serve the frontend interface. You need Python 3.8+ installed.
```bash
pip install -r requirements.txt
```

### 3. Start the Application Server
Run the Python backend server. It starts in live UDP mode and listens for fused observations on UDP port 5051.
```bash
python server/kraken_server.py
```
*The server will boot up and bind to **http://localhost:5050**.*

### 4. Open the Web App
Open any modern web browser (Chrome, Firefox, Edge) and navigate to:
```url
http://localhost:5050
```
You are now in the app! Use the **Data Playback** console at the top to hit **Play (▶)** and watch the historical triangulation happen in real-time.

---

## Directory Structure
```text
kraken-triangulator/
├── README.md                  # This file
├── requirements.txt           # Python backend dependencies
├── server/
│   └── kraken_server.py       # Flask backend & math/playback engine
├── data/
│   └── bearings_*.json        # Pre-recorded JSON telemetry for playback testing
└── app/
    ├── index.html             # The main Single Page Application GUI
    ├── style.css              # Dark mode styling and CSS definitions
    └── js/
        ├── main.js            # App initialization, settings, and playback state
        ├── map.js             # Leaflet GPS map, heatmap, and custom overlays
        ├── triangulation.js   # Core LS-AoA / Bayesian solvers & Spatial Filtering
        └── data_feed.js       # Asynchronous data polling from the Python backend
```

---

## Supported Input Formats

The app accepts native waypoint JSON and fused JSONL records. New fused records use `doa_deg` as an already-converted compass bearing and `doa_papr_db` as a raw peak-sharpness value. PAPR is measured in dB and is not a 0–1 probability.

The replay adapter also accepts older fusion logs that use `confidence_0_1`. Those values are treated as a legacy raw quality metric, not blindly displayed as a percentage. `usable_for_triangulation` is honored when present, and spectrum metrics are preserved for display and weighting.

Raw Kraken logger JSONL does not contain the UAV telemetry position needed for triangulation. Fuse it with telemetry first, or load the resulting fusion JSONL instead.

---

## Live Hardware Integration

When you are ready to connect a live KrakenSDR unit instead of using Playback data, invoke the server by passing the Kraken's API URL as an environment variable:

**Linux / Mac:**
```bash
KRAKEN_API_URL=http://<kraken-ip-address>:8080 python server/kraken_server.py
```

**Windows (PowerShell):**
```powershell
$env:KRAKEN_API_URL="http://<kraken-ip-address>:8080"
python server/kraken_server.py
```


---

## License

This is Free and Open Source Software (FOSS) developed for the **Northrop Grumman Collaboration Project** (NGCP). 
It is distributed under the [MIT License](LICENSE).
