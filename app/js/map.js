/**
 * map.js — Leaflet GPS Map Module (Single Mobile KrakenSDR)
 * ===========================================================
 * Renders the KrakenSDR route and bearing observations:
 *  - Polyline path of all observation positions visited
 *  - Small dot markers at each historical observation + bearing line
 *  - Pulsing marker at the CURRENT (latest) position
 *  - Bearing line from current position (brighter/thicker)
 *  - Triangulation result marker + uncertainty circle
 */

const MapView = (() => {
    let _map            = null;
    let _pathLayer      = null;   // The driven route polyline
    let _obsLayer       = null;   // Historical observation dots + bearing lines
    let _currentLayer   = null;   // Current position marker + bearing line
    let _resultLayer    = null;   // Triangulated result + uncertainty circle
    let _customLayer    = null;   // User-placed custom markers
    let _heatLayer      = null;   // Triangulated result heatmap
    let _tileLayer      = null;
    let _lineLengthKm   = 2;
    let _showUncertainty = true;
    let _firstLoad       = true;  // Prevent permanent zoom-lock

    const OBS_COLOR     = '#4f8ef7';  // Blue-ish — historical observations
    const PATH_COLOR   = '#3498db';
    const RESULT_COLOR = '#00ff00';
    const BEARING_COLOR = '#3498db'; // Kraken uses blue for bearings
    const HEADING_COLOR = '#e74c3c'; // Kraken uses red for vehicle heading

    let _heatPoints = [];
    let _heatRadius = 30;
    let _heatBlur = 20;
    let _heatOpacity = 0.7;

    let _drawnObsIds = new Set();
    let _resultCrosshair = null;
    let _resultCenterDot = null;
    let _customMarkers = [];
    let _customPolyline = null;
    
    // Masking variables
    let _maskLayer = null;
    let _maskPoints = [];
    let _maskPolygon = null;
    let _maskCenterMarker = null;
    let _maskColor = '#9b59b6';

    // Polygon variables
    let _polyLayer = null;
    let _polyPoints = [];
    let _polyPolygon = null;
    let _polyCenterMarker = null;
    let _polyColor = '#fa8231';
    let _polyIsLocked = false;
    window.saveCustomMarkerName = function(id, newName) {
        const obj = _customMarkers.find(o => o.id === id);
        if (obj && newName.trim() !== '') {
            obj.name = newName.trim();
        }
    };

    window.toggleCustomMarkerLock = function(id) {
        const obj = _customMarkers.find(o => o.id === id);
        if (!obj) return;
        obj.locked = !obj.locked;
        if (obj.locked) {
            obj.marker.dragging.disable();
        } else {
            obj.marker.dragging.enable();
        }
        _refreshCustomMarkers();
    };

    window.setCustomMarkerColor = function(id, color) {
        const obj = _customMarkers.find(o => o.id === id);
        if (!obj) return;
        obj.color = color;
        // Update marker icon color
        const icon = _customIcon(color);
        obj.marker.setIcon(icon);
        _refreshCustomMarkers();
    };

    window.removeCustomMarker = function(id) {
        const obj = _customMarkers.find(o => o.id === id);
        if (!obj) return;
        obj.marker.remove();
        _customMarkers = _customMarkers.filter(o => o.id !== id);
        MapView.refreshCustomMarkers();
    };

    function _constructMaskMarkers(pts) {
        // Centroid draggable marker
        const centerIcon = L.divIcon({
            className: 'mask-centroid-icon',
            html: ``,
            iconSize: [12, 12],
            iconAnchor: [6, 6]
        });
        
        const bounds = L.latLngBounds(pts.map(p => [p.lat, p.lon]));
        const c = bounds.getCenter();
        
        _maskCenterMarker = L.marker(c, { icon: centerIcon, draggable: true, zIndexOffset: 1000 }).addTo(_maskLayer);
        let lastCenterLat = c.lat;
        let lastCenterLng = c.lng;
        
        _maskCenterMarker.on('drag', (e) => {
            const ll = e.target.getLatLng();
            const dLat = ll.lat - lastCenterLat;
            const dLng = ll.lng - lastCenterLng;
            
            _maskPoints.forEach(m => {
                const ml = m.getLatLng();
                m.setLatLng([ml.lat + dLat, ml.lng + dLng]);
            });
            lastCenterLat = ll.lat;
            lastCenterLng = ll.lng;
            _redrawMaskPolygon();
            if (window._syncMaskUI) window._syncMaskUI();
            if (window._triggerAppUpdate) window._triggerAppUpdate();
        });
        
        _maskCenterMarker.on('dragend', () => { 
            if (window._syncMaskUI) window._syncMaskUI(); 
            if (window._triggerAppUpdate) window._triggerAppUpdate();
        });

        const handleIcon = L.divIcon({
            className: 'mask-handle-icon',
            html: ``,
            iconSize: [12, 12],
            iconAnchor: [6, 6]
        });

        // Use only TL (0) and BR (2) as handles
        const handlePts = [
            { lat: pts[0].lat, lon: pts[0].lon, role: 'TL' },
            { lat: pts[2].lat, lon: pts[2].lon, role: 'BR' }
        ];

        handlePts.forEach((p, idx) => {
            const marker = L.marker([p.lat, p.lon], { icon: handleIcon, draggable: true }).addTo(_maskLayer);
            marker.maskRole = p.role;
            
            marker.on('drag', (e) => {
                const ll = e.target.getLatLng();
                const otherIdx = idx === 0 ? 1 : 0;
                const otherLL = _maskPoints[otherIdx].getLatLng();
                
                const newBounds = L.latLngBounds(ll, otherLL);
                const c = newBounds.getCenter();
                _maskCenterMarker.setLatLng(c);
                lastCenterLat = c.lat;
                lastCenterLng = c.lng;
                
                _redrawMaskPolygon();
                if (window._syncMaskUI) window._syncMaskUI();
                if (window._triggerAppUpdate) window._triggerAppUpdate();
            });
            
            marker.on('dragend', () => { 
                if (window._syncMaskUI) window._syncMaskUI(); 
                if (window._triggerAppUpdate) window._triggerAppUpdate();
            });
            _maskPoints.push(marker);
        });
        
        _redrawMaskPolygon();
        if (window._syncMaskUI) window._syncMaskUI();
        if (window._triggerAppUpdate) window._triggerAppUpdate();
    }

    // Mask API
    let _cancelDrawingFn = null;

    window.drawDefaultMask = function(widthMeters = null, heightMeters = null, colorHex = '#00e87a', customLat = null, customLng = null) {
        if (_maskPolygon) return;
        
        _maskColor = colorHex;

        if (!widthMeters || !heightMeters) {
            // Draw mode
            _map.getContainer().style.cursor = 'crosshair';
            let drawStart = null;
            let drawingRect = null;
            
            const onMouseDown = (e) => {
                drawStart = e.latlng;
                drawingRect = L.rectangle([drawStart, drawStart], { color: _maskColor, weight: 2, fillColor: _maskColor, fillOpacity: 0.2, dashArray: '5, 5' }).addTo(_map);
                _map.dragging.disable();
            };
            
            const onMouseMove = (e) => {
                if (!drawStart) return;
                drawingRect.setBounds([drawStart, e.latlng]);
                if (window._syncMaskUI) {
                    const b = drawingRect.getBounds();
                    window._syncMaskUI({
                        minLat: b.getSouth(),
                        maxLat: b.getNorth(),
                        minLon: b.getWest(),
                        maxLon: b.getEast()
                    });
                }
                if (window._triggerAppUpdate) window._triggerAppUpdate();
            };
            
            const onMouseUp = (e) => {
                if (!drawStart) return;
                const bounds = drawingRect.getBounds();
                _map.removeLayer(drawingRect);
                _map.getContainer().style.cursor = '';
                _map.dragging.enable();
                
                _map.off('mousedown', onMouseDown);
                _map.off('mousemove', onMouseMove);
                _map.off('mouseup', onMouseUp);
                
                if (bounds.getNorth() === bounds.getSouth() || bounds.getEast() === bounds.getWest()) {
                    window.drawDefaultMask(100, 100, colorHex); // fallback
                    return;
                }
                
                const pts = [
                    { lat: bounds.getNorth(), lon: bounds.getWest(), role: 'NW' },
                    { lat: bounds.getNorth(), lon: bounds.getEast(), role: 'NE' },
                    { lat: bounds.getSouth(), lon: bounds.getEast(), role: 'SE' },
                    { lat: bounds.getSouth(), lon: bounds.getWest(), role: 'SW' }
                ];
                _constructMaskMarkers(pts);
                
                _cancelDrawingFn = null;
            };
            
            _cancelDrawingFn = () => {
                if (drawingRect) _map.removeLayer(drawingRect);
                _map.getContainer().style.cursor = '';
                _map.dragging.enable();
                _map.off('mousedown', onMouseDown);
                _map.off('mousemove', onMouseMove);
                _map.off('mouseup', onMouseUp);
                _cancelDrawingFn = null;
            }
            
            _map.on('mousedown', onMouseDown);
            _map.on('mousemove', onMouseMove);
            _map.on('mouseup', onMouseUp);
            return;
        }
        
        const centerParams = (customLat !== null && customLng !== null) ? { lat: customLat, lng: customLng } : _map.getCenter();
        const center = centerParams;
        const w2 = widthMeters / 2;
        const h2 = heightMeters / 2;
        
        const northLat = TriangulationHelpers.projectPoint(center.lat, center.lng, 0, h2).lat;
        const southLat = TriangulationHelpers.projectPoint(center.lat, center.lng, 180, h2).lat;
        const eastLon = TriangulationHelpers.projectPoint(center.lat, center.lng, 90, w2).lon;
        const westLon = TriangulationHelpers.projectPoint(center.lat, center.lng, 270, w2).lon;
        
        const pts = [
            { lat: northLat, lon: westLon, role: 'NW' },
            { lat: northLat, lon: eastLon, role: 'NE' },
            { lat: southLat, lon: eastLon, role: 'SE' },
            { lat: southLat, lon: westLon, role: 'SW' }
        ];
        
        _constructMaskMarkers(pts);
    };

    window.clearMask = function() {
        if (_cancelDrawingFn) _cancelDrawingFn();
        _maskPoints = [];
        _maskCenterMarker = null;
        _maskLayer.clearLayers();
        _maskPolygon = null;
        if (window._syncMaskUI) window._syncMaskUI();
        if (window._triggerAppUpdate) window._triggerAppUpdate();
    };

    window.getMaskBounds = function() {
        if (!_maskPolygon) return null;
        const bounds = _maskPolygon.getBounds();
        return {
            minLat: bounds.getSouth(),
            maxLat: bounds.getNorth(),
            minLon: bounds.getWest(),
            maxLon: bounds.getEast()
        };
    };

    function _redrawMaskPolygon() {
        if (_maskPoints.length < 2) return;
        const b = L.latLngBounds(_maskPoints[0].getLatLng(), _maskPoints[1].getLatLng());
        const latlngs = [
            [b.getNorth(), b.getWest()],
            [b.getNorth(), b.getEast()],
            [b.getSouth(), b.getEast()],
            [b.getSouth(), b.getWest()]
        ];
        if (_maskPolygon) {
            _maskPolygon.setLatLngs(latlngs);
        } else {
            _maskPolygon = L.polygon(latlngs, {
                color: _maskColor,
                weight: 2,
                fillColor: _maskColor,
                fillOpacity: 0.15,
                dashArray: '5, 5'
            }).addTo(_maskLayer);
        }
        if (window._triggerAppUpdate) window._triggerAppUpdate();
    }

    // ── Polygon API ──
    window.drawPolygonMask = function(numVertices = 6, colorHex = '#fa8231', forceCenter = null) {
        let center = forceCenter;
        if (!center) {
            if (_polyCenterMarker) center = _polyCenterMarker.getLatLng();
            else center = _map.getCenter();
        }
        
        if (_polyPolygon) window.clearPoly();
        _polyColor = colorHex;
        
        // Initial spawn logic (equally spaced vertices in a circle around center)
        const pts = [];
        const radiusM = 50; // default size
        
        for (let i = 0; i < numVertices; i++) {
            const angleDeg = (i * 360) / numVertices;
            const pt = TriangulationHelpers.projectPoint(center.lat, center.lng, angleDeg, radiusM);
            pts.push({ lat: pt.lat, lon: pt.lon });
        }
        
        _constructPolyMarkers(pts);
    };

    function _constructPolyMarkers(pts) {
        if (!_polyLayer) _polyLayer = L.layerGroup().addTo(_map);
        window.clearPoly();
        
        const centerHtml = `<div style="width:100%; height:100%; background:${_polyColor}; border:2px solid #fff; box-sizing:border-box; box-shadow:0 0 6px rgba(0,0,0,0.5); border-radius:2px; cursor:move;"></div>`;
        const centerIcon = L.divIcon({ className: '', html: centerHtml, iconSize: [14, 14], iconAnchor: [7, 7] });
        
        const vertexHtml = `<div style="width:100%; height:100%; background:#fff; border:2px solid ${_polyColor}; box-sizing:border-box; border-radius:2px; box-shadow:0 0 4px rgba(0,0,0,0.5); cursor:move;"></div>`;
        const vertexIcon = L.divIcon({ className: '', html: vertexHtml, iconSize: [12, 12], iconAnchor: [6, 6] });

        
        const bounds = L.latLngBounds(pts.map(p => [p.lat, p.lon]));
        const c = bounds.getCenter();
        
        _polyCenterMarker = L.marker(c, { icon: centerIcon, draggable: true, zIndexOffset: 1000 }).addTo(_polyLayer);
        // Hide centroid initially unless locked
        if (!_polyIsLocked) _polyCenterMarker.getElement()?.style.setProperty('display', 'none', 'important');
        
        let lastCenterLat = c.lat;
        let lastCenterLng = c.lng;
        
        _polyCenterMarker.on('drag', (e) => {
            const ll = e.target.getLatLng();
            const dLat = ll.lat - lastCenterLat;
            const dLng = ll.lng - lastCenterLng;
            
            _polyPoints.forEach(m => {
                const ml = m.getLatLng();
                m.setLatLng([ml.lat + dLat, ml.lng + dLng]);
            });
            lastCenterLat = ll.lat;
            lastCenterLng = ll.lng;
            
            _redrawPolygonPath();
            if (window._triggerAppUpdate) window._triggerAppUpdate();
        });
        
        _polyCenterMarker.on('dragend', () => {
            if (window._triggerAppUpdate) window._triggerAppUpdate();
        });

        pts.forEach(p => {
            const marker = L.marker([p.lat, p.lon], { icon: vertexIcon, draggable: !_polyIsLocked }).addTo(_polyLayer);
            if (_polyIsLocked) marker.getElement()?.style.setProperty('display', 'none', 'important');
            
            marker.on('drag', () => {
                // Update centroid mathematically on vertex drag
                const latlngs = _polyPoints.map(m => Object.values(m.getLatLng()));
                const b = L.latLngBounds(latlngs);
                const nc = b.getCenter();
                _polyCenterMarker.setLatLng(nc);
                lastCenterLat = nc.lat;
                lastCenterLng = nc.lng;
                
                _redrawPolygonPath();
                if (window._triggerAppUpdate) window._triggerAppUpdate();
            });
            
            marker.on('dragend', () => { 
                if (window._triggerAppUpdate) window._triggerAppUpdate();
            });
            _polyPoints.push(marker);
        });
        
        _redrawPolygonPath();
        if (window._triggerAppUpdate) window._triggerAppUpdate();
    }

    function _redrawPolygonPath() {
        if (_polyPoints.length < 3) return;
        const latlngs = _polyPoints.map(m => m.getLatLng());
        
        if (_polyPolygon) {
            _polyPolygon.setLatLngs(latlngs);
            _polyPolygon.setStyle({ color: _polyColor, fillColor: _polyColor });
        } else {
            _polyPolygon = L.polygon(latlngs, {
                color: _polyColor,
                weight: 2,
                fillColor: _polyColor,
                fillOpacity: 0.15,
                dashArray: '5, 5'
            }).addTo(_polyLayer);
        }
    }

    window.clearPoly = function() {
        _polyPoints = [];
        _polyCenterMarker = null;
        if (_polyLayer) _polyLayer.clearLayers();
        _polyPolygon = null;
        if (window._triggerAppUpdate) window._triggerAppUpdate();
    };

    window.getPolyBounds = function() {
        if (!_polyPolygon || _polyPoints.length < 3) return null;
        return _polyPoints.map(m => {
            const ll = m.getLatLng();
            return { lat: ll.lat, lon: ll.lng };
        });
    };

    window.updatePolyStyle = function(numVerts, colorHex) {
        _polyColor = colorHex;
        if (_polyPoints.length > 0 && _polyPoints.length !== numVerts) {
            window.drawPolygonMask(numVerts, colorHex); // redraws entirely
        } else {
            _redrawPolygonPath(); // just updates color if active
        }
    };

    window.togglePolyLock = function(isLocked) {
        _polyIsLocked = isLocked;
        if (_polyCenterMarker) {
            const cel = _polyCenterMarker.getElement();
            if (cel) cel.style.setProperty('display', isLocked ? 'block' : 'none', 'important');
        }
        _polyPoints.forEach(m => {
            const el = m.getElement();
            if (el) el.style.setProperty('display', isLocked ? 'none' : 'block', 'important');
            if (m.dragging) {
                if (isLocked) m.dragging.disable();
                else m.dragging.enable();
            }
        });
    };

    window.getSearchAreaSummary = function() {
        if (_maskPolygon && _maskPoints.length > 0) {
            const center = _maskCenterMarker ? _maskCenterMarker.getLatLng() : _maskPolygon.getBounds().getCenter();
            return {
                isActive: true,
                center: { lat: center.lat, lon: center.lng },
                isInside: (lat, lon) => {
                    const pt = turf.point([lon, lat]);
                    const poly = getMaskGeoJSON(); // uses internal helper
                    if (!poly) return false;
                    return turf.booleanPointInPolygon(pt, poly);
                }
            };
        }
        if (_polyPolygon && _polyPoints.length > 0) {
            const center = _polyCenterMarker ? _polyCenterMarker.getLatLng() : _polyPolygon.getBounds().getCenter();
            return {
                isActive: true,
                center: { lat: center.lat, lon: center.lng },
                isInside: (lat, lon) => {
                    const pt = turf.point([lon, lat]);
                    const pts = _polyPoints.map(m => {
                        const ll = m.getLatLng();
                        return [ll.lng, ll.lat];
                    });
                    if (pts.length < 3) return false;
                    pts.push(pts[0]);
                    return turf.booleanPointInPolygon(pt, turf.polygon([pts]));
                }
            };
        }
        return { isActive: false };
    };

    const TILE_CONFIGS = {
        osm: {
            url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
            attribution: '© <a href="https://openstreetmap.org">OpenStreetMap</a>',
        },
        topo: {
            url: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
            attribution: '© OpenTopoMap',
        },
        satellite: {
            url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attribution: '© Esri',
        },
    };

    // Helpers for geographic math
    const TriangulationHelpers = {
        projectPoint(lat, lon, bearing, distance) {
            const rad = Math.PI / 180;
            const R = 6371000;
            const d = distance / R;
            const brng = bearing * rad;
            const lat1 = lat * rad;
            const lon1 = lon * rad;

            let lat2 = Math.asin(Math.sin(lat1) * Math.cos(d) + Math.cos(lat1) * Math.sin(d) * Math.cos(brng));
            let lon2 = lon1 + Math.atan2(Math.sin(brng) * Math.sin(d) * Math.cos(lat1), Math.cos(d) - Math.sin(lat1) * Math.sin(lat2));

            return { lat: lat2 / rad, lon: lon2 / rad };
        },

        // Creates points for a regular polygon (like an octagon)
        createPolygon(lat, lon, radiusMeters, sides) {
            const points = [];
            for (let i = 0; i <= sides; i++) {
                const angle = (360 / sides) * i;
                const pt = this.projectPoint(lat, lon, angle, radiusMeters);
                points.push([pt.lat, pt.lon]);
            }
            return points;
        }
    };

    function _obsIcon(color, size = 10) {
        return L.divIcon({
            className: '',
            html: `<div style="
                width:${size}px;height:${size}px;border-radius:50%;
                background:${color};
                border:2px solid rgba(255,255,255,0.7);
                box-shadow:0 0 6px ${color};
            "></div>`,
            iconSize: [size, size],
            iconAnchor: [size/2, size/2],
        });
    }

    function _currentIcon() {
        return L.divIcon({
            className: '',
            html: `<div style="
                width:16px;height:16px;border-radius:50%;
                background:${OBS_COLOR};
                border:3px solid rgba(255,255,255,0.9);
                box-shadow:0 0 14px ${OBS_COLOR}, 0 0 4px rgba(0,0,0,0.5);
                animation: none;
            "></div>`,
            iconSize: [16, 16],
            iconAnchor: [8, 8],
        });
    }

    function _resultIcon() {
        return L.divIcon({
            className: '',
            html: `<div style="
                width:18px;height:18px;border-radius:50%;
                background:${RESULT_COLOR};
                border:3px solid rgba(255,255,255,0.9);
                box-shadow:0 0 14px ${RESULT_COLOR}, 0 0 4px rgba(0,0,0,0.6);
            "></div>`,
            iconSize: [18, 18],
            iconAnchor: [9, 9],
        });
    }

    function setTile(key) {
        const cfg = TILE_CONFIGS[key] || TILE_CONFIGS.osm;
        if (_tileLayer) _map.removeLayer(_tileLayer);
        _tileLayer = L.tileLayer(cfg.url, { attribution: cfg.attribution, maxZoom: 19 });
        _tileLayer.addTo(_map);
    }

    function init(elementId) {
        _map = L.map(elementId, {
            center: [34.057, -117.821],
            zoom: 15,
            zoomControl: true,
        });
        setTile('osm');
        _pathLayer    = L.layerGroup().addTo(_map);
        _obsLayer     = L.layerGroup().addTo(_map);
        _currentLayer = L.layerGroup().addTo(_map);
        _maskLayer    = L.layerGroup().addTo(_map);
        _resultLayer  = L.layerGroup().addTo(_map);
        _customLayer  = L.layerGroup().addTo(_map);

        _heatLayer = L.heatLayer([], {
            radius: _heatRadius,
            blur: _heatBlur,
            maxZoom: 17,
            max: 1.0,
            minOpacity: _heatOpacity * 0.3,
            gradient: { 0.2: '#00d4ff', 0.5: '#9b59ff', 0.8: '#ff4d6a', 1.0: '#ff0000' },
        }).addTo(_map);

        // Right-click / Long-press to place custom markers
        _map.on('contextmenu', (e) => {
            const markerId = Date.now();
            const defaultColor = '#4f8ef7';
            const marker = L.marker(e.latlng, { 
                draggable: true,
                icon: _customIcon(defaultColor)
            }).addTo(_customLayer);
            
            const obj = { 
                id: markerId, 
                marker: marker, 
                name: `Custom Marker ${_customMarkers.length + 1}`,
                color: defaultColor,
                locked: false 
            };
            _customMarkers.push(obj);

            marker.bindPopup('', { autoClose: false, closeOnClick: false, className: 'custom-premium-popup' }).openPopup();

            marker.on('drag', _refreshCustomMarkers);
            marker.on('contextmenu', (me) => {
                L.DomEvent.stopPropagation(me);
                window.removeCustomMarker(markerId);
            });
            _refreshCustomMarkers();
        });

        // ── Custom draggable popup logic for "Premium Popups" ──
        _map.on('popupopen', function(e) {
            const el = e.popup.getElement();
            if (el && el.classList.contains('custom-premium-popup')) {
                let isDragging = false;
                let startX, startY, startLeft, startBottom;
                
                el.style.cursor = 'move';
                el.addEventListener('mousedown', (evt) => {
                    if (evt.target.tagName === 'INPUT' || evt.target.tagName === 'BUTTON') return;
                    isDragging = true;
                    startX = evt.clientX;
                    startY = evt.clientY;
                    
                    const compStyle = window.getComputedStyle(el);
                    startLeft = parseInt(compStyle.marginLeft, 10) || 0;
                    startBottom = parseInt(compStyle.marginBottom, 10) || 0;
                    
                    L.DomEvent.disableClickPropagation(el);
                    _map.dragging.disable();
                });
                
                window.addEventListener('mousemove', (evt) => {
                    if (!isDragging) return;
                    const dx = evt.clientX - startX;
                    const dy = evt.clientY - startY;
                    el.style.marginLeft = (startLeft + dx) + 'px';
                    el.style.marginBottom = (startBottom - dy) + 'px';
                });
                
                window.addEventListener('mouseup', () => {
                    if (isDragging) {
                        isDragging = false;
                        _map.dragging.enable();
                    }
                });
            }
        });
    }

    function _customIcon(color) {
        const svg = `<svg viewBox="0 0 24 36" fill="${color}" stroke="#ffffff" stroke-width="2" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 2C6.48 2 2 6.48 2 12C2 19.5 12 34 12 34C12 34 22 19.5 22 12C22 6.48 17.52 2 12 2Z"/>
            <circle cx="12" cy="12" r="4" fill="#ffffff" stroke="none"/>
        </svg>`;
        return L.divIcon({
            className: 'custom-pin-icon',
            html: `<div style="width:16px; height:24px; filter: drop-shadow(0px 2px 2px rgba(0,0,0,0.5));">${svg}</div>`,
            iconSize: [16, 24],
            iconAnchor: [8, 24],
            popupAnchor: [0, -24]
        });
    }

    function _refreshCustomMarkers() {
        const unit = document.getElementById('setting-units')?.value || 'metric';

        // 1. Update Polyline
        const latlngs = _customMarkers.map(o => o.marker.getLatLng());
        if (_customPolyline) _customLayer.removeLayer(_customPolyline);
        if (latlngs.length > 1) {
            _customPolyline = L.polyline(latlngs, { color: '#f39c12', dashArray: '5 5', weight: 1.5, opacity: 0.6 }).addTo(_customLayer);
        } else {
            _customPolyline = null;
        }

        // 2. Update Popups
        for (let i = 0; i < _customMarkers.length; i++) {
            const obj = _customMarkers[i];
            const pos = obj.marker.getLatLng();
            let distHtml = '';

            if (i > 0) {
                const prevPos = _customMarkers[i-1].marker.getLatLng();
                const distM = pos.distanceTo(prevPos);
                if (unit === 'imperial') {
                    const distFt = distM * 3.28084;
                    const distMi = distFt / 5280;
                    distHtml = distMi > 0.1 ? `${distMi.toFixed(2)} mi from prev` : `${distFt.toFixed(0)} ft from prev`;
                } else {
                    distHtml = distM > 1000 ? `${(distM/1000).toFixed(2)} km from prev` : `${distM.toFixed(0)} m from prev`;
                }
            }

            const colors = ['#4f8ef7', '#00e87a', '#ffb84d', '#ff4d6a', '#9b59ff'];
            const colorSwatches = colors.map(c => 
                `<div class="color-swatch ${obj.color === c ? 'active' : ''}" 
                      style="background:${c};" 
                      onclick="window.setCustomMarkerColor(${obj.id}, '${c}')"></div>`
            ).join('');

            obj.marker.setPopupContent(`
                <div style="min-width: 180px;">
                    <input type="text" value="${obj.name}" 
                           onchange="window.saveCustomMarkerName(${obj.id}, this.value)" 
                           style="background: transparent; border: none; border-bottom: 2px solid transparent; color: var(--text-primary); font-family: 'Inter', sans-serif; font-weight: 700; font-size: 1.0rem; text-align: center; width: 100%; margin-bottom: 4px; padding-bottom: 2px; outline: none; transition: border-color 0.2s;"
                           onfocus="this.style.borderBottom='2px solid var(--border-color)'"
                           onblur="this.style.borderBottom='2px solid transparent'"
                           title="Rename Marker">
                    <div class="marker-popup-coords">
                        ${pos.lat.toFixed(6)}, ${pos.lng.toFixed(6)}
                    </div>
                    
                    <div class="marker-color-picker">
                        ${colorSwatches}
                    </div>
                    
                    ${distHtml ? `<div style="color: var(--accent-green); font-size: 0.75rem; font-weight: 600; text-align: center; margin-bottom: 8px;">${distHtml}</div>` : ''}
                    
                    <div class="marker-popup-footer">
                        <label style="font-size: 0.7rem; color: var(--text-muted); cursor: pointer; display: flex; align-items: center; gap: 4px;">
                            <input type="checkbox" ${obj.locked ? 'checked' : ''} onclick="window.toggleCustomMarkerLock(${obj.id})"> 
                            Lock
                        </label>
                        <span style="color: var(--accent-red); font-size: 0.7rem; font-weight: 700; cursor: pointer; text-transform: uppercase; letter-spacing: 0.05em;" 
                              onclick="window.removeCustomMarker(${obj.id})">Delete</span>
                    </div>
                </div>
            `);
        }
    }

    /**
     * update(data, result)
     * data: { observation_history[], current_observation, frequency_hz, doa_method }
     * result: { lat, lon, residual_m, stationsUsed } | null
     */
    function update(data, result, validObs = null) {
        if (!_map) return;
        const history = data?.observation_history ?? [];
        const current = data?.current_observation ?? null;

        const validArray = validObs || history;
        const validSet = new Set(validArray.map(o => o.id));

        _currentLayer.clearLayers();
        // Clear history layer every update to ensure real-time color changes when mask moves
        _obsLayer.clearLayers();
        _drawnObsIds.clear();

        if (history.length === 0) return;

        const bounds = [];

        // ── Historical Path & Observation Log ──────────────────────────
        const latlngs = [];
        const freqMHz = data?.frequency_hz ? (data.frequency_hz / 1e6).toFixed(4) : '—';
        const _showLines = true; // Always show lines for now

        history.forEach((obs, index) => {
            latlngs.push([obs.lat, obs.lon]);
            
            const isUsed = validSet.has(obs.id);
            const maskActive = typeof window.getMaskBounds === 'function' && window.getMaskBounds() !== null;
            const polyActive = typeof window.getPolyBounds === 'function' && window.getPolyBounds() !== null;
            
            // "Anything below a set confidence threshold should hide the bearing vectors"
            // "any bearing vector that does not intersect with a custom drawn area should be hidden"
            if (!isUsed) return;
            
            // "For those bearing vectors that do intersect with this drawn area should have their bearing colors change to magenta"
            const curBearingColor = (maskActive || polyActive) ? '#ff00ff' : BEARING_COLOR;

            _drawnObsIds.add(obs.id);

            // Use the normalized solver score for color, but display the raw
            // PAPR metric separately when it is available.
            const scoreValue = Number(obs.quality_score ?? obs.confidence);
            const score = Number.isFinite(scoreValue) ? Math.max(0, Math.min(1, scoreValue)) : 1;
            let dotColor = '#e74c3c'; // Red
            if (score >= 0.7) dotColor = '#2ecc71'; // Green
            else if (score >= 0.4) dotColor = '#f1c40f'; // Yellow

            const papr = Number(obs.doa_papr_db);
            const qualityText = Number.isFinite(papr)
                ? `${papr.toFixed(2)} dB PAPR`
                : Number.isFinite(scoreValue)
                    ? `${(score * 100).toFixed(0)}% ${obs.quality_kind === 'legacy_confidence' ? 'legacy' : 'score'}`
                    : '—';
            const usableText = obs.usable_for_triangulation === false ? 'NO' : 'YES';
            const obsFreqMHz = obs.frequency_hz
                ? (obs.frequency_hz / 1e6).toFixed(4)
                : freqMHz;

            const cMarker = L.circleMarker([obs.lat, obs.lon], {
                radius: 4, fillColor: dotColor, color: '#000', weight: 1, fillOpacity: 1
            });
            
            // Informational Popup
            cMarker.bindPopup(`
                <b>Log Number: ${index + 1}</b><br>
                RDF Bearing: ${obs.bearing_deg.toFixed(1)}&deg;<br>
                Vehicle Heading: ${(obs.heading_used_deg || 0).toFixed(1)}&deg;<br>
                Quality: ${qualityText}<br>
                Upstream usable: ${usableText}<br>
                Frequency: ${obsFreqMHz} MHz
            `, { autoClose: false, closeOnClick: false });
            _obsLayer.addLayer(cMarker);

            // Short blue/magenta bearing vector for history
            if (_showLines) {
                const vec = TriangulationHelpers.projectPoint(obs.lat, obs.lon, obs.bearing_deg, Math.min(600, _lineLengthKm * 100));
                _obsLayer.addLayer(L.polyline([
                    [obs.lat, obs.lon], [vec.lat, vec.lon]
                ], { color: curBearingColor, weight: isUsed ? 2 : 1, opacity: isUsed ? 0.9 : 0.4 }));
            }
        });

        // The actual connected path line (we redraw this strictly from the unfiltered `history` array 
        // to ensure the vehicle polyline doesn't look like teleportation between high-confidence waypoints)
        _pathLayer.clearLayers();
        if (latlngs.length > 1) {
            _pathLayer.addLayer(L.polyline(latlngs, { color: '#888', weight: 2, dashArray: '5, 5', opacity: 0.5 }));
        }

        // ── Current Observation point ──────────────────────────────────
        if (current) {
            bounds.push([current.lat, current.lon]);
            
            // Red Vehicle Heading Line
            const headVec = TriangulationHelpers.projectPoint(
                current.lat, current.lon, current.heading_used_deg || 0, _lineLengthKm * 50
            );
            _currentLayer.addLayer(L.polyline([
                [current.lat, current.lon], [headVec.lat, headVec.lon]
            ], { color: HEADING_COLOR, weight: 3 }));

            // The main long bearing line to estimated tx
            if (_showLines) {
                const brngVec = TriangulationHelpers.projectPoint(
                    current.lat, current.lon, current.bearing_deg, _lineLengthKm * 1000
                );
                _currentLayer.addLayer(L.polyline([
                    [current.lat, current.lon], [brngVec.lat, brngVec.lon]
                ], { color: BEARING_COLOR, weight: 2 }));
            }
            
            // Current dot
            _currentLayer.addLayer(L.circleMarker([current.lat, current.lon], {
                color: '#fff', fillColor: '#3498db', fillOpacity: 1, radius: 6, weight: 2
            }));
        }

        // ── Triangulation result ─────────────────────────────────────
        if (result) {
            bounds.push([result.lat, result.lon]);

            // Green Octagon for Estimated Source (KrakenSDR style)
            const octagonPoints = TriangulationHelpers.createPolygon(
                result.lat, result.lon, Math.max(result.residual_m, 1), 8  // 1 meter minimum for visual rendering
            );
            
            if (!_resultCrosshair) {
                _resultCrosshair = L.polygon(octagonPoints, {
                    color: RESULT_COLOR,
                    weight: 4,
                    fill: false
                }).addTo(_resultLayer);

                // Small dot in center
                _resultCenterDot = L.circleMarker([result.lat, result.lon], {
                    color: RESULT_COLOR, fillColor: RESULT_COLOR, fillOpacity: 1, radius: 3, weight: 1
                }).addTo(_resultLayer);

                // Add premium dark popup specific class
                _resultCrosshair.bindPopup('', { autoClose: false, closeOnClick: false, className: 'custom-premium-popup' });
            } else {
                _resultCrosshair.setLatLngs(octagonPoints);
                _resultCenterDot.setLatLng([result.lat, result.lon]);
            }

            const isImp = document.getElementById('setting-units')?.value === 'imperial';
            let radiusStr = '';
            if (isImp) {
                const ft = result.residual_m * 3.28084;
                if (ft >= 5280) radiusStr = `${(ft / 5280).toFixed(2)} mi`;
                else radiusStr = `${ft.toFixed(1)} ft`;
            } else {
                if (result.residual_m >= 1000) radiusStr = `${(result.residual_m / 1000).toFixed(2)} km`;
                else radiusStr = `${result.residual_m.toFixed(1)} m`;
            }

            _resultCrosshair.setPopupContent(`
                <div style="min-width: 180px;">
                    <div class="marker-popup-title">Estimated Source</div>
                    <div class="marker-popup-coords">
                        ${result.lat.toFixed(6)}, ${result.lon.toFixed(6)}
                    </div>
                    <div style="font-size: 0.75rem; color: var(--text-secondary); text-align: center; margin-top: 8px;">
                        Uncertainty Radius: <span style="color: var(--text-primary); font-weight: 600;">${radiusStr}</span><br>
                        From <span style="color: var(--text-primary); font-weight: 600;">${result.stationsUsed}</span> obs.
                    </div>
                </div>
            `);

        } else {
            if (_resultCrosshair) {
                _resultLayer.clearLayers();
                _resultCrosshair = null;
                _resultCenterDot = null;
            }
        }

        // Only snap to the bounds on the very first load to allow user complete zoom/pan freedom.
        if (bounds.length > 1 && _firstLoad) {
            _map.fitBounds(bounds, { padding: [40, 40], maxZoom: 17 });
            _firstLoad = false;
        }
    }

    function setLineLength(km) { _lineLengthKm = km; }
    function setShowUncertainty(val) { _showUncertainty = val; }
    function invalidateSize() { if (_map) _map.invalidateSize(); }

    function addHeatPoint(lat, lon, intensity = 1.0) {
        _heatPoints.push([lat, lon, intensity]);
        if (_heatLayer) _heatLayer.setLatLngs(_heatPoints);
    }
    function clearHeat() {
        _heatPoints = [];
        if (_heatLayer) _heatLayer.setLatLngs([]);
    }
    function setHeatGrid(pointsArray) {
        _heatPoints = pointsArray;
        if (_heatLayer) _heatLayer.setLatLngs(_heatPoints);
    }
    function setHeatRadius(r) {
        _heatRadius = r;
        if (_heatLayer) _heatLayer.setOptions({ radius: r });
    }
    function setHeatBlur(b) {
        _heatBlur = b;
        if (_heatLayer) _heatLayer.setOptions({ blur: b });
    }
    function setHeatOpacity(o) {
        _heatOpacity = o;
        if (_heatLayer) _heatLayer.setOptions({ minOpacity: o * 0.3 });
    }
    
    function getHeatPointCount() { 
        return _heatPoints.length; 
    }

    function getMaskGeoJSON() {
        if (!_maskPolygon) return null;
        const pts = _maskPolygon.getLatLngs()[0].map(ll => {
            return [ll.lng, ll.lat]; // Turf uses GeoJSON [lng, lat] format
        });
        if (pts.length < 3) return null;
        pts.push([pts[0][0], pts[0][1]]); // close loop safely
        return turf.polygon([pts]);
    }

    // ── Ground Truth Layer ──────────────────────────────────────────
    let _gtLayer = null;
    let _gtMarkers = [];  // { id, lat, lon, label, marker, circle }

    function _gtIcon() {
        // Distinctive green crosshair with pulsing outer ring
        return L.divIcon({
            className: '',
            html: `<div style="position:relative; width:24px; height:24px;">
                <div style="position:absolute; inset:0; border:2px solid #00e87a; border-radius:50%; box-shadow:0 0 10px rgba(0,232,122,0.5); animation: gt-pulse 2s infinite;"></div>
                <div style="position:absolute; top:50%; left:0; right:0; height:2px; background:#00e87a; transform:translateY(-50%);"></div>
                <div style="position:absolute; left:50%; top:0; bottom:0; width:2px; background:#00e87a; transform:translateX(-50%);"></div>
                <div style="position:absolute; top:50%; left:50%; width:6px; height:6px; background:#00e87a; border-radius:50%; transform:translate(-50%,-50%); box-shadow:0 0 6px #00e87a;"></div>
            </div>
            <style>@keyframes gt-pulse { 0%,100% { opacity:1; transform:scale(1); } 50% { opacity:0.5; transform:scale(1.15); } }</style>`,
            iconSize: [24, 24],
            iconAnchor: [12, 12],
            popupAnchor: [0, -14],
        });
    }

    function addGroundTruth(lat, lon, label) {
        if (!_map) return null;
        if (!_gtLayer) _gtLayer = L.layerGroup().addTo(_map);

        const id = Date.now();
        const marker = L.marker([lat, lon], {
            icon: _gtIcon(),
            zIndexOffset: 2000,
        }).addTo(_gtLayer);

        // Accuracy circle (20m default visual)
        const circle = L.circle([lat, lon], {
            radius: 8,
            color: '#00e87a',
            weight: 1.5,
            fillColor: '#00e87a',
            fillOpacity: 0.08,
            dashArray: '4 4',
        }).addTo(_gtLayer);

        const displayLabel = label || `Marker #${_gtMarkers.length + 1}`;
        marker.bindPopup(`
            <div style="min-width:140px;">
                <div class="marker-popup-title" style="color:#00e87a;">${displayLabel}</div>
                <div class="marker-popup-coords">${lat.toFixed(6)}, ${lon.toFixed(6)}</div>
                <div style="text-align:center; font-size:0.65rem; color:var(--text-muted); margin-top:4px;">Plotted Coordinate</div>
            </div>
        `, { className: 'custom-premium-popup' });

        const obj = { id, lat, lon, label: displayLabel, marker, circle };
        _gtMarkers.push(obj);

        // Pan to the new marker
        _map.setView([lat, lon], Math.max(_map.getZoom(), 15));

        return obj;
    }

    function removeGroundTruth(id) {
        const idx = _gtMarkers.findIndex(o => o.id === id);
        if (idx === -1) return;
        const obj = _gtMarkers[idx];
        if (obj.marker) obj.marker.remove();
        if (obj.circle) obj.circle.remove();
        _gtMarkers.splice(idx, 1);
    }

    function clearGroundTruth() {
        if (_gtLayer) _gtLayer.clearLayers();
        _gtMarkers = [];
    }

    function getGroundTruthMarkers() {
        return _gtMarkers.map(o => ({ id: o.id, lat: o.lat, lon: o.lon, label: o.label }));
    }

    return { 
        init, update, setTile, setLineLength, setShowUncertainty, invalidateSize,
        addHeatPoint, clearHeat, setHeatGrid, setHeatRadius, setHeatBlur, setHeatOpacity, getHeatPointCount,
        getMaskGeoJSON, refreshCustomMarkers: _refreshCustomMarkers,
        addGroundTruth, removeGroundTruth, clearGroundTruth, getGroundTruthMarkers,
    };
})();

