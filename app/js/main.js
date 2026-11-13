/**
 * main.js — App Orchestrator (Single Mobile KrakenSDR Model)
 * ===========================================================
 * Wires tab routing, settings, bearing log, and the
 * full data pipeline:
 *   DataFeed -> Triangulation.solve(observation_history) -> MapView + HeatmapView
 *
 * Data schema: { observation_history[], current_observation, frequency_hz,
 *                doa_method }
 *
 * Timestamp note:
 *   "LAST RECEIVED" shows the laptop system clock at the moment the poll
 *   response arrived.
 */

(function () {

    // ── Tab Routing ────────────────────────────────────────────────
    const tabButtons = document.querySelectorAll('.tab-btn');
    const tabPanels  = document.querySelectorAll('.tab-panel');

    let _estimationMap = null;
    let _estimationMarkers = null;
    let _estimationHits = [];
    let _estIdCounter = 0;

    function initEstimationMap() {
        if (_estimationMap) {
            _estimationMap.invalidateSize();
            return;
        }
        _estimationMap = L.map('estimation-map', { zoomControl: true });
        // Use initial view near equator or something safe until first hit
        _estimationMap.setView([0, 0], 2);
        
        const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '© OpenStreetMap'
        });
        tileLayer.addTo(_estimationMap);
        _estimationMarkers = L.layerGroup().addTo(_estimationMap);

        // sync tile layer setting if exists
        const elTile = document.getElementById('setting-tile');
        if (elTile && typeof TILE_CONFIGS !== 'undefined') {
            const tc = TILE_CONFIGS[elTile.value] || TILE_CONFIGS['osm'];
            tileLayer.setUrl(tc.url);
        }
    }

    function switchTab(targetId) {
        tabButtons.forEach(btn => btn.classList.toggle('active', btn.dataset.tab === targetId));
        tabPanels.forEach(panel => {
            panel.classList.toggle('active', panel.id === `panel-${targetId}`);
        });
        if (targetId === 'map') requestAnimationFrame(() => MapView.invalidateSize());
        if (targetId === 'estimation') requestAnimationFrame(() => initEstimationMap());
    }

    function _formatCoord(c) {
        return c.toFixed(6);
    }

    // ── Convergence Computation ─────────────────────────────────
    function _computeConvergence() {
        if (_estimationHits.length < 2) return null;

        let totalWeight = 0;
        let wLat = 0, wLon = 0;

        for (const hit of _estimationHits) {
            // Weight = 1 / residual_m (clamped to avoid division by near-zero)
            const residual = Math.max(hit.residual_m ?? 1, 0.1);
            const w = 1.0 / residual;
            wLat += w * hit.lat;
            wLon += w * hit.lon;
            totalWeight += w;
        }

        const cLat = wLat / totalWeight;
        const cLon = wLon / totalWeight;

        // Weighted standard deviation (spread) in meters
        let wSumSqDist = 0;
        for (const hit of _estimationHits) {
            const residual = Math.max(hit.residual_m ?? 1, 0.1);
            const w = 1.0 / residual;
            const distM = Triangulation.distanceMeters(cLat, cLon, hit.lat, hit.lon);
            wSumSqDist += w * distM * distM;
        }
        const spreadM = Math.sqrt(wSumSqDist / totalWeight);

        return {
            lat: cLat,
            lon: cLon,
            spreadM: spreadM,
            count: _estimationHits.length
        };
    }

    function _renderEstimationLog() {
        const histContainer = document.getElementById('est-log-history');
        const hitCountEl = document.getElementById('est-hit-count');
        if (!histContainer) return;

        const convergenceEnabled = document.getElementById('setting-convergence')?.checked ?? true;

        // ── Update Converged Estimate Card ──
        _updateConvergedCard(convergenceEnabled);

        // ── Render Hit Log ──
        histContainer.innerHTML = '';
        if (hitCountEl) hitCountEl.textContent = `${_estimationHits.length} hits`;

        if (_estimationHits.length === 0) {
            histContainer.innerHTML = `<div style="font-size:0.75rem; color:var(--text-muted); font-style:italic; padding:8px; text-align:center;">No hits recorded inside search area.</div>`;
            return;
        }

        const normHits = [..._estimationHits].reverse();

        normHits.forEach(hit => {
            const itemDiv = document.createElement('div');
            itemDiv.className = `estimation-item significant`;
            const isImperial = document.getElementById('setting-units')?.value === 'imperial';
            let residualLabel = '—';
            if (hit.residual_m != null) {
                if (isImperial) {
                    const ft = hit.residual_m * 3.28084;
                    residualLabel = ft >= 5280 ? (ft / 5280).toFixed(2) + ' mi' : ft.toFixed(1) + ' ft';
                } else {
                    residualLabel = hit.residual_m >= 1000
                        ? (hit.residual_m / 1000).toFixed(2) + ' km'
                        : hit.residual_m.toFixed(1) + ' m';
                }
            }
            itemDiv.innerHTML = `
                <div class="est-row">
                    <span>Lat/Lon</span>
                    <span class="est-val">${_formatCoord(hit.lat)}, ${_formatCoord(hit.lon)}</span>
                </div>
                <div class="est-row">
                    <span>Residual</span>
                    <span class="est-val">${residualLabel}</span>
                </div>
                <div class="est-row">
                    <span>Time</span>
                    <span class="est-val">${new Date(hit.timestamp).toLocaleTimeString()}</span>
                </div>
                <div class="est-actions">
                    <button class="est-btn transmit" data-id="${hit.id}">Transmit</button>
                    <button class="est-btn delete" data-id="${hit.id}">Delete</button>
                </div>
            `;
            histContainer.appendChild(itemDiv);
        });

        // Bind buttons
        document.querySelectorAll('.est-btn.transmit').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const id = parseInt(e.target.dataset.id);
                const hit = _estimationHits.find(h => h.id === id);
                if (hit) {
                    console.log(`[Telemetry] TRANSMITTING: ${hit.lat}, ${hit.lon}`);
                    _transmitToGCS(hit.lat, hit.lon, 0, 1)
                        .then(() => alert(`Transmitted coordinates to GCS: \nLat: ${hit.lat}\nLon: ${hit.lon}`))
                        .catch(e => alert(`Transmission failed: ${e}`));
                }
            });
        });
        document.querySelectorAll('.est-btn.delete').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const id = parseInt(e.target.dataset.id);
                _estimationHits = _estimationHits.filter(h => h.id !== id);
                _redrawEstimationMarkers();
                _renderEstimationLog();
            });
        });
    }

    function _updateConvergedCard(enabled) {
        const card = document.getElementById('converged-estimate-card');
        const badge = document.getElementById('converged-status-badge');
        const btnTransmit = document.getElementById('btn-conv-transmit');
        if (!card) return;

        if (!enabled) {
            card.className = 'converged-card converged-disabled';
            if (badge) badge.textContent = 'OFF';
            if (btnTransmit) btnTransmit.disabled = true;
            return;
        }

        const conv = _computeConvergence();

        if (!conv) {
            card.className = 'converged-card converged-active';
            if (badge) badge.textContent = _estimationHits.length === 0 ? 'WAITING' : '1 HIT';
            document.getElementById('conv-lat').textContent = _estimationHits.length === 1 ? _estimationHits[0].lat.toFixed(6) + '°' : '—';
            document.getElementById('conv-lon').textContent = _estimationHits.length === 1 ? _estimationHits[0].lon.toFixed(6) + '°' : '—';
            document.getElementById('conv-count').textContent = `${_estimationHits.length} hits`;
            document.getElementById('conv-spread').textContent = '—';
            if (btnTransmit) btnTransmit.disabled = _estimationHits.length < 1;
            return;
        }

        card.className = 'converged-card converged-active';
        if (badge) badge.textContent = 'LIVE';

        document.getElementById('conv-lat').textContent = conv.lat.toFixed(6) + '°';
        document.getElementById('conv-lon').textContent = conv.lon.toFixed(6) + '°';
        document.getElementById('conv-count').textContent = `${conv.count} hits`;

        // Display spread in appropriate units
        const isImperial = document.getElementById('setting-units')?.value === 'imperial';
        if (isImperial) {
            const spreadFt = conv.spreadM * 3.28084;
            document.getElementById('conv-spread').textContent = spreadFt >= 5280
                ? `± ${(spreadFt / 5280).toFixed(2)} mi`
                : `± ${spreadFt.toFixed(1)} ft`;
        } else {
            document.getElementById('conv-spread').textContent = conv.spreadM >= 1000
                ? `± ${(conv.spreadM / 1000).toFixed(2)} km`
                : `± ${conv.spreadM.toFixed(1)} m`;
        }

        if (btnTransmit) btnTransmit.disabled = false;
    }

    function _redrawEstimationMarkers() {
        if (!_estimationMarkers) return;
        _estimationMarkers.clearLayers();

        // Individual hit markers
        _estimationHits.forEach(hit => {
            L.circleMarker([hit.lat, hit.lon], {
                radius: 5,
                color: '#fff',
                weight: 1,
                fillColor: '#00e87a',
                fillOpacity: 0.8
            }).addTo(_estimationMarkers);
        });

        // Converged centroid marker (if enabled and ≥ 2 hits)
        const convergenceEnabled = document.getElementById('setting-convergence')?.checked ?? true;
        if (convergenceEnabled) {
            const conv = _computeConvergence();
            if (conv) {
                // Outer glow ring
                L.circleMarker([conv.lat, conv.lon], {
                    radius: 14,
                    color: '#00e87a',
                    weight: 2,
                    fillColor: '#00e87a',
                    fillOpacity: 0.12,
                    dashArray: '4 4'
                }).addTo(_estimationMarkers);

                // Inner crosshair dot
                L.circleMarker([conv.lat, conv.lon], {
                    radius: 7,
                    color: '#fff',
                    weight: 2,
                    fillColor: '#00e87a',
                    fillOpacity: 1.0
                }).addTo(_estimationMarkers)
                    .bindTooltip(`Converged: ${conv.lat.toFixed(6)}, ${conv.lon.toFixed(6)}`, {
                        permanent: false,
                        direction: 'top',
                        offset: [0, -12]
                    });
            }
        }
    }

    function _addEstimationHit(result) {
        // Prevent duplicate spam if location hasn't moved much (< 1 meter approx)
        const isSpam = _estimationHits.some(h => {
            if (Triangulation && Triangulation.distanceMeters) {
                return Triangulation.distanceMeters(h.lat, h.lon, result.lat, result.lon) < 1.0;
            }
            return Math.abs(h.lat - result.lat) < 0.00001 && Math.abs(h.lon - result.lon) < 0.00001;
        });

        if (isSpam) return;

        const hit = {
            id: ++_estIdCounter,
            lat: result.lat,
            lon: result.lon,
            residual_m: result.residual_m ?? null,
            timestamp: Date.now()
        };

        _estimationHits.push(hit);
        _redrawEstimationMarkers();
        _renderEstimationLog();
        
        // Auto-pan if we are at map default view [0,0]
        if (_estimationMap && _estimationMap.getZoom() === 2) {
            _estimationMap.setView([result.lat, result.lon], 17);
        }
    }

    // Set up clear button and converged transmit
    document.addEventListener('DOMContentLoaded', () => {
        document.getElementById('btn-est-clear-all')?.addEventListener('click', () => {
            if (confirm("Clear all estimation history?")) {
                _estimationHits = [];
                _redrawEstimationMarkers();
                _renderEstimationLog();
            }
        });

        // Converged Estimate transmit button
        document.getElementById('btn-conv-transmit')?.addEventListener('click', () => {
            const convergenceEnabled = document.getElementById('setting-convergence')?.checked ?? true;
            if (!convergenceEnabled) return;

            const conv = _computeConvergence();
            if (conv) {
                console.log(`[Telemetry] TRANSMITTING CONVERGED: ${conv.lat}, ${conv.lon} (±${conv.spreadM.toFixed(1)}m, ${conv.count} hits)`);
                _transmitToGCS(conv.lat, conv.lon, conv.spreadM, conv.count)
                    .then(() => alert(`Transmitted CONVERGED coordinates to GCS:\nLat: ${conv.lat.toFixed(6)}\nLon: ${conv.lon.toFixed(6)}\nSpread: ±${conv.spreadM.toFixed(1)}m\nBased on: ${conv.count} hits`))
                    .catch(e => alert(`Transmission failed: ${e}`));
            } else if (_estimationHits.length === 1) {
                // Single hit — transmit directly
                const hit = _estimationHits[0];
                console.log(`[Telemetry] TRANSMITTING SINGLE: ${hit.lat}, ${hit.lon}`);
                _transmitToGCS(hit.lat, hit.lon, 0, 1)
                    .then(() => alert(`Transmitted coordinates to GCS:\nLat: ${hit.lat.toFixed(6)}\nLon: ${hit.lon.toFixed(6)}`))
                    .catch(e => alert(`Transmission failed: ${e}`));
            }
        });
    });

    tabButtons.forEach(btn => btn.addEventListener('click', () => switchTab(btn.dataset.tab)));
    switchTab('map');

    // ── Settings Wiring ────────────────────────────────────────────
    const elPollInterval  = document.getElementById('setting-poll-interval');
    const elTile          = document.getElementById('setting-tile');
    const elUncertainty   = document.getElementById('setting-uncertainty');
    const elLineLength    = document.getElementById('setting-line-length');
    const elAlgo          = document.getElementById('setting-algo');
    const elMinConf       = document.getElementById('setting-min-conf');
    const elUnits         = document.getElementById('setting-units');
    const elConvergence   = document.getElementById('setting-convergence');

    function bindSetting(el) {
        if (!el) return;
        const key = `kraken_setting_${el.id}`;
        const saved = localStorage.getItem(key);
        
        if (saved !== null) {
            if (el.type === 'checkbox') el.checked = (saved === 'true');
            else el.value = saved;
        }
        
        el.addEventListener('change', () => {
            const val = el.type === 'checkbox' ? el.checked : el.value;
            localStorage.setItem(key, val);
        });
    }

    const _allSettings = [elPollInterval, elTile, elUncertainty, elLineLength, elAlgo, elMinConf, elUnits, 
        document.getElementById('setting-filter-spatial'), 
        document.getElementById('setting-filter-temporal'), 
        document.getElementById('setting-filter-attitude'), 
        document.getElementById('setting-filter-angular'),
        elConvergence];
        
    _allSettings.forEach(bindSetting);

    // Convergence toggle → immediately re-render estimation UI
    elConvergence?.addEventListener('change', () => {
        _renderEstimationLog();
    });

    elPollInterval?.addEventListener('change', () =>
        DataFeed.setPollInterval(parseInt(elPollInterval.value, 10) || 2000));
    elTile?.addEventListener('change', () => MapView.setTile(elTile.value));
    elUncertainty?.addEventListener('change', () => MapView.setShowUncertainty(elUncertainty.checked));
    elLineLength?.addEventListener('change', () => {
        const isImp = document.getElementById('setting-units')?.value === 'imperial';
        let len = parseFloat(elLineLength.value) || 2;
        if (isImp) len = len * 1.60934;
        MapView.setLineLength(len);
        if (_lastData) _processData(_lastData);
    });

    const elFilterSpatial = document.getElementById('setting-filter-spatial');
    const elFilterTemporal = document.getElementById('setting-filter-temporal');
    const elFilterAttitude = document.getElementById('setting-filter-attitude');
    const elFilterAngular = document.getElementById('setting-filter-angular');

    [elFilterSpatial, elFilterTemporal, elFilterAttitude, elFilterAngular, elAlgo, elMinConf].forEach(el => {
        el?.addEventListener('change', () => {
            if (_lastData) _processData(_lastData);
        });
    });

    elUnits?.addEventListener('change', () => {
        MapView.refreshCustomMarkers();
        
        const isImperial = elUnits.value === 'imperial';
        const isPrevImperial = elUnits.dataset.prev === 'imperial';
        
        // Sync Mask UI units
        document.querySelectorAll('.mask-unit-lbl').forEach(lbl => {
            lbl.textContent = isImperial ? 'mi' : 'km';
        });
        
        const lineLenLbl = document.getElementById('line-length-unit');
        if (lineLenLbl) lineLenLbl.textContent = isImperial ? 'mi' : 'km';
        
        if (elUnits.dataset.prev && isImperial !== isPrevImperial) {
            // Update line length value directly
            if (elLineLength) {
                let v = parseFloat(elLineLength.value);
                if (!isNaN(v)) {
                    elLineLength.value = isImperial ? (v / 1.60934).toFixed(1) : (v * 1.60934).toFixed(1);
                    elLineLength.dispatchEvent(new Event('change'));
                }
            }
        }
        
        elUnits.dataset.prev = elUnits.value;
        if (window._syncMaskUI) window._syncMaskUI();
        if (_lastData) _processData(_lastData);
        _renderEstimationLog();
        _refreshGTList();
    });
    // Trigger initial label sync
    elUnits.dataset.prev = elUnits.value;
    elUnits?.dispatchEvent(new Event('change'));

    document.getElementById('btn-shutdown')?.addEventListener('click', async () => {
        if (!confirm("Are you sure you want to stop the background tracking server? You will lose live telemetry. The terminal will exit.")) return;
        try {
            await fetch('/api/shutdown', { method: 'POST' });
            document.body.innerHTML = '<div style="display:flex; height:100vh; align-items:center; justify-content:center; color:#fff; text-align:center; font-family:sans-serif;"><h1>Server Shutdown Successfully. <br><span style="font-size:16px; color:#888;">You can safely close this browser tab.</span></h1></div>';
            setTimeout(() => window.close(), 500);
        } catch(e) {}
    });

    document.getElementById('btn-collapse-obs')?.addEventListener('click', () => {
        const card = document.getElementById('station-list-card');
        if (card) card.classList.toggle('obs-collapsed');
    });

    // ── Heatmap Controls ───────────────────────────────────────────
    function wireSlider(id, valId, factor, setFn) {
        const slider = document.getElementById(id);
        const label  = document.getElementById(valId);
        if (!slider) return;
        slider.addEventListener('input', () => {
            const v = parseInt(slider.value, 10);
            label.textContent = factor ? (v * factor).toFixed(1) : v;
            setFn(factor ? v * factor : v);
        });
    }
    wireSlider('hm-radius',  'hm-radius-val',  null, MapView.setHeatRadius);
    wireSlider('hm-blur',    'hm-blur-val',    null, MapView.setHeatBlur);
    wireSlider('hm-opacity', 'hm-opacity-val', 0.1,  MapView.setHeatOpacity);
    document.getElementById('hm-clear-btn')?.addEventListener('click', MapView.clearHeat);

    // ── Global App Logic ───────────────────────────────────────────
    window._triggerAppUpdate = () => { if (_lastData) _processData(_lastData); };

    document.getElementById('btn-master-clear')?.addEventListener('click', () => {
        if (!confirm('Clear all logged data, heatmaps, and running observation arrays?')) return;
        document.getElementById('log-clear-btn')?.click();
        MapView.clearHeat();
        if (_lastData && _lastData.observation_history) {
            _lastData.observation_history = [];
            _lastData.current_observation = null;
        }
        _processData(_lastData);
    });
    const syncMenuStates = () => {
        const maskToggle = document.getElementById('btn-draw-mask-toggle');
        const polyToggle = document.getElementById('btn-draw-poly-toggle');
        const maskBlock = document.getElementById('mask-tools-block');
        const polyBlock = document.getElementById('poly-tools-block');
        
        if (maskToggle && polyToggle && maskBlock && polyBlock) {
            const maskToggleRow = maskToggle.closest('div');
            const polyToggleRow = polyToggle.closest('div');
            
            // Reset state
            maskBlock.style.opacity = '1'; maskBlock.style.pointerEvents = 'auto';
            polyBlock.style.opacity = '1'; polyBlock.style.pointerEvents = 'auto';
            if (maskToggleRow) { maskToggleRow.style.opacity = '1'; maskToggleRow.style.pointerEvents = 'auto'; maskToggle.disabled = false; }
            if (polyToggleRow) { polyToggleRow.style.opacity = '1'; polyToggleRow.style.pointerEvents = 'auto'; polyToggle.disabled = false; }

            if (maskToggle.checked) {
                // Dim Poly entirely
                polyBlock.style.opacity = '0.3'; polyBlock.style.pointerEvents = 'none';
                if (polyToggleRow) { polyToggleRow.style.opacity = '0.3'; polyToggleRow.style.pointerEvents = 'none'; polyToggle.disabled = true; }
            } else if (polyToggle.checked) {
                // Dim Mask entirely
                maskBlock.style.opacity = '0.3'; maskBlock.style.pointerEvents = 'none';
                if (maskToggleRow) { maskToggleRow.style.opacity = '0.3'; maskToggleRow.style.pointerEvents = 'none'; maskToggle.disabled = true; }
            }
        }
    };

    document.getElementById('btn-draw-mask-toggle')?.addEventListener('change', (e) => {
        if (!window.drawDefaultMask) return;
        if (e.target.checked) {
            const polyToggle = document.getElementById('btn-draw-poly-toggle');
            if (polyToggle && polyToggle.checked) {
                polyToggle.checked = false;
                if (window.clearPoly) window.clearPoly();
            }
            window.drawDefaultMask(null, null); // Trigger interactive crosshair drawing mode immediately
        } else {
            if (window.clearMask) window.clearMask();
        }
        syncMenuStates();
    });
    
    document.getElementById('btn-draw-poly-toggle')?.addEventListener('change', (e) => {
        if (!window.drawPolygonMask) return;
        if (e.target.checked) {
            const maskToggle = document.getElementById('btn-draw-mask-toggle');
            if (maskToggle && maskToggle.checked) {
                maskToggle.checked = false;
                if (window.clearMask) window.clearMask();
            }
            const verts = parseInt(document.getElementById('poly-vertices')?.value || 6);
            const color = document.getElementById('poly-color')?.value || '#fa8231';
            window.drawPolygonMask(verts, color);
        } else {
            if (window.clearPoly) window.clearPoly();
        }
        syncMenuStates();
    });

    document.getElementById('btn-clear-mask')?.addEventListener('click', () => {
        const toggle = document.getElementById('btn-draw-mask-toggle');
        if (toggle) toggle.checked = false;
        if (window.clearMask) window.clearMask();
        syncMenuStates();
    });
    
    document.getElementById('btn-clear-poly')?.addEventListener('click', () => {
        const toggle = document.getElementById('btn-draw-poly-toggle');
        if (toggle) toggle.checked = false;
        if (window.clearPoly) window.clearPoly();
        syncMenuStates();
    });

    const updatePolyLive = () => {
        const toggle = document.getElementById('btn-draw-poly-toggle');
        if (toggle && toggle.checked && window.updatePolyStyle) {
            const verts = parseInt(document.getElementById('poly-vertices')?.value || 6);
            const color = document.getElementById('poly-color')?.value || '#fa8231';
            window.updatePolyStyle(verts, color);
        }
    };
    document.getElementById('poly-vertices')?.addEventListener('change', () => {
        updatePolyLive();
        if (window._triggerAppUpdate) window._triggerAppUpdate();
    });
    document.getElementById('poly-color')?.addEventListener('input', () => {
        updatePolyLive();
    });
    document.getElementById('poly-lock')?.addEventListener('change', (e) => {
        if (window.togglePolyLock) window.togglePolyLock(e.target.checked);
    });

    document.getElementById('btn-clear-all-overlays')?.addEventListener('click', () => {
        if (window.clearMask) window.clearMask();
        if (window.clearPoly) window.clearPoly();
        if (MapView.clearHeat) MapView.clearHeat();
        
        const maskToggle = document.getElementById('btn-draw-mask-toggle');
        if (maskToggle) maskToggle.checked = false;
        const polyToggle = document.getElementById('btn-draw-poly-toggle');
        if (polyToggle) polyToggle.checked = false;
        syncMenuStates();
    });
    
    window._triggerAppUpdate = () => {
        if (_lastData) _processData(_lastData);
    };

    // Helper to sync UI fields with map geometry
    window._syncMaskUI = (overrideBounds = null) => {
        const bounds = overrideBounds || (typeof window.getMaskBounds === 'function' ? window.getMaskBounds() : null);
        if (!bounds) return;
        
        const isImperial = (document.getElementById('setting-units')?.value === 'imperial');
        const factor = isImperial ? 1609.34 : 1000;
        
        // Calculate width/height in chosen units
        // Simple approximation for short distances
        const latMid = (bounds.minLat + bounds.maxLat) / 2;
        const widthM = L.latLng(latMid, bounds.minLon).distanceTo(L.latLng(latMid, bounds.maxLon));
        const heightM = L.latLng(bounds.minLat, bounds.minLon).distanceTo(L.latLng(bounds.maxLat, bounds.minLon));
        
        const elW = document.getElementById('mask-width');
        const elH = document.getElementById('mask-height');
        if (elW) elW.value = (widthM / factor).toFixed(2);
        if (elH) elH.value = (heightM / factor).toFixed(2);
    };

    const updateMaskFromInputs = () => {
        const widthVal = document.getElementById('mask-width')?.value;
        const heightVal = document.getElementById('mask-height')?.value;
        if (!widthVal || !heightVal || !window.drawDefaultMask) return;
        
        const isImperial = (document.getElementById('setting-units')?.value === 'imperial');
        const widthMeters = parseFloat(widthVal) * (isImperial ? 1609.34 : 1000);
        const heightMeters = parseFloat(heightVal) * (isImperial ? 1609.34 : 1000);
        
        let oldCenterLat = null;
        let oldCenterLng = null;
        if (typeof window.getMaskCenter === 'function') {
            const c = window.getMaskCenter();
            if (c) {
                oldCenterLat = c.lat;
                oldCenterLng = c.lng;
            }
        }
        
        if (window.clearMask) window.clearMask();
        window.drawDefaultMask(widthMeters, heightMeters, undefined, oldCenterLat, oldCenterLng);
        const toggle = document.getElementById('btn-draw-mask-toggle');
        if (toggle) toggle.checked = true;
    };

    ['mask-width', 'mask-height'].forEach(id => {
        document.getElementById(id)?.addEventListener('change', updateMaskFromInputs);
    });


    // ── Bearing Log ───────────────────────────────────────────────
    let _logEntries = [];
    let _logCounter = 0;
    let _loggedIds  = new Set();

    function _number(value) {
        const parsed = Number(value);
        return Number.isFinite(parsed) ? parsed : null;
    }

    function _qualityScore(obs) {
        const explicit = _number(obs?.quality_score);
        if (explicit !== null) return Math.max(0, Math.min(1, explicit));
        const legacy = _number(obs?.confidence);
        if (legacy !== null) return Math.max(0, Math.min(1, legacy));
        return 1.0;
    }

    function _qualityText(obs) {
        const papr = _number(obs?.doa_papr_db);
        if (papr !== null) return `${papr.toFixed(2)} dB PAPR`;
        const score = _number(obs?.quality_score);
        if (obs?.quality_kind === 'legacy_confidence' || obs?.confidence !== undefined) {
            const legacy = _number(obs?.confidence ?? score);
            if (legacy !== null) return `${(legacy * 100).toFixed(0)}% legacy`;
        }
        if (score !== null) return `${(score * 100).toFixed(0)}% score`;
        return '—';
    }

    function _passesQuality(obs) {
        if (obs?.usable_for_triangulation === false) return false;
        const threshold = Math.max(0, _number(elMinConf?.value) ?? 0.3);
        const papr = _number(obs?.doa_papr_db);
        if (papr !== null) return papr >= threshold;

        const legacy = _number(obs?.confidence ?? obs?.quality_score);
        if (legacy !== null) return legacy >= Math.min(threshold, 1.0);
        return true;
    }

    function _hasUpstreamUsabilityDecision(obs) {
        return Boolean(
            obs?.source_schema?.startsWith('fusion') &&
            typeof obs.usable_for_triangulation === 'boolean'
        );
    }

    document.getElementById('log-clear-btn')?.addEventListener('click', () => {
        _logEntries = [];
        _logCounter = 0;
        _loggedIds.clear();
        const tbody = document.getElementById('log-body');
        tbody.innerHTML = '<tr class="log-placeholder"><td colspan="6">Log cleared.</td></tr>';
        document.getElementById('log-count').textContent = '0 entries';
    });

    document.getElementById('log-export-btn')?.addEventListener('click', () => {
        const blob = new Blob([JSON.stringify(_logEntries, null, 2)], { type: 'application/json' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = `kraken_bearing_log_${Date.now()}.json`;
        a.click();
    });

    function appendNewObservations(data) {
        const observations = data?.observation_history ?? [];
        const tbody = document.getElementById('log-body');
        const placeholder = tbody.querySelector('.log-placeholder');
        const freqMHz = data?.frequency_hz ? (data.frequency_hz / 1e6).toFixed(4) : '—';
        let addedAny = false;

        observations.forEach(obs => {
            if (_loggedIds.has(obs.id)) return;
            _loggedIds.add(obs.id);
            addedAny = true;
            _logCounter++;

            const score = _qualityScore(obs);
            const qualityClass = score >= 0.8 ? 'conf-high' : score >= 0.5 ? 'conf-med' : 'conf-low';
            const quality = obs.usable_for_triangulation === false
                ? `REJECTED · ${_qualityText(obs)}`
                : _qualityText(obs);
            // Use server-assigned system-clock time (received_at)
            const time = obs.received_at
                ? new Date(obs.received_at).toLocaleTimeString()
                : new Date().toLocaleTimeString();

            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${_logCounter}</td>
                <td>${time}</td>
                <td>${obs.label || obs.id}</td>
                <td>${obs.bearing_deg.toFixed(1)}&deg;</td>
                <td class="${qualityClass}">${quality}</td>
                <td>${freqMHz}</td>
            `;
            if (placeholder) placeholder.remove();
            tbody.appendChild(row);
            _logEntries.push({
                n: _logCounter, time, id: obs.id,
                bearing_deg: obs.bearing_deg,
                confidence: obs.confidence,
                quality_score: obs.quality_score,
                quality_kind: obs.quality_kind,
                doa_papr_db: obs.doa_papr_db,
                usable_for_triangulation: obs.usable_for_triangulation,
                lat: obs.lat, lon: obs.lon, freq_hz: data.frequency_hz,
            });
        });

        if (addedAny) {
            document.getElementById('log-count').textContent = `${_logCounter} entries`;
        }
    }

    // ── Sidebar Result Panel ───────────────────────────────────────
    function updateResultPanel(data, result, displayHistory) {
        const set = (id, val) => {
            const el = document.getElementById(id);
            if (el) el.textContent = val ?? '—';
        };

        const current = data?.current_observation;

        if (result) {
            set('res-lat',      result.lat.toFixed(6) + '\u00b0');
            set('res-lon',      result.lon.toFixed(6) + '\u00b0');
            
            const isEmp = document.getElementById('setting-units')?.value === 'imperial';
            if (isEmp) {
                const ft = result.residual_m * 3.28084;
                if (ft >= 5280) set('res-error', (ft / 5280).toFixed(2) + ' mi');
                else set('res-error', ft.toFixed(1) + ' ft');
            } else {
                if (result.residual_m >= 1000) set('res-error', (result.residual_m / 1000).toFixed(2) + ' km');
                else set('res-error', result.residual_m.toFixed(1) + ' m');
            }
            
            set('res-stations', `${result.stationsUsed} / ${displayHistory.length}`);
        } else {
            ['res-lat','res-lon','res-error','res-stations'].forEach(id => set(id, '—'));
        }

        const freqMHz = data?.frequency_hz ? (data.frequency_hz / 1e6).toFixed(4) + ' MHz' : '—';
        set('res-freq', freqMHz);
        set('res-doa',  data?.doa_method ?? '—');

        // LAST RECEIVED = laptop system clock when this poll was processed
        set('res-timestamp', new Date().toLocaleTimeString());

        // Observation list
        const stList = document.getElementById('station-list');
        if (stList && displayHistory.length > 0) {
            const isCurrent = (obs) => current && obs.id === current.id;
            // Use reverse to show newest observations at the top of the list for better UX
            const renderedHtml = [...displayHistory].reverse().map(obs => {
                const cur = isCurrent(obs);
                return `<div class="station-item">
                    <div class="station-dot" style="background:${cur ? '#00e87a' : '#4f8ef7'}"></div>
                    <span class="station-name" style="${cur ? 'color:#00e87a;font-weight:600' : ''}">
                        ${cur ? '\u25b6 ' : ''}${obs.label || obs.id}
                    </span>
                    <span class="station-bearing">${obs.bearing_deg.toFixed(1)}&deg;</span>
                </div>`;
            }).join('');
            stList.innerHTML = renderedHtml;
        } else if (stList) {
            stList.innerHTML = '<p class="placeholder-text">Waiting for observations...</p>';
        }
    }

    // ── Core Data Pipeline ─────────────────────────────────────────
    let _lastData = null;

    const modeBadge = document.getElementById('mode-badge');

    function _processData(data) {
        _lastData = data;

        // Status Badge
        if (data?.source === 'udp_stream') {
            if (modeBadge) {
                modeBadge.textContent = 'LIVE TELEMETRY';
                modeBadge.className = 'badge badge-live';
                modeBadge.style.background = '#00b894';
                modeBadge.style.color = '#fff';
                modeBadge.style.border = 'none';
            }
        } else if (data?.source === 'replay') {
            if (modeBadge) {
                modeBadge.textContent = 'REPLAY';
                modeBadge.className = 'badge badge-mock';
                modeBadge.style.background = 'rgba(255,184,77,0.15)';
                modeBadge.style.color = '#ffb84d';
                modeBadge.style.border = '1px solid rgba(255,184,77,0.3)';
            }
        } else if (data?.source === 'waiting') {
            if (modeBadge) {
                modeBadge.textContent = 'WAITING';
                modeBadge.className = 'badge badge-waiting';
                modeBadge.style.background = '';
                modeBadge.style.color = '';
                modeBadge.style.border = '';
            }
        }

        // Sync replay transport UI if in replay mode
        if (data?.playback) {
            _syncReplayTransport(data.playback);
        }

        let history = data?.observation_history ?? [];
        // 1. Initial manual pruning (Quality, Attitude, Temporal)
        let filtered = history.filter(_passesQuality);
        
        if (elFilterAttitude?.checked) {
            filtered = filtered.filter(obs => {
                if (_hasUpstreamUsabilityDecision(obs)) return true;
                const roll = Math.abs(obs.roll_deg || 0);
                const pitch = Math.abs(obs.pitch_deg || 0);
                return roll <= 15.0 && pitch <= 15.0;
            });
        }
        
        if (elFilterTemporal?.checked) {
            filtered = filtered.slice(-30);
        }

        // 2. Fetch spatial AABB mask if active
        let aabb = null;
        if (typeof window.getMaskBounds === 'function') {
            aabb = window.getMaskBounds();
        }
        
        let polyBounds = null;
        if (typeof window.getPolyBounds === 'function') {
            polyBounds = window.getPolyBounds();
        }
        
        const config = {
            filterSpatial: document.getElementById('btn-draw-mask-toggle')?.checked ?? false,
            filterPoly: document.getElementById('btn-draw-poly-toggle')?.checked ?? false,
            filterAngular: elFilterAngular?.checked ?? true,
            aabb: aabb,
            polyBounds: polyBounds
        };

        const algo   = elAlgo?.value || 'ls_aoa';
        let result = filtered.length >= 2 ? Triangulation.solve(filtered, algo, config) : null;
        
        // 3. Identify which observations were actually used in the final math (for map dots coloring)
        let validObs = Triangulation.filterStations(filtered, config);
        
        // Strict boundary pruning: if resulting triangulation coordinate forms outside mask, drop it!
        if (result && aabb) {
            if (result.lat < aabb.minLat || result.lat > aabb.maxLat || 
                result.lon < aabb.minLon || result.lon > aabb.maxLon) {
                result = null;
            }
        }

        MapView.update(data, result, validObs);

        if (result) {
            if (algo === 'bayesian' && result.heatGrid) {
                MapView.setHeatGrid(result.heatGrid);
            } else {
                const avgQuality = validObs.reduce((s, o) => s + _qualityScore(o), 0) / validObs.length;
                const intensity = Math.min(1.0, (validObs.length / 6) * avgQuality);
                MapView.addHeatPoint(result.lat, result.lon, intensity);
            }
            const el = document.getElementById('hm-point-count');
            if (el) el.textContent = MapView.getHeatPointCount();

            // ── Estimation Mask & Proximity Flagging ──
            if (typeof window.getSearchAreaSummary === 'function') {
                const sa = window.getSearchAreaSummary();
                if (sa && sa.isActive && sa.center && sa.isInside) {
                    const isInside = sa.isInside(result.lat, result.lon);
                    if (isInside) {
                        _addEstimationHit(result);
                    }
                }
            }
        }

        updateResultPanel(data, result, validObs);
        appendNewObservations(data);
    }

    DataFeed.onData(_processData);

    // ── GCS Bridge ─────────────────────────────────────────────────
    async function _transmitToGCS(lat, lon, spread, count) {
        const url = window.location.origin + '/api/transmit';
        const res = await fetch(url, {
            method: 'POST',
            body: JSON.stringify({ lat, lon, spread_m: spread, count })
        });
        if (!res.ok) {
            throw new Error(`Server returned ${res.status}`);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // ── PLOT COORDINATE WIRING ────────────────────────────────────
    // ═══════════════════════════════════════════════════════════════

    let _lastResult = null;       // Track latest triangulation for distance calc
    let _referenceMarkerId = null; // Which marker is the user's reference

    // Override the updateResultPanel to also capture the result for distance calc
    const _origUpdateResultPanel = updateResultPanel;
    updateResultPanel = function(data, result, validObs) {
        _lastResult = result;
        _origUpdateResultPanel(data, result, validObs);
        _refreshGTList();
        _updateRefDistance();
    };

    document.getElementById('btn-gt-submit')?.addEventListener('click', () => {
        const latEl = document.getElementById('gt-lat');
        const lonEl = document.getElementById('gt-lon');
        const labelEl = document.getElementById('gt-label');
        
        const lat = parseFloat(latEl?.value);
        const lon = parseFloat(lonEl?.value);
        const label = labelEl?.value?.trim() || '';
        
        if (isNaN(lat) || isNaN(lon) || lat < -90 || lat > 90 || lon < -180 || lon > 180) {
            alert('Please enter valid latitude (-90 to 90) and longitude (-180 to 180).');
            return;
        }
        
        const obj = MapView.addGroundTruth(lat, lon, label);
        
        // Auto-set as reference if it's the first marker
        if (obj && MapView.getGroundTruthMarkers().length === 1) {
            _referenceMarkerId = obj.id;
        }
        
        _refreshGTList();
        _updateRefDistance();
        
        // Clear inputs for next entry
        if (latEl) latEl.value = '';
        if (lonEl) lonEl.value = '';
        if (labelEl) labelEl.value = '';
    });

    // Also allow Enter key in lat/lon fields to submit
    ['gt-lat', 'gt-lon', 'gt-label'].forEach(id => {
        document.getElementById(id)?.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                document.getElementById('btn-gt-submit')?.click();
            }
        });
    });

    function _refreshGTList() {
        const listEl = document.getElementById('gt-list');
        if (!listEl) return;
        
        const markers = MapView.getGroundTruthMarkers();
        if (markers.length === 0) {
            listEl.innerHTML = '';
            _referenceMarkerId = null;
            _updateRefDistance();
            return;
        }

        // If the current reference was removed, clear it
        if (_referenceMarkerId && !markers.find(m => m.id === _referenceMarkerId)) {
            _referenceMarkerId = null;
        }

        const unit = document.getElementById('setting-units')?.value || 'metric';
        
        listEl.innerHTML = markers.map(gt => {
            const isRef = gt.id === _referenceMarkerId;
            
            // Reference toggle button
            const refBtn = isRef
                ? `<span class="gt-ref-badge" title="Active reference">REF</span>`
                : `<button class="gt-ref-btn" onclick="window._setGTRef(${gt.id})" title="Set as reference">Set Ref</button>`;

            // Distance from triangulation result
            let distText = '';
            if (_lastResult && _lastResult.lat && _lastResult.lon) {
                const distM = _haversineM(_lastResult.lat, _lastResult.lon, gt.lat, gt.lon);
                if (unit === 'imperial') {
                    const distFt = distM * 3.28084;
                    distText = distFt > 5280 ? `${(distFt/5280).toFixed(2)} mi` : `${distFt.toFixed(0)} ft`;
                } else {
                    distText = distM > 1000 ? `${(distM/1000).toFixed(2)} km` : `${distM.toFixed(0)} m`;
                }
                distText = `Δ ${distText}`;
            }

            return `<div class="gt-item ${isRef ? 'gt-item-active' : ''}">
                <div class="gt-item-info">
                    <span class="gt-item-label">${gt.label}</span>
                    <span class="gt-item-coords">${gt.lat.toFixed(6)}, ${gt.lon.toFixed(6)}</span>
                </div>
                ${distText ? `<span class="gt-item-dist">${distText}</span>` : ''}
                ${refBtn}
                <button class="gt-item-remove" onclick="window._removeGT(${gt.id})" title="Remove">✕</button>
            </div>`;
        }).join('');
    }

    // Update the Δ REF. DISTANCE row in the Triangulation Result card
    function _updateRefDistance() {
        const row = document.getElementById('res-ref-row');
        const distEl = document.getElementById('res-ref-dist');
        if (!row || !distEl) return;

        if (!_referenceMarkerId) {
            row.style.display = 'none';
            distEl.textContent = '—';
            return;
        }

        const ref = MapView.getGroundTruthMarkers().find(m => m.id === _referenceMarkerId);
        if (!ref || !_lastResult || !_lastResult.lat || !_lastResult.lon) {
            row.style.display = 'flex';
            distEl.textContent = '—';
            return;
        }

        const distM = _haversineM(_lastResult.lat, _lastResult.lon, ref.lat, ref.lon);
        const unit = document.getElementById('setting-units')?.value || 'metric';
        let text;
        if (unit === 'imperial') {
            const ft = distM * 3.28084;
            text = ft >= 5280 ? `${(ft/5280).toFixed(2)} mi` : `${ft.toFixed(1)} ft`;
        } else {
            text = distM >= 1000 ? `${(distM/1000).toFixed(2)} km` : `${distM.toFixed(1)} m`;
        }

        row.style.display = 'flex';
        distEl.textContent = `${text}  (${ref.label})`;
    }

    // Haversine helper for distance calculation
    function _haversineM(lat1, lon1, lat2, lon2) {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat/2)**2 + Math.cos(lat1*Math.PI/180) * Math.cos(lat2*Math.PI/180) * Math.sin(dLon/2)**2;
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    }

    // Global handlers
    window._removeGT = function(id) {
        if (_referenceMarkerId === id) _referenceMarkerId = null;
        MapView.removeGroundTruth(id);
        _refreshGTList();
        _updateRefDistance();
    };

    window._setGTRef = function(id) {
        _referenceMarkerId = id;
        _refreshGTList();
        _updateRefDistance();
    };

    // ═══════════════════════════════════════════════════════════════
    // ── REPLAY TAB WIRING ─────────────────────────────────────────
    // ═══════════════════════════════════════════════════════════════

    let _replayActive = false;

    // ── File List ──
    async function _fetchReplayFiles() {
        const list = document.getElementById('replay-file-list');
        if (!list) return;
        list.innerHTML = '<div class="replay-file-placeholder">Loading files...</div>';
        try {
            const resp = await fetch('/api/replay/files');
            const data = await resp.json();
            if (!data.files || data.files.length === 0) {
                list.innerHTML = '<div class="replay-file-placeholder">No replay files found.</div>';
                return;
            }
            list.innerHTML = '';
            data.files.forEach(file => {
                const item = document.createElement('div');
                item.className = 'replay-file-item';
                item.innerHTML = `
                    <span class="replay-file-type type-${file.type}">${file.type.replace(/_/g, ' ')}</span>
                    <span class="replay-file-name" title="${file.path}">${file.name}</span>
                    <span class="replay-file-meta">${file.size_kb} KB</span>
                    <button class="replay-file-load-btn">LOAD</button>
                `;
                item.querySelector('.replay-file-load-btn').addEventListener('click', (e) => {
                    e.stopPropagation();
                    _loadReplayFile(file.path, file.name);
                });
                list.appendChild(item);
            });
        } catch (e) {
            list.innerHTML = '<div class="replay-file-placeholder">Error loading file list.</div>';
            console.error('[Replay] File list error:', e);
        }
    }

    // ── Load a file for replay ──
    async function _loadReplayFile(path, name) {
        try {
            const resp = await fetch('/api/replay/load', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ path })
            });
            const data = await resp.json();
            if (data.error) {
                alert('Load failed: ' + data.error);
                return;
            }
            _replayActive = true;
            _updateReplayUI(true, data.filename, data.total);
            // Sync settings toggle
            const toggle = document.getElementById('setting-replay-mode');
            if (toggle) toggle.checked = true;
            console.log(`[Replay] Loaded ${data.total} waypoints from ${data.filename}`);
        } catch (e) {
            alert('Failed to load replay: ' + e.message);
        }
    }

    function _updateReplayUI(active, filename, total) {
        const badge = document.getElementById('replay-status-badge');
        const transport = document.getElementById('replay-transport-section');
        const nameEl = document.getElementById('replay-loaded-name');
        const countEl = document.getElementById('replay-loaded-count');

        if (active) {
            if (badge) { badge.className = 'replay-badge replay-badge-active'; badge.textContent = 'ACTIVE'; }
            if (transport) { transport.style.opacity = '1'; transport.style.pointerEvents = 'auto'; }
            if (nameEl) nameEl.textContent = filename || 'Loaded';
            if (countEl) countEl.textContent = `${total || 0} waypoints`;
        } else {
            if (badge) { badge.className = 'replay-badge replay-badge-inactive'; badge.textContent = 'INACTIVE'; }
            if (transport) { transport.style.opacity = '0.4'; transport.style.pointerEvents = 'none'; }
            if (nameEl) nameEl.textContent = 'No file loaded';
            if (countEl) countEl.textContent = '0 waypoints';
        }
    }

    // ── Transport Controls ──
    async function _playbackAction(action, value) {
        try {
            const body = { action };
            if (value !== undefined) body.value = value;
            await fetch('/api/playback', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body)
            });
        } catch (e) {
            console.error('[Replay] Playback action error:', e);
        }
    }

    function _syncReplayTransport(pb) {
        if (!pb) return;
        const posEl = document.getElementById('rb-pos');
        const scrubber = document.getElementById('rb-scrubber');
        const playBtn = document.getElementById('rb-play-pause');

        if (posEl) posEl.textContent = `${pb.index} / ${pb.total}`;
        if (scrubber) {
            scrubber.max = pb.total || 1;
            scrubber.value = pb.index;
        }
        if (playBtn) {
            if (pb.paused) {
                playBtn.textContent = '\u25b6';
                playBtn.classList.add('paused');
            } else {
                playBtn.textContent = '\u23f8';
                playBtn.classList.remove('paused');
            }
        }
    }

    // Wire up transport buttons
    document.getElementById('rb-play-pause')?.addEventListener('click', () => {
        const btn = document.getElementById('rb-play-pause');
        const isPaused = btn?.classList.contains('paused');
        _playbackAction(isPaused ? 'play' : 'pause');
    });
    document.getElementById('rb-forward')?.addEventListener('click', () => _playbackAction('forward'));
    document.getElementById('rb-rewind')?.addEventListener('click', () => _playbackAction('rewind'));
    document.getElementById('rb-reset')?.addEventListener('click', () => _playbackAction('reset'));
    document.getElementById('rb-end')?.addEventListener('click', () => {
        const scrubber = document.getElementById('rb-scrubber');
        if (scrubber) _playbackAction('seek', parseInt(scrubber.max));
    });
    document.getElementById('rb-scrubber')?.addEventListener('input', (e) => {
        _playbackAction('seek', parseInt(e.target.value));
    });
    document.getElementById('rb-speed')?.addEventListener('change', (e) => {
        _playbackAction('set_speed', parseFloat(e.target.value));
    });

    // ── Stop Replay (back to live) ──
    document.getElementById('btn-replay-stop')?.addEventListener('click', async () => {
        try {
            await fetch('/api/replay/stop', { method: 'POST' });
            _replayActive = false;
            _updateReplayUI(false);
            const toggle = document.getElementById('setting-replay-mode');
            if (toggle) toggle.checked = false;
            console.log('[Replay] Stopped, back to live mode.');
        } catch (e) {
            console.error('[Replay] Stop error:', e);
        }
    });

    // ── Refresh button ──
    document.getElementById('btn-replay-refresh')?.addEventListener('click', _fetchReplayFiles);

    // ── Upload ──
    const uploadZone = document.getElementById('replay-upload-zone');
    const fileInput = document.getElementById('replay-file-input');
    const uploadStatus = document.getElementById('replay-upload-status');

    uploadZone?.addEventListener('click', () => fileInput?.click());
    uploadZone?.addEventListener('dragover', (e) => { e.preventDefault(); uploadZone.classList.add('drag-over'); });
    uploadZone?.addEventListener('dragleave', () => uploadZone.classList.remove('drag-over'));
    uploadZone?.addEventListener('drop', (e) => {
        e.preventDefault();
        uploadZone.classList.remove('drag-over');
        if (e.dataTransfer.files.length > 0) _uploadFile(e.dataTransfer.files[0]);
    });
    fileInput?.addEventListener('change', () => {
        if (fileInput.files.length > 0) _uploadFile(fileInput.files[0]);
    });

    async function _uploadFile(file) {
        if (uploadStatus) { uploadStatus.style.display = 'block'; uploadStatus.textContent = `Uploading ${file.name}...`; uploadStatus.style.color = 'var(--text-secondary)'; }
        const form = new FormData();
        form.append('file', file);
        try {
            const resp = await fetch('/api/replay/upload', { method: 'POST', body: form });
            const data = await resp.json();
            if (data.error) {
                if (uploadStatus) { uploadStatus.textContent = `Error: ${data.error}`; uploadStatus.style.color = 'var(--accent-red)'; }
                return;
            }
            if (uploadStatus) { uploadStatus.textContent = `\u2713 Uploaded ${data.name} (${data.size_kb} KB)`; uploadStatus.style.color = 'var(--accent-green)'; }
            _fetchReplayFiles(); // refresh list
        } catch (e) {
            if (uploadStatus) { uploadStatus.textContent = `Upload failed: ${e.message}`; uploadStatus.style.color = 'var(--accent-red)'; }
        }
    }

    // ── Settings Toggle ──
    const elReplayMode = document.getElementById('setting-replay-mode');
    elReplayMode?.addEventListener('change', async () => {
        if (elReplayMode.checked) {
            // Just switch to replay tab — user needs to pick a file
            switchTab('replay');
        } else {
            // Stop replay, back to live
            try {
                await fetch('/api/replay/stop', { method: 'POST' });
                _replayActive = false;
                _updateReplayUI(false);
            } catch (e) { console.error(e); }
        }
    });

    // ── Initial load of file list when replay tab is first shown ──
    let _replayFilesLoaded = false;
    const origSwitchTab = switchTab;
    switchTab = function(targetId) {
        origSwitchTab(targetId);
        if (targetId === 'replay' && !_replayFilesLoaded) {
            _replayFilesLoaded = true;
            _fetchReplayFiles();
        }
    };

    // ── Eager Map Init ─────────────────────────────────────────────
    requestAnimationFrame(() => {
        MapView.init('map');
        
        // Eagerly apply loaded settings cleanly
        _allSettings.forEach(el => {
            if (el && el !== elPollInterval) {
                el.dispatchEvent(new Event('change'));
            }
        });
        
        console.log('[App] Maps initialised. Starting data feed...');
        DataFeed.start();
    });

})();

