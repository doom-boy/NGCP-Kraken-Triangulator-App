/**
 * triangulation.js — LS-AoA Triangulation Math
 * =============================================
 * Implements Least Squares Angle-of-Arrival (LS-AoA) intersection.
 *
 * Given N receiver stations, each with a known (lat, lon) and a bearing
 * estimate (degrees from north, clockwise), this module computes the
 * best-estimate source position by minimizing the perpendicular distance
 * from each bearing line to the unknown target point.
 *
 * Reference algorithm:
 *   Shao, X. et al. "Source Localization from Bearings-Only Measurements."
 *   Adapted for WGS-84 lat/lon coordinates using flat-earth approximation
 *   (valid for short ranges < ~50 km).
 */

const Triangulation = (() => {

    const DEG2RAD = Math.PI / 180;
    const RAD2DEG = 180 / Math.PI;
    const EARTH_R  = 6371000; // metres

    function stationWeight(station) {
        const value = Number(station.quality_score ?? station.confidence);
        return Number.isFinite(value) && value > 0 ? value : 1.0;
    }

    /**
     * Convert a (lat, lon) to local ENU (East, North) metres
     * relative to a reference origin.
     */
    function latLonToENU(lat, lon, refLat, refLon) {
        const dLat = (lat - refLat) * DEG2RAD;
        const dLon = (lon - refLon) * DEG2RAD;
        const N = dLat * EARTH_R;
        const E = dLon * EARTH_R * Math.cos(refLat * DEG2RAD);
        return { E, N };
    }

    /**
     * Convert local ENU (East, North) back to (lat, lon).
     */
    function enuToLatLon(E, N, refLat, refLon) {
        const lat = refLat + (N / EARTH_R) * RAD2DEG;
        const lon = refLon + (E / (EARTH_R * Math.cos(refLat * DEG2RAD))) * RAD2DEG;
        return { lat, lon };
    }

    /**
     * Core LS-AoA solver.
     *
     * stations: Array of { lat, lon, bearing_deg, quality_score }
     *
     * Each bearing line can be written as:
     *   sin(theta) * (y - y_i) - cos(theta) * (x - x_i) = 0
     * => a_i * x + b_i * y = c_i
     *
     * where theta = bearing in ENU (note: bearing is from N, clockwise)
     *   theta_enu = 90 - bearing_deg  (converts to math angle CCW from east)
     *
     * Weighted least squares: (A^T W A) x = (A^T W b)
     *
     * Returns: { lat, lon, residual, stationsUsed } or null if < 2 stations.
     */
    function lsAoA(stations) {
        if (!stations || stations.length < 2) return null;

        // Reference origin: centroid of stations
        const refLat = stations.reduce((s, st) => s + st.lat, 0) / stations.length;
        const refLon = stations.reduce((s, st) => s + st.lon, 0) / stations.length;

        // Build weighted normal equations
        let AtWA = [[0,0],[0,0]];
        let AtWb = [0, 0];

        for (const st of stations) {
            const w = stationWeight(st);
            const { E, N } = latLonToENU(st.lat, st.lon, refLat, refLon);

            // Convert bearing (CW from N) to ENU math angle (CCW from E)
            const theta = (90 - st.bearing_deg) * DEG2RAD;
            const sinT = Math.sin(theta);
            const cosT = Math.cos(theta);

            // Row: [sinT, -cosT] . [x, y]^T = sinT * E - cosT * N
            const a = sinT;
            const b = -cosT;
            const c = sinT * E - cosT * N;

            AtWA[0][0] += w * a * a;
            AtWA[0][1] += w * a * b;
            AtWA[1][0] += w * b * a;
            AtWA[1][1] += w * b * b;
            AtWb[0]    += w * a * c;
            AtWb[1]    += w * b * c;
        }

        // Solve 2x2 system via Cramer's rule
        const det = AtWA[0][0] * AtWA[1][1] - AtWA[0][1] * AtWA[1][0];
        if (Math.abs(det) < 1e-10) {
            console.warn('[Triangulation] Singular matrix — bearings may be parallel.');
            return null;
        }

        const x = (AtWb[0] * AtWA[1][1] - AtWb[1] * AtWA[0][1]) / det; // East
        const y = (AtWA[0][0] * AtWb[1] - AtWA[1][0] * AtWb[0]) / det; // North

        // Compute mean per-station residual (metres from bearing line)
        let totalResidual = 0;
        for (const st of stations) {
            const { E, N } = latLonToENU(st.lat, st.lon, refLat, refLon);
            const theta = (90 - st.bearing_deg) * DEG2RAD;
            const sinT = Math.sin(theta);
            const cosT = Math.cos(theta);
            const dist = Math.abs(sinT * (x - E) - cosT * (y - N));
            totalResidual += dist;
        }
        const residual = totalResidual / stations.length;

        const result = enuToLatLon(x, y, refLat, refLon);
        return {
            lat: result.lat,
            lon: result.lon,
            residual_m: residual,
            stationsUsed: stations.length,
        };
    }

    /**
     * Simple midpoint method for exactly 2 stations.
     * Finds the closest approach point between two bearing lines.
     * Only used when algo === 'midpoint'.
     */
    function midpoint(stations) {
        if (!stations || stations.length < 2) return null;
        const [a, b] = stations;
        const refLat = (a.lat + b.lat) / 2;
        const refLon = (a.lon + b.lon) / 2;
        const pA = latLonToENU(a.lat, a.lon, refLat, refLon);
        const pB = latLonToENU(b.lat, b.lon, refLat, refLon);
        const tA = (90 - a.bearing_deg) * DEG2RAD;
        const tB = (90 - b.bearing_deg) * DEG2RAD;
        const dA = { x: Math.cos(tA), y: Math.sin(tA) };
        const dB = { x: Math.cos(tB), y: Math.sin(tB) };
        const dx = pB.E - pA.E;
        const dy = pB.N - pA.N;
        const cross = dA.x * dB.y - dA.y * dB.x;
        if (Math.abs(cross) < 1e-10) return null;
        const t = (dx * dB.y - dy * dB.x) / cross;
        const ix = pA.E + t * dA.x;
        const iy = pA.N + t * dA.y;
        const res = enuToLatLon(ix, iy, refLat, refLon);
        return { lat: res.lat, lon: res.lon, residual_m: 0, stationsUsed: 2 };
    }

    /**
     * Solution 2: Bayesian Spatial Grid Triangulation
     * Creates a spatial grid and evaluates the probability density 
     * mathematically eliminating multipath interference.
     */
    function bayesianGrid(stations) {
        if (!stations || stations.length < 2) return null;

        // Use LS-AoA as the grid center to focus the compute power
        const lsResult = lsAoA(stations);
        let refLat = lsResult ? lsResult.lat : stations.reduce((s, st) => s + st.lat, 0) / stations.length;
        let refLon = lsResult ? lsResult.lon : stations.reduce((s, st) => s + st.lon, 0) / stations.length;

        const GRID_SIZE_M = 15000; // 15km x 15km grid
        const STEP_M = 100; // 100m cell resolution
        const HALF = GRID_SIZE_M / 2;
        const SIGMA = 5.0; // 5 degrees standard deviation for bearing blur

        let maxScore = -Infinity;
        let bestE = 0, bestN = 0;
        let heatPoints = [];

        // Pre-convert stations
        const enuStations = stations.map(st => ({
            ...st,
            enu: latLonToENU(st.lat, st.lon, refLat, refLon)
        }));

        // Traverse the spatial grid
        for (let y = -HALF; y <= HALF; y += STEP_M) {
            for (let x = -HALF; x <= HALF; x += STEP_M) {
                let logScore = 0;
                for (const st of enuStations) {
                    const dx = x - st.enu.E;
                    const dy = y - st.enu.N;
                    
                    // Angle from station to cell
                    const mathAngle = Math.atan2(dy, dx) * RAD2DEG;
                    let targetBearing = 90 - mathAngle;
                    if (targetBearing < 0) targetBearing += 360;

                    let diff = Math.abs(targetBearing - st.bearing_deg) % 360;
                    if (diff > 180) diff = 360 - diff;

                    const weight = stationWeight(st);
                    // Log-probability of Gaussian
                    logScore += weight * (-(diff * diff) / (2 * SIGMA * SIGMA));
                }

                if (logScore > maxScore) {
                    maxScore = logScore;
                    bestE = x;
                    bestN = y;
                }
                heatPoints.push({ x, y, score: logScore });
            }
        }

        // Normalize scores to probabilities for the Leaflet visual map
        const visualGrid = heatPoints
            .map(pt => {
                pt.prob = Math.exp(pt.score - maxScore); // highest will be 1.0
                return pt;
            })
            .filter(pt => pt.prob > 0.1) // Only render the 'hot' cloud to save rendering CPU
            .map(pt => {
                const ll = enuToLatLon(pt.x, pt.y, refLat, refLon);
                return [ll.lat, ll.lon, pt.prob];
            });

        const resultLatLon = enuToLatLon(bestE, bestN, refLat, refLon);

        // Compute geometric residual variance
        let totalResidual = 0;
        for (const st of enuStations) {
            const theta = (90 - st.bearing_deg) * DEG2RAD;
            const dist = Math.abs(Math.sin(theta) * (bestE - st.enu.E) - Math.cos(theta) * (bestN - st.enu.N));
            totalResidual += dist;
        }

        return {
            lat: resultLatLon.lat,
            lon: resultLatLon.lon,
            residual_m: totalResidual / stations.length,
            stationsUsed: stations.length,
            heatGrid: visualGrid
        };
    }

    /**
     * Pairwise intersection helper
     */
    function getIntersection(a, b, refLat, refLon) {
        const pA = latLonToENU(a.lat, a.lon, refLat, refLon);
        const pB = latLonToENU(b.lat, b.lon, refLat, refLon);
        const tA = (90 - a.bearing_deg) * DEG2RAD;
        const tB = (90 - b.bearing_deg) * DEG2RAD;
        const dA = { x: Math.cos(tA), y: Math.sin(tA) };
        const dB = { x: Math.cos(tB), y: Math.sin(tB) };
        const dx = pB.E - pA.E;
        const dy = pB.N - pA.N;
        const cross = dA.x * dB.y - dA.y * dB.x;
        if (Math.abs(cross) < 1e-5) return null; // parallel
        const t = (dx * dB.y - dy * dB.x) / cross;
        const ix = pA.E + t * dA.x;
        const iy = pA.N + t * dA.y;
        // Verify intersection is forward along both bearing vectors
        const dpA = (ix - pA.E)*dA.x + (iy - pA.N)*dA.y;
        const dpB = (ix - pB.E)*dB.x + (iy - pB.N)*dB.y;
        if (dpA < 0 || dpB < 0) return null; // intersection is behind them
        return { E: ix, N: iy };
    }

    /**
     * 2D Ray-AABB Intersection (Slab Method)
     * Checks if a ray starting at 'origin' with direction 'dir' intersects
     * the box defined by 'min' and 'max'.
     */
    function rayAABBIntersect(origin, dir, min, max) {
        let tmin = -Infinity, tmax = Infinity;

        // X-axis
        if (dir.x !== 0) {
            let t1 = (min.x - origin.x) / dir.x;
            let t2 = (max.x - origin.x) / dir.x;
            tmin = Math.max(tmin, Math.min(t1, t2));
            tmax = Math.min(tmax, Math.max(t1, t2));
        } else if (origin.x < min.x || origin.x > max.x) {
            return false;
        }

        // Y-axis
        if (dir.y !== 0) {
            let t1 = (min.y - origin.y) / dir.y;
            let t2 = (max.y - origin.y) / dir.y;
            tmin = Math.max(tmin, Math.min(t1, t2));
            tmax = Math.min(tmax, Math.max(t1, t2));
        } else if (origin.y < min.y || origin.y > max.y) {
            return false;
        }

        return tmax >= Math.max(0, tmin);
    }

    /**
     * Standard Point-in-Polygon (Raycasting)
     */
    function pointInPolygon2D(pt, poly) {
        let inside = false;
        for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
            const xi = poly[i].E, yi = poly[i].N;
            const xj = poly[j].E, yj = poly[j].N;
            const intersect = ((yi > pt.y) !== (yj > pt.y)) && 
                              (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    /**
     * 2D Ray-Segment Intersection
     */
    function raySegmentIntersect2D(origin, dir, p1, p2) {
        // ray: O + t * D
        // seg: p1 + u * (p2 - p1)
        const dx = dir.x, dy = dir.y;
        const vx = p2.E - p1.E, vy = p2.N - p1.N;
        const wx = p1.E - origin.x, wy = p1.N - origin.y;
        
        // Cramer's rule determinant
        const det = vx * dy - vy * dx;
        if (Math.abs(det) < 1e-6) return false;
        
        // parameter 't' along the ray, 'u' along the segment bounds
        const t = (vx * wy - vy * wx) / det;
        const u = (dx * wy - dy * wx) / det;
        
        // t >= 0: intersection is strictly forward extending from the sensor
        // 0 <= u <= 1: intersection lands exactly between the two polygon vertices
        return (t >= 0 && u >= 0 && u <= 1);
    }

    /**
     * Apply geometric and numeric filters before doing LS-AoA
     */
    function filterStations(stations, config) {
        if (!stations || stations.length < 1) return stations;
        let filtered = stations.filter(st => st?.usable_for_triangulation !== false);

        // 1A. Spatial Box Geofencing (AABB Raycasting)
        // config.aabb = { minLat, maxLat, minLon, maxLon }
        if (config.filterSpatial && config.aabb) {
            const { minLat, maxLat, minLon, maxLon } = config.aabb;
            // Reference origin for local ENU conversion (center of box)
            const refLat = (minLat + maxLat) / 2;
            const refLon = (minLon + maxLon) / 2;
            
            const bMin = latLonToENU(minLat, minLon, refLat, refLon);
            const bMax = latLonToENU(maxLat, maxLon, refLat, refLon);
            
            filtered = filtered.filter(st => {
                const origin = latLonToENU(st.lat, st.lon, refLat, refLon);
                const theta = (90 - st.bearing_deg) * DEG2RAD;
                const dir = { x: Math.cos(theta), y: Math.sin(theta) };
                
                // Box bounds in ENU
                const boxMin = { x: Math.min(bMin.E, bMax.E), y: Math.min(bMin.N, bMax.N) };
                const boxMax = { x: Math.max(bMin.E, bMax.E), y: Math.max(bMin.N, bMax.N) };
                
                return rayAABBIntersect({ x: origin.E, y: origin.N }, dir, boxMin, boxMax);
            });
        }

        // 1B. Spatial Polygon Geofencing (N-sided Polygon Intersection)
        if (config.filterPoly && config.polyBounds && config.polyBounds.length > 2) {
            const refLat = config.polyBounds[0].lat;
            const refLon = config.polyBounds[0].lon;
            
            const polyENU = config.polyBounds.map(p => latLonToENU(p.lat, p.lon, refLat, refLon));
            
            filtered = filtered.filter(st => {
                const origin = latLonToENU(st.lat, st.lon, refLat, refLon);
                const theta = (90 - st.bearing_deg) * DEG2RAD;
                const dir = { x: Math.cos(theta), y: Math.sin(theta) };
                const oPt = { x: origin.E, y: origin.N };
                
                // If station is physically inside the polygon, it automatically intersects bounds.
                if (pointInPolygon2D(oPt, polyENU)) return true;
                
                // Otherwise, check if the ray crosses any polygon edge segment.
                for (let i = 0, j = polyENU.length - 1; i < polyENU.length; j = i++) {
                    if (raySegmentIntersect2D(oPt, dir, polyENU[j], polyENU[i])) {
                        return true;
                    }
                }
                return false;
            });
        }

        if (filtered.length < 2) return filtered;

        // 2. Angular Separation Gating: remove observations that are too co-linear
        if (config.filterAngular) {
            const minAngleSep = 2.0; // degrees
            const pruned = [];
            for (const st of filtered) {
                let tooClose = false;
                for (const other of pruned) {
                    let diff = Math.abs(st.bearing_deg - other.bearing_deg) % 360;
                    if (diff > 180) diff = 360 - diff;
                    if (diff < minAngleSep || Math.abs(180 - diff) < minAngleSep) {
                        tooClose = true;
                        break;
                    }
                }
                // Keep if distinct enough, or just allow it if we only have 1 so far
                if (!tooClose || pruned.length === 0) {
                    pruned.push(st);
                }
            }
            if (pruned.length >= 2) {
                filtered = pruned;
            }
        }

        // 2. Spatial Outlier Rejection (Median Intersection Clustering)
        if (config.filterSpatial && filtered.length > 2) {
            const refLat = filtered[0].lat;
            const refLon = filtered[0].lon;
            const intersections = [];

            // Compute all pairwise intersections
            for (let i = 0; i < filtered.length; i++) {
                for (let j = i + 1; j < filtered.length; j++) {
                    const ix = getIntersection(filtered[i], filtered[j], refLat, refLon);
                    if (ix) intersections.push(ix);
                }
            }

            if (intersections.length > 0) {
                // Find median intersection
                intersections.sort((a,b) => a.E - b.E);
                const medE = intersections[Math.floor(intersections.length / 2)].E;
                intersections.sort((a,b) => a.N - b.N);
                const medN = intersections[Math.floor(intersections.length / 2)].N;

                // For each station, compute distance from its ray to the median point
                const inliers = filtered.filter(st => {
                    const p = latLonToENU(st.lat, st.lon, refLat, refLon);
                    const theta = (90 - st.bearing_deg) * DEG2RAD;
                    const sinT = Math.sin(theta);
                    const cosT = Math.cos(theta);
                    const dist = Math.abs(sinT * (medE - p.E) - cosT * (medN - p.N));
                    return dist < 200; // max acceptable residual from median intersection (m)
                });
                
                if (inliers.length >= 2) {
                    filtered = inliers;
                }
            }
        }

        return filtered;
    }

    /**
     * Haversine formula for spherical distance (returns meters)
     */
    function distanceMeters(lat1, lon1, lat2, lon2) {
        const dLat = (lat2 - lat1) * DEG2RAD;
        const dLon = (lon2 - lon1) * DEG2RAD;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                  Math.cos(lat1 * DEG2RAD) * Math.cos(lat2 * DEG2RAD) *
                  Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return EARTH_R * c;
    }

    function distanceFeet(lat1, lon1, lat2, lon2) {
        return distanceMeters(lat1, lon1, lat2, lon2) * 3.28084;
    }

    /**
     * Public API
     */
    return {
        lsAoA,
        midpoint,
        bayesianGrid,
        filterStations,
        distanceMeters,
        distanceFeet,
        /**
         * Main entry: picks algorithm based on settings.
         * @param {Array}  stations  — filtered station objects
         * @param {string} algo      — 'ls_aoa' | 'midpoint' | 'bayesian'
         * @param {Object} config    — filtering options map
         */
        solve(stations, algo = 'ls_aoa', config = {}) {
            const filtered = this.filterStations(stations, config);
            if (!filtered || filtered.length < 2) return null;
            if (algo === 'midpoint' && filtered.length === 2) return midpoint(filtered);
            if (algo === 'bayesian') return bayesianGrid(filtered);
            return lsAoA(filtered);
        },
    };
})();
