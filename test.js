const assert = require('assert');
const fs = require('fs');
const path = require('path');

const inputPath = process.argv[2] || path.join('data', 'mock_bearings.json');
const rawText = fs.readFileSync(inputPath, 'utf8');
const records = path.extname(inputPath).toLowerCase() === '.jsonl'
    ? rawText.split(/\r?\n/).filter(Boolean).map(line => JSON.parse(line))
    : JSON.parse(rawText).waypoint_sequence;

function numberOrNull(value) {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
}

function normalize(record, index) {
    const lat = numberOrNull(record.lat ?? record.lat_deg);
    const lon = numberOrNull(record.lon ?? record.lon_deg);
    const bearing = numberOrNull(record.bearing_deg ?? record.doa_deg);
    assert.notStrictEqual(lat, null, `record ${index} is missing latitude`);
    assert.notStrictEqual(lon, null, `record ${index} is missing longitude`);
    assert.notStrictEqual(bearing, null, `record ${index} is missing bearing`);

    const papr = numberOrNull(record.doa_papr_db ?? record.confidence_0_1);
    const qualityScore = numberOrNull(
        record.quality_score ?? record.spectrum_peakiness_0_1 ?? record.confidence
    );
    return {
        lat,
        lon,
        bearing_deg: ((bearing % 360) + 360) % 360,
        doa_papr_db: papr,
        quality_score: qualityScore === null ? 1 : Math.max(0, Math.min(1, qualityScore)),
        usable_for_triangulation: record.usable_for_triangulation !== false,
    };
}

const normalized = records.map(normalize);
const validObs = normalized
    .filter(obs => obs.usable_for_triangulation)
    .filter(obs => obs.doa_papr_db === null || obs.doa_papr_db >= 0.3)
    .slice(-30);

assert(validObs.length >= 2, 'not enough valid observations for triangulation');

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const EARTH_R = 6371000;

function latLonToENU(lat, lon, refLat, refLon) {
    const dLat = (lat - refLat) * DEG2RAD;
    const dLon = (lon - refLon) * DEG2RAD;
    return {
        E: dLon * EARTH_R * Math.cos(refLat * DEG2RAD),
        N: dLat * EARTH_R,
    };
}

function enuToLatLon(E, N, refLat, refLon) {
    return {
        lat: refLat + (N / EARTH_R) * RAD2DEG,
        lon: refLon + (E / (EARTH_R * Math.cos(refLat * DEG2RAD))) * RAD2DEG,
    };
}

function weight(station) {
    return station.quality_score > 0 ? station.quality_score : 1;
}

function lsAoA(stations) {
    const refLat = stations.reduce((sum, station) => sum + station.lat, 0) / stations.length;
    const refLon = stations.reduce((sum, station) => sum + station.lon, 0) / stations.length;
    const AtWA = [[0, 0], [0, 0]];
    const AtWb = [0, 0];

    for (const station of stations) {
        const { E, N } = latLonToENU(station.lat, station.lon, refLat, refLon);
        const theta = (90 - station.bearing_deg) * DEG2RAD;
        const a = Math.sin(theta);
        const b = -Math.cos(theta);
        const c = a * E - Math.cos(theta) * N;
        const w = weight(station);
        AtWA[0][0] += w * a * a;
        AtWA[0][1] += w * a * b;
        AtWA[1][0] += w * b * a;
        AtWA[1][1] += w * b * b;
        AtWb[0] += w * a * c;
        AtWb[1] += w * b * c;
    }

    const determinant = AtWA[0][0] * AtWA[1][1] - AtWA[0][1] * AtWA[1][0];
    assert(Math.abs(determinant) >= 1e-10, 'bearing matrix is singular');
    const x = (AtWb[0] * AtWA[1][1] - AtWb[1] * AtWA[0][1]) / determinant;
    const y = (AtWA[0][0] * AtWb[1] - AtWA[1][0] * AtWb[0]) / determinant;
    const result = enuToLatLon(x, y, refLat, refLon);
    assert(Number.isFinite(result.lat) && Number.isFinite(result.lon), 'invalid LS-AoA result');
    return result;
}

const result = lsAoA(validObs);
console.log(`PASS: ${inputPath} (${records.length} records, ${validObs.length} used)`);
console.log(`LS-AoA result: ${result.lat.toFixed(6)}, ${result.lon.toFixed(6)}`);
