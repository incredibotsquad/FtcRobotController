package org.firstinspires.ftc.teamcode.vision.biobuzz;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/** Immutable configuration; loaded from the same JSON used by the Mac tester. */
public final class BioBuzzConfig {
    public final int width, height, maxBlobsPerColor, maxAgeMs, settleMs;
    final int kernel;
    final double minArea, maxArea, minFill, maxAspect;
    final double radius, pollenWeight, nectarWeight, opponentPenalty, distancePenalty, maxRange;
    final double[] roi;
    final int[][][] hsv;
    final double[] homography;
    final double[][] polygon;
    final double calibrationAspect;

    public BioBuzzConfig(String json) throws JSONException {
        JSONObject c = new JSONObject(json);
        require(c.getInt("version") == 1, "Unsupported configuration version");
        width = c.getInt("width"); height = c.getInt("height");
        maxBlobsPerColor = c.getInt("max_blobs_per_color");
        maxAgeMs = c.getInt("max_age_ms"); settleMs = c.getInt("settle_ms");
        kernel = c.getInt("morph_kernel");
        require(width > 0 && height > 0 && maxBlobsPerColor > 0 && maxAgeMs > 0 && settleMs >= 0, "Invalid dimensions/timing/limit");
        require(kernel >= 1 && kernel <= 15 && kernel % 2 == 1, "Invalid morph_kernel");
        minArea = number(c, "min_area_fraction"); maxArea = number(c, "max_area_fraction");
        minFill = number(c, "min_fill"); maxAspect = number(c, "max_aspect");
        require(0 < minArea && minArea <= maxArea && maxArea <= 1 && minFill >= 0 && minFill <= 1 && maxAspect > 0, "Invalid blob filters");
        radius = number(c, "cluster_radius_inches"); maxRange = number(c, "max_range_inches");
        pollenWeight = number(c, "pollen_weight"); nectarWeight = number(c, "nectar_weight");
        opponentPenalty = number(c, "opponent_penalty"); distancePenalty = number(c, "distance_penalty_per_inch");
        require(radius > 0 && maxRange > 0 && pollenWeight >= 0 && nectarWeight >= 0 && opponentPenalty >= 0 && distancePenalty >= 0, "Invalid ranking parameters");
        JSONArray r = c.getJSONArray("roi"); require(r.length() == 4, "Invalid ROI");
        roi = new double[4]; for (int i = 0; i < 4; i++) roi[i] = r.getDouble(i);
        require(0 <= roi[0] && roi[0] < roi[2] && roi[2] <= 1 && 0 <= roi[1] && roi[1] < roi[3] && roi[3] <= 1, "Invalid ROI");
        hsv = new int[3][][];
        for (BioBuzzResult.Kind kind : BioBuzzResult.Kind.values()) {
            JSONArray ranges = c.getJSONObject("hsv").getJSONArray(kind.name());
            require(ranges.length() > 0, "Empty HSV ranges");
            hsv[kind.ordinal()] = new int[ranges.length()][6];
            for (int j = 0; j < ranges.length(); j++) {
                JSONArray range = ranges.getJSONArray(j); require(range.length() == 6, "Invalid HSV range");
                for (int i = 0; i < 6; i++) hsv[kind.ordinal()][j][i] = range.getInt(i);
                int[] v = hsv[kind.ordinal()][j];
                for (int i = 0; i < 3; i++) require(0 <= v[i] && v[i] <= v[i+3] && v[i+3] <= (i == 0 ? 179 : 255), "Invalid HSV limits");
            }
        }
        if (c.isNull("calibration")) {
            homography = null; polygon = null; calibrationAspect = 0;
        } else {
            JSONObject cal = c.getJSONObject("calibration");
            calibrationAspect = number(cal, "aspect"); require(calibrationAspect > 0, "Invalid calibration aspect");
            JSONArray h = cal.getJSONArray("homography"), p = cal.getJSONArray("polygon");
            require(h.length() == 3 && p.length() == 4, "Invalid calibration dimensions");
            homography = new double[9]; polygon = new double[4][2];
            for (int i = 0; i < 3; i++) {
                require(h.getJSONArray(i).length() == 3, "Invalid homography");
                for (int j = 0; j < 3; j++) {
                    homography[i*3+j] = h.getJSONArray(i).getDouble(j);
                    require(Double.isFinite(homography[i*3+j]), "Nonfinite homography");
                }
            }
            double[] a = homography;
            double det = a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);
            require(Math.abs(det) >= 1e-9, "Singular homography");
            for (int i = 0; i < 4; i++) {
                require(p.getJSONArray(i).length() == 2, "Invalid polygon point");
                for (int j = 0; j < 2; j++) {
                    polygon[i][j] = p.getJSONArray(i).getDouble(j);
                    require(polygon[i][j] >= 0 && polygon[i][j] <= 1, "Invalid polygon point");
                }
            }
            double sign = 0, area = 0;
            for (int i = 0; i < 4; i++) {
                double[] p0 = polygon[i], p1 = polygon[(i+1)%4], p2 = polygon[(i+2)%4];
                double cross = (p1[0]-p0[0])*(p2[1]-p1[1])-(p1[1]-p0[1])*(p2[0]-p1[0]);
                require(cross != 0 && (sign == 0 || cross*sign > 0), "Polygon must be convex and ordered");
                sign = cross; area += p0[0]*p1[1]-p1[0]*p0[1];
            }
            require(Math.abs(area)/2 >= .001, "Calibration polygon too small");
        }
    }
    public boolean isCalibrated() { return homography != null; }
    private static double number(JSONObject c, String key) throws JSONException {
        double value = c.getDouble(key); require(Double.isFinite(value), "Nonfinite " + key); return value;
    }
    private static void require(boolean good, String message) {
        if (!good) throw new IllegalArgumentException(message);
    }
}
