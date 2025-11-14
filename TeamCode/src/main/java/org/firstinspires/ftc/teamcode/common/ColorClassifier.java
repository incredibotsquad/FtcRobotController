package org.firstinspires.ftc.teamcode.common;

public class ColorClassifier {

    public static final class Result {
        public final GameColors color;
        public final double confidence; // 0..1 (rough)
        Result(GameColors c, double conf){ this.color=c; this.confidence=conf; }
    }

    // ---------- Tuned thresholds ----------
    // Dimness floor (accept faint but valid reads)
    private static final double MIN_CONFIDENCE_V = 0.0018; //0.0025;

    // --- WHITE: “sure” test (spandex-safe) ---
    private static final double WHITE_MIN_V = 0.0070;          // includes your W5 sample
    private static final double WHITE_MAX_CHROMA_NORM = 0.29;
    private static final double WHITE_MIN_R_OVER_G = 0.52;
    private static final double WHITE_MAX_R_OVER_G = 0.59;
    private static final double WHITE_MIN_G_OVER_B = 1.06;
    private static final double WHITE_MAX_G_OVER_B = 1.12;
    private static final double WHITE_MAX_GREEN_STRENGTH  = 0.80; // g/(r+b)
    private static final double WHITE_MAX_PURPLE_STRENGTH = 0.75; // b/(r+g)
    private static final double WHITE_ACCEPT_SCORE = 0.65;        // accept if >= this score

    // --- GREEN: require hue band + dominance/ratios ---
    private static final double GREEN_H_MIN = 155.0;
    private static final double GREEN_H_MAX = 175.0;
    private static final double GREEN_STRONG_STRENGTH = 0.85;     // g/(r+b)
    private static final double GREEN_MIN_S_LOWER      = 0.45;     // allows your low-S green sample
    private static final double GREEN_MIN_G_OVER_B     = 1.06;     // g/b

    // --- PURPLE: support empirical (your data ~220–245) and classic wrap (270–30) ---
    private static final double PURPLE_MIN_S = 0.42;
    private static final double PURPLE_MIN_PURPLE_STR = 0.78; //0.80;      // b/(r+g)
    private static final double[][] PURPLE_H_RANGES = new double[][] {
            {218.0, 245.0},   // your blue-violet cluster (~226–230°) //220 - 245
            {270.0, 360.0},   // classic purple range (wrap)
            {0.0,   30.0}
    };

    // ---------- (Optional) quick white calibration windows ----------
    private static double calWhiteRGMin = WHITE_MIN_R_OVER_G;
    private static double calWhiteRGMax = WHITE_MAX_R_OVER_G;
    private static double calWhiteGBMin = WHITE_MIN_G_OVER_B;
    private static double calWhiteGBMax = WHITE_MAX_G_OVER_B;

    /** Optional: call once pre-match with 10–20 normalized WHITE samples to tighten windows. */
    public static void calibrateWhite(double[] r, double[] g, double[] b) {
        if (r == null || g == null || b == null || r.length == 0 || r.length != g.length || r.length != b.length) return;
        int n = r.length;
        double[] rg = new double[n], gb = new double[n];
        for (int i=0;i<n;i++){ rg[i] = r[i]/(g[i]+1e-9); gb[i] = g[i]/(b[i]+1e-9); }
        double rgMean = mean(rg), rgStd = std(rg, rgMean);
        double gbMean = mean(gb), gbStd = std(gb, gbMean);
        double rgMin = Math.max(WHITE_MIN_R_OVER_G, rgMean - 3*rgStd);
        double rgMax = Math.min(WHITE_MAX_R_OVER_G, rgMean + 3*rgStd);
        double gbMin = Math.max(WHITE_MIN_G_OVER_B, gbMean - 3*gbStd);
        double gbMax = Math.min(WHITE_MAX_G_OVER_B, gbMean + 3*gbStd);
        if (rgMin < rgMax) { calWhiteRGMin = rgMin; calWhiteRGMax = rgMax; }
        if (gbMin < gbMax) { calWhiteGBMin = gbMin; calWhiteGBMax = gbMax; }
    }

    // ---------- Public APIs ----------
    public static GameColors classify(double r, double g, double b) {
        return classifyWithScore(r,g,b).color;
    }

    public static Result classifyWithScore(double r, double g, double b) {
        if (!Double.isFinite(r) || !Double.isFinite(g) || !Double.isFinite(b)) return new Result(GameColors.UNKNOWN, 0);

        double v = Math.max(r, Math.max(g, b));
        if (v < MIN_CONFIDENCE_V) return new Result(GameColors.UNKNOWN, 0);

        // 1) Sure WHITE first (spandex-safe). Early return if accepted.
        double wScore = whiteScore(r,g,b);
        if (wScore >= WHITE_ACCEPT_SCORE) return new Result(GameColors.NONE, clamp01(wScore));

        // 2) HSV + dominance
        HSV hsv = rgbToHsv(r, g, b);
        double h = hsv.h, s = hsv.s;
        double greenStrength  = g / (r + b + 1e-9);
        double purpleStrength = b / (r + g + 1e-9);
        double gOverB = g / (b + 1e-9);

        // 3) GREEN: must be in hue band AND (strong strength OR good s + G/B)
        if (inHueRange(h, GREEN_H_MIN, GREEN_H_MAX) &&
                (greenStrength >= GREEN_STRONG_STRENGTH || (s >= GREEN_MIN_S_LOWER && gOverB >= GREEN_MIN_G_OVER_B))) {
            double conf = scoreBand(h, GREEN_H_MIN, GREEN_H_MAX) *
                    smoothStep(greenStrength, 0.65, 0.95) *
                    smoothStep(s, 0.35, 0.85);
            return new Result(GameColors.GREEN, clamp01(conf));
        }

        // 4) PURPLE: any purple band + saturation + blue dominance
        if (inAnyHueRange(h, PURPLE_H_RANGES) && s >= PURPLE_MIN_S && purpleStrength >= PURPLE_MIN_PURPLE_STR) {
            double conf = Math.max(scoreBands(h, PURPLE_H_RANGES), 0.6) *
                    smoothStep(purpleStrength, 0.70, 1.05) *
                    smoothStep(s, 0.30, 0.90);
            return new Result(GameColors.PURPLE, clamp01(conf));
        }

        // 5) Not sure
        return new Result(GameColors.UNKNOWN, 0.0);
    }

    // ---------- WHITE scoring (0..1). We accept if >= WHITE_ACCEPT_SCORE ----------
    private static double whiteScore(double r, double g, double b) {
        double v = Math.max(r, Math.max(g, b));
        if (v < WHITE_MIN_V) return 0;

        // Normalized chroma = distance from gray axis / total intensity
        double chromaNorm = Math.sqrt((r-g)*(r-g) + (g-b)*(g-b) + (b-r)*(b-r)) / (r + g + b + 1e-9);
        if (chromaNorm > WHITE_MAX_CHROMA_NORM) return 0;

        double RG = r / (g + 1e-9);
        double GB = g / (b + 1e-9);
        if (RG < calWhiteRGMin || RG > calWhiteRGMax) return 0;
        if (GB < calWhiteGBMin || GB > calWhiteGBMax) return 0;

        double greenStrength  = g / (r + b + 1e-9);
        double purpleStrength = b / (r + g + 1e-9);
        if (greenStrength > WHITE_MAX_GREEN_STRENGTH)  return 0;
        if (purpleStrength > WHITE_MAX_PURPLE_STRENGTH) return 0;

        // Confidence: how central RG/GB are inside calibrated window
        double rgCenter = centerScore(RG, calWhiteRGMin, calWhiteRGMax);
        double gbCenter = centerScore(GB, calWhiteGBMin, calWhiteGBMax);
        return 0.6*rgCenter + 0.4*gbCenter;
    }

    // ---------- Utilities ----------
    private static class HSV { final double h, s, v; HSV(double h, double s, double v){this.h=h; this.s=s; this.v=v;} }

    // Robust RGB->HSV (no % with doubles). Hue in [0,360).
    private static HSV rgbToHsv(double r, double g, double b) {
        double max = Math.max(r, Math.max(g, b));
        double min = Math.min(r, Math.min(g, b));
        double v = max;
        double d = max - min;
        double s = (max <= 1e-12) ? 0 : d / max;

        double h;
        if (d < 1e-12) {
            h = 0.0;
        } else if (max == r) {
            h = 60.0 * ((g - b) / d);
        } else if (max == g) {
            h = 60.0 * ((b - r) / d) + 120.0;
        } else {
            h = 60.0 * ((r - g) / d) + 240.0;
        }
        if (h < 0) h += 360.0;
        return new HSV(h, s, v);
    }

    private static boolean inHueRange(double h, double min, double max) {
        if (min <= max) return (h >= min && h <= max);
        // wraparound (e.g., 350–20)
        return (h >= min && h < 360.0) || (h >= 0.0 && h <= max);
    }

    private static boolean inAnyHueRange(double h, double[][] ranges) {
        for (double[] r : ranges) if (inHueRange(h, r[0], r[1])) return true;
        return false;
    }

    // Confidence helpers
    private static double scoreBand(double h, double min, double max) {
        double mid = (min <= max) ? (0.5*(min+max)) : (((min + (max+360.0)) * 0.5) % 360.0);
        double span = (min <= max) ? (0.5*(max-min)) : (0.5*((max+360.0)-min));
        double dist = angularDistance(h, mid);
        return smoothStep(1.0 - (dist / Math.max(span, 1e-6)), 0.0, 1.0);
    }
    private static double scoreBands(double h, double[][] ranges) {
        double best = 0;
        for (double[] r : ranges) best = Math.max(best, scoreBand(h, r[0], r[1]));
        return best;
    }
    private static double angularDistance(double a, double b) {
        double d = Math.abs(a - b) % 360.0;
        return d > 180.0 ? 360.0 - d : d;
    }
    private static double centerScore(double x, double min, double max) {
        double mid = 0.5*(min+max), half = Math.max(1e-6, 0.5*(max-min));
        double t = 1.0 - Math.abs(x - mid)/half; // 1 at center, 0 at edges
        return clamp01(t);
    }
    private static double smoothStep(double x, double lo, double hi) {
        if (x <= lo) return 0;
        if (x >= hi) return 1;
        double t = (x - lo) / (hi - lo);
        return t * t * (3 - 2*t);
    }
    private static double mean(double[] a){ double s=0; for(double v:a) s+=v; return s/a.length; }
    private static double std(double[] a, double m){ double s=0; for(double v:a){ double d=v-m; s+=d*d; } return Math.sqrt(s/Math.max(1,a.length-1)); }
    private static double clamp01(double x){ return x<0?0:(x>1?1:x); }
}

