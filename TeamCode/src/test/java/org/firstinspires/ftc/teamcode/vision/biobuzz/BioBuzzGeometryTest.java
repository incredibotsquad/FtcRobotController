package org.firstinspires.ftc.teamcode.vision.biobuzz;

import org.junit.Test;
import org.json.JSONObject;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.vision.biobuzz.BioBuzzResult.*;

public class BioBuzzGeometryTest {
    private JSONObject json() throws Exception {
        return new JSONObject(new String(Files.readAllBytes(Paths.get("src/main/assets/biobuzz.json")),StandardCharsets.UTF_8));
    }
    private BioBuzzConfig calibrated() throws Exception {
        JSONObject j=json();
        j.put("calibration",new JSONObject("{\"homography\":[[0,-60,70],[-60,0,30],[0,0,1]],\"polygon\":[[0.05,0.05],[0.95,0.05],[0.95,0.95],[0.05,0.95]],\"aspect\":1.3333333333333333}"));
        return new BioBuzzConfig(j.toString());
    }
    private Blob blob(Kind kind,double forward) {
        return new Blob(kind,0,0,20,20,300,.5,.5,false,new Position(forward,0));
    }
    @Test public void projectionRequiresCalibrationAndCoverage() throws Exception {
        assertNull(BioBuzzGeometry.project(.5,.5,new BioBuzzConfig(json().toString()),4.0/3));
        BioBuzzConfig c=calibrated();
        Position p=BioBuzzGeometry.project(.5,.5,c,4.0/3);
        assertEquals(40,p.forward,1e-6); assertEquals(0,p.left,1e-6);
        assertNull(BioBuzzGeometry.project(0,.5,c,4.0/3));
        assertNull(BioBuzzGeometry.project(.5,.5,c,16.0/9));
    }
    @Test public void rankHonorsDensityModeAndAlliance() throws Exception {
        JSONObject j=json(); j.put("cluster_radius_inches",4); j.put("distance_penalty_per_inch",.01);
        BioBuzzConfig c=new BioBuzzConfig(j.toString());
        List<Blob> blobs=Arrays.asList(blob(Kind.POLLEN,10),blob(Kind.POLLEN,35),blob(Kind.POLLEN,36),
                blob(Kind.RED_NECTAR,60),blob(Kind.RED_NECTAR,61),blob(Kind.RED_NECTAR,62),blob(Kind.BLUE_NECTAR,5));
        assertEquals(60,BioBuzzGeometry.rank(blobs,c,Alliance.RED,Mode.MIXED).get(0).position.forward,0);
        assertEquals(35,BioBuzzGeometry.rank(blobs,c,Alliance.RED,Mode.POLLEN_ONLY).get(0).position.forward,0);
        assertEquals(35,BioBuzzGeometry.rank(blobs,c,Alliance.BLUE,Mode.MIXED).get(0).position.forward,0);
        assertTrue(BioBuzzGeometry.rank(Collections.singletonList(blob(Kind.BLUE_NECTAR,5)),c,Alliance.RED,Mode.MIXED).isEmpty());
    }
    @Test public void staleAndFutureFramesAreRejected() {
        BioBuzzResult result=new BioBuzzResult(1_000_000_000L,Collections.emptyList(),Collections.emptyList(),true,false,0);
        assertTrue(result.isFresh(1_200_000_000L,250));
        assertFalse(result.isFresh(1_300_000_000L,250));
        assertFalse(result.isFresh(900_000_000L,250));
        assertFalse(BioBuzzResult.empty().isFresh(0,250));
    }
    @Test public void fieldTransformUsesLeftAndCcwHeading() {
        FieldPoint p=new Position(10,5).toField(100,200,Math.PI/2);
        assertEquals(95,p.x,1e-6); assertEquals(210,p.y,1e-6);
    }
    @Test(expected=IllegalArgumentException.class) public void rejectsInvalidFilter() throws Exception {
        JSONObject j=json(); j.put("morph_kernel",2); new BioBuzzConfig(j.toString());
    }
    @Test public void snapshotsAreImmutable() {
        BioBuzzResult result=new BioBuzzResult(1,Collections.emptyList(),Collections.emptyList(),true,false,0);
        try { result.blobs.add(blob(Kind.POLLEN,10)); fail("Snapshot should be immutable"); }
        catch(UnsupportedOperationException expected) { }
    }
}
