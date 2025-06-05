package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Environment;
import org.tensorflow.lite.Interpreter;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.Paths;

public class TensorFlow {
    private Interpreter tflite;
    private float[] scalerXMean;
    private float[] scalerXScale;
    private float[] scalerYMean;
    private float[] scalerYScale;

    private static final String TFLITE_FOLDER = "/sdcard/FIRST/tflite/";
    private static final String MODEL_FILE = "vision_calibration_model.tflite";
    private static final String SCALER_X_FILE = "scaler_X_params.json";
    private static final String SCALER_Y_FILE = "scaler_y_params.json";

    // Constructor
    public TensorFlow() {
        try {
            // Load the TFLite model from SD card
            tflite = new Interpreter(loadModelFromSDCard());

            // Load scaler parameters from SD card
            loadScalerParamsFromSDCard();

        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to initialize VisionCalibration: " + e.getMessage());
        }
    }

    // Load TFLite model from SD card
    private MappedByteBuffer loadModelFromSDCard() throws IOException {
        String modelPath = TFLITE_FOLDER + MODEL_FILE;
        File modelFile = new File(modelPath);

        if (!modelFile.exists()) {
            throw new IOException("Model file not found at: " + modelPath);
        }

        FileInputStream inputStream = new FileInputStream(modelFile);
        FileChannel fileChannel = inputStream.getChannel();
        long fileSize = modelFile.length();

        return fileChannel.map(FileChannel.MapMode.READ_ONLY, 0, fileSize);
    }

    // Load scaler parameters from JSON files on SD card
    private void loadScalerParamsFromSDCard() throws Exception {
        // Load input scaler parameters
        String scalerXPath = TFLITE_FOLDER + SCALER_X_FILE;
        String scalerXJson = readFileAsString(scalerXPath);
        JSONObject scalerXObj = new JSONObject(scalerXJson);
        JSONArray meanArray = scalerXObj.getJSONArray("mean");
        JSONArray scaleArray = scalerXObj.getJSONArray("scale");

        scalerXMean = new float[meanArray.length()];
        scalerXScale = new float[scaleArray.length()];

        for (int i = 0; i < meanArray.length(); i++) {
            scalerXMean[i] = (float) meanArray.getDouble(i);
            scalerXScale[i] = (float) scaleArray.getDouble(i);
        }

        // Load output scaler parameters
        String scalerYPath = TFLITE_FOLDER + SCALER_Y_FILE;
        String scalerYJson = readFileAsString(scalerYPath);
        JSONObject scalerYObj = new JSONObject(scalerYJson);
        JSONArray yMeanArray = scalerYObj.getJSONArray("mean");
        JSONArray yScaleArray = scalerYObj.getJSONArray("scale");

        scalerYMean = new float[yMeanArray.length()];
        scalerYScale = new float[yScaleArray.length()];

        for (int i = 0; i < yMeanArray.length(); i++) {
            scalerYMean[i] = (float) yMeanArray.getDouble(i);
            scalerYScale[i] = (float) yScaleArray.getDouble(i);
        }
    }

    // Helper method to read file as string
    private String readFileAsString(String filePath) throws IOException {
        File file = new File(filePath);
        if (!file.exists()) {
            throw new IOException("File not found: " + filePath);
        }

        StringBuilder content = new StringBuilder();
        try (FileInputStream fis = new FileInputStream(file);
             java.util.Scanner scanner = new java.util.Scanner(fis, "UTF-8")) {
            while (scanner.hasNextLine()) {
                content.append(scanner.nextLine()).append("\n");
            }
        }
        return content.toString();
    }

    // Scale input data using the fitted scaler parameters
    private float[] scaleInput(float[] input) {
        float[] scaled = new float[input.length];
        for (int i = 0; i < input.length; i++) {
            scaled[i] = (input[i] - scalerXMean[i]) / scalerXScale[i];
        }
        return scaled;
    }

    // Inverse scale output data using the fitted scaler parameters
    private float[] inverseScaleOutput(float[] output) {
        float[] unscaled = new float[output.length];
        for (int i = 0; i < output.length; i++) {
            unscaled[i] = output[i] * scalerYScale[i] + scalerYMean[i];
        }
        return unscaled;
    }

    // Main prediction method
    public float[] predictRealCoordinates(float observedX, float observedY, float observedAngle) {
        // Prepare input data
        float[] input = {observedX, observedY, observedAngle};
        float[] scaledInput = scaleInput(input);

        // Prepare input/output arrays for TFLite
        float[][] inputArray = {scaledInput};
        float[][] outputArray = new float[1][3]; // 3 outputs: real_x, real_y, real_angle

        // Run inference
        tflite.run(inputArray, outputArray);

        // Inverse scale the output
        float[] prediction = inverseScaleOutput(outputArray[0]);

        return prediction; // [real_x, real_y, real_angle]
    }

    // Convenience method that returns individual values
    public class CalibrationResult {
        public float realX;
        public float realY;
        public float realAngle;

        public CalibrationResult(float x, float y, float angle) {
            this.realX = x;
            this.realY = y;
            this.realAngle = angle;
        }

        @Override
        public String toString() {
            return String.format("Real: (%.2f, %.2f, %.2f°)", realX, realY, realAngle);
        }
    }

    public CalibrationResult calibrate(float observedX, float observedY, float observedAngle) {
        float[] result = predictRealCoordinates(observedX, observedY, observedAngle);
        return new CalibrationResult(result[0], result[1], result[2]);
    }

    // Method to check if all required files exist
    public static boolean areFilesAvailable() {
        File modelFile = new File(TFLITE_FOLDER + MODEL_FILE);
        File scalerXFile = new File(TFLITE_FOLDER + SCALER_X_FILE);
        File scalerYFile = new File(TFLITE_FOLDER + SCALER_Y_FILE);

        return modelFile.exists() && scalerXFile.exists() && scalerYFile.exists();
    }

    // Method to get file status for debugging
    public static String getFileStatus() {
        StringBuilder status = new StringBuilder();
        status.append("File Status:\n");

        File modelFile = new File(TFLITE_FOLDER + MODEL_FILE);
        File scalerXFile = new File(TFLITE_FOLDER + SCALER_X_FILE);
        File scalerYFile = new File(TFLITE_FOLDER + SCALER_Y_FILE);

        status.append("Model file: ").append(modelFile.exists() ? "✓ Found" : "✗ Missing").append("\n");
        status.append("Scaler X file: ").append(scalerXFile.exists() ? "✓ Found" : "✗ Missing").append("\n");
        status.append("Scaler Y file: ").append(scalerYFile.exists() ? "✓ Found" : "✗ Missing").append("\n");

        return status.toString();
    }

    // Clean up resources
    public void close() {
        if (tflite != null) {
            tflite.close();
        }
    }
}