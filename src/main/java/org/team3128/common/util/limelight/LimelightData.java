package org.team3128.common.util.limelight;

import java.util.HashMap;

public class LimelightData {
    private HashMap<String, Double> values;

    public LimelightData() {
        values = new HashMap<String, Double>();

        for (String valueKey : LimelightConstants.valueKeys) {
            values.put(valueKey, 0.0);
        }
    }

    // Getters
    public double tx() {
        return values.get("tx");
    }

    public double ty() {
        return values.get("ty");
    }

    public double shear() {
        return values.get("ts");
    }

    public double area() {
        return values.get("ta");
    }

    public double boxWidth() {
        return values.get("thor");
    }

    public double boxHeight() {
        return values.get("tvert");
    }

    public double fittedShort() {
        return values.get("tshort");
    }

    public double fittedLong() {
        return values.get("tlong");
    }

    // Setter
    public void set(String valueKey, double value) {
        values.put(valueKey, value);
    }
}