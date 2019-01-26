package org.team3128.testbench.main;

import java.util.ArrayList;

public class Limelight {

    public HashMap<String, Double> tx, ty, ts, ta, thoriz, tvert, tshort, tlong; //variables for big bounding box(dual target)
    public HashMap<String, Double> tyL, taL, thorizL, tvertL, tshortL, tlongL; //variables for LEFT small bounding box(single target)
    public HashMap<String, Double> tyR, taR, thorizR, tvertR, tshortR, tlongR; //variables for RIGHT small bounding box(single target)

    public double d, d0, d1, theta, theta0, theta1, deltax, deltay, ratio;

    final double w = 12;
    public File f;
    public File ftemp;
    BufferedWriter bw;
    BufferedReader br;
    FileWriter fw;
    FileReader fr;
    FileWriter fwtemp;
    FileReader frtemp;
    BufferedReader brtemp;
    BufferedWriter bwtemp;
    String currentLine, newLine;
    private int errorCase = 0;
    private int counter = 0;

    final double camAngle = 26;
    final double camHeight = 6.15;
    final double hatchHeight = 28.5;

    public ArrayList<HashMap> dualTargets = new ArrayList<HashMap>();
    public ArrayList<HashMap> leftTargets = new ArrayList<HashMap>();
    public ArrayList<HashMap> rightTargets = new ArrayList<HashMap>();
    public ArrayList<HashMap> finalValues = new ArrayList<HashMap>();
    
    public void fillArray(ArrayList array, String name) {
        int i = 0;
        if (name.equalsIgnoreCase("dualTargets")) {
            dualTargets[0].put("tx", 0.0);
            dualTargets[1].put("ty", 0.0);
            dualTargets[2].put("ts", 0.0);
            dualTargets[3].put("ta", 0.0);
            dualTargets[4].put("thoriz", 0.0);
            dualTargets[5].put("tvert", 0.0);
            dualTargets[6].put("tshort", 0.0);
            dualTargets[7].put("tlong", 0.0);
            Log.info("LIMELIGHT", "Dual Target Array Filled.");
            i = 1;
        }
        if (name.equalsIgnoreCase("leftTargets")) {
            leftTargets[0].put("tyL", 0.0);
            leftTargets[1].put("taL", 0.0);
            leftTargets[2].put("thorizL", 0.0);
            leftTargets[3].put("tvertL", 0.0);
            leftTargets[4].put("tshortL", 0.0);
            leftTargets[5].put("tlongL", 0.0);
            Log.info("LIMELIGHT", "Left Target Array Filled.");
            i = 1;
        }
        if (name.equalsIgnoreCase("rightTargets")) {
            rightTargets[0].put("tyR", 0.0);
            rightTargets[1].put("taR", 0.0);
            rightTargets[2].put("thorizR", 0.0);
            rightTargets[3].put("tvertR", 0.0);
            rightTargets[4].put("tshortR", 0.0);
            rightTargets[5].put("tlongR", 0.0);
            Log.info("LIMELIGHT", "Right Target Array Filled.");
            i = 1;
        }
        if (name.equalsIgnoreCase("finalValues")) {
            finalValues[0].put("d", 0.0);
            finalValues[1].put("d0", 0.0);
            finalValues[2].put("d1", 0.0);
            finalValues[3].put("theta", 0.0);
            finalValues[4].put("theta0", 0.0);
            finalValues[5].put("theta1", 0.0);
            finalValues[6].put("deltax", 0.0);
            finalValues[7].put("deltay", 0.0);
            finalValues[8].put("ratio", 0.0);
            Log.info("LIMELIGHT", "Final Values Array Filled.");
            i = 1;
        }
        if (i == 0) {
            Log.info("LIMELIGHT ERROR", name + " could not be filled.");
        }
        
    }

    public void fillAllArray() {
        clearAllArray();
        fillArray(dualTargets, "dualTargets");
        fillArray(rightTargets, "rightTargets");
        fillArray(leftTargets, "leftTargets");
        fillArray(finalValues, "finalValues");
        Log.info("LIMELIGHT", "All Arrays Filled.");
    }

    public void clearAllArray() {
        dualTargets.clear();
        leftTargets.clear();
        rightTargets.clear();
        finalValues.clear();
        Log.info("LIMELIGHT", "All Arrays Cleared.");
    }

    public void setArrayValues(ArrayList a, double x) {
        for (int i = 0; i < a.length(), i++) {
            a[i].put(a[i].keySet()[0], x);
        }
        Log.info("LIMELIGHT", "Target Array values set to " + x);
    }

    public String getName(HashMap map) {
        return map.keySet()[0];
    }

    public double getValue(HashMap map) {
        return map.get(map.keySet()[0]);
    }

    public void averageValuesArray(ArrayList a, int returnCount, NetworkTable table, int cnt) {
        //ArrayList<Double> values = new ArrayList<Double>();
        for (int i = 0; i < cnt, i++) {
            for(int d = 0; d < returnCount; d++) {
                a[i].put(getName(a[i]), getValue(a[i]) + table.getEntry(getName(a[i])).getDouble(0.0));
            }
            a[i].put(getName(a[i]), getValue(a[i]) / returnCount);
        }
    }

    public void recordLimelight(NetworkTable table, int returnCount) {
        newLine = ""; 
        try {
            Log.info("LIMELIGHT", "Initialized Limelight Data Record");
            setArrayValues(dualTargets, 0.0);
            setArrayValues(rightTargets, 0.0);
            setArrayValues(leftTargets, 0.0);

            table.getEntry("pipeline").setDouble(0.0);
            averageValuesArray(dualTargets, 2000, table, 6);
            
            table.getEntry("pipeline").setDouble(1.0);
            averageValuesArray(leftTargets, 2000, table, 3);

            table.getEntry("pipeline").setDouble(2.0);
            averageValuesArray(rightTargets, 2000, table, 3);
            
            table.getEntry("pipeline").setDouble(0.0);
            ratio.put("ratio", getValue(taL)/getValue(taR));

            newLine = newLine + getValue(tx) + ",";
            newLine = newLine + getValue(ty) + ",";
            newLine = newLine + getValue(ts) + ",";
            newLine = newLine + getValue(ta) + ",";
            newLine = newLine + getValue(taL) + ",";
            newLine = newLine + getValue(taR) + ",";
            newLine = newLine + getValue(ratio) + ",";
            newLine = newLine + getValue(thoriz) + ",";
            newLine = newLine + getValue(tvert) + ",";
            newLine = newLine + getValue(thorizL) + ",";
            newLine = newLine + getValue(tvertL) + ",";
            newLine = newLine + getValue(thorizR) + ",";
            newLine = newLine + getValue(tvertR);

            doMath();

            counter++;
            NarwhalDashboard.put("counter", String.valueOf(counter));
            NarwhalDashboard.put("txav", getValue(tx));
            NarwhalDashboard.put("tyav", getValue(ty));
            NarwhalDashboard.put("tzav", getValue(ts));
            NarwhalDashboard.put("taav", getValue(ta));
            bw.write(newLine + "\r\n");
            Log.info("line", newLine);
            Log.info("tx", getValue(tx));
            Log.info("ty", getValue(ty));
            Log.info("ts", getValue(ts));
            Log.info("ta", getValue(ta));
            Log.info("taL", getValue(taL));
            Log.info("taR", getValue(taR));
            Log.info("ratio", getValue(ratio));
            Log.info("thoriz", getValue(thoriz));
            Log.info("tvert", getValue(tvert));
            Log.info("thorizL", getValue(thorizL));
            Log.info("tvertL", getValue(tvertL));
            Log.info("thorizR", getValue(thorizR));
            Log.info("tvertR", getValue(tvertR));
            Log.info("INFO", "Successfully Calculated and stored Data!");
            Log.info("INFO", "Counter: " + counter);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void doMath(){
        getValue(theta0) = getValue(tx) - 0.5*(59.6*getValue(thoriz)/320);//possibly change to tlong/tshort based on how well this works
        getValue(theta1) = getValue(tx) + 0.5*(59.6*getValue(thoriz)/320);

        getValue(d) = (hatchHeight-camHeight)/Math.tan((Math.PI/180)*(getValue(ty)+camAngle));
        getValue(d0) = (hatchHeight-camHeight)/Math.tan((Math.PI/180)*(getValue(tyR)+camAngle));
        getValue(d1) = (hatchHeight-camHeight)/Math.tan((Math.PI/180)*(getValue(tyL)+camAngle));

        getValue(deltax) = getValue(d)*Math.sin((Math.PI/180)*getValue(tx));
        getValue(deltay) = getValue(d)*Math.cos((Math.PI/180)*getValue(tx));

        getValue(theta) = Math.asin((getValue(d1)*Math.sin((180/Math.PI)*getValue(theta1)) - getValue(d0)*Math.sin((180/Math.PI)*getValue(theta0)))/w);
    }
}