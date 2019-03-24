package org.team3128.common.hardware.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.team3128.common.util.RobotMath;

/**
 * 
 * @author Adham Elarabawy, Mason Holst, Jude Lifset
 *
 */
public class Limelight
{    
    public double cameraAngle;
    public double cameraHeight;
    //public double targetHeight;
    public double targetWidth;
    public double centerDist;

    public NetworkTable limelightTable, calcValsTable;
    public NetworkTableEntry nd, nd0, nd1, ntheta, ntheta0, ntheta1, ndeltax, ndeltay, camtran;
    
    /**
     * 
     * @param centerDist - The vertical distance between the center of the robot's wheelbase and the camera
     * @param cameraAngle - The vertical angle of the limelight
     * @param cameraHeight - The height off of the ground of the limelight
     * @param targetWidth - The width of the target
     */
    public Limelight(double centerDist, double cameraAngle, double cameraHeight, double targetWidth) {
        this.centerDist = centerDist;
        this.cameraAngle = cameraAngle;
        this.cameraHeight = cameraHeight;
        //this.targetHeight = targetHeight;
        this.targetWidth = targetWidth;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        calcValsTable = NetworkTableInstance.getDefault().getTable("calculatedLimelight");

        nd = calcValsTable.getEntry("d");
        nd0 = calcValsTable.getEntry("d0");
        nd1 = calcValsTable.getEntry("d1");
        ntheta = calcValsTable.getEntry("theta");
        ntheta0 = calcValsTable.getEntry("theta0");
        ntheta1 = calcValsTable.getEntry("theta1");
        ndeltax = calcValsTable.getEntry("deltax");
        ndeltay = calcValsTable.getEntry("deltay");
    }

    public double getValue(String key, int numSamples){
        double runningTotal = 0;
        int count = 0;

        while (count <= numSamples) {
            if (hasValidTarget()) {
                runningTotal += limelightTable.getEntry(key).getDouble(0.0);
                count += 1;
            }
        }

        return runningTotal/numSamples;
    }
    public boolean hasValidTarget(){
        return limelightTable.getEntry("tv").getDouble(0.0) > 0.99;
    }

    public LimelightData getValues(int numSamples) {
        LimelightData data = new LimelightData();
        double runningTotal;
        double[] camtranArray;
        int index = 0;
        for(String valueKey : LimelightConstants.valueKeys) {
            runningTotal = 0;
            for(int a = 0; a < numSamples; a++){
                runningTotal += limelightTable.getEntry(valueKey).getDouble(0.0);
            }

            data.set(valueKey, runningTotal / numSamples);
        }

        data.set("tx", -1 * data.tx());
        //load the camtranArray
        camtranArray = limelightTable.getEntry("camtran").getDoubleArray(new double[6]);
        //add each element in the array to the updated values for however many times numSamples dictates(minus 1 because the array is initially loaded in with values)
        for(int a = 0; a < numSamples - 1; a++){
            for(int b = 0; b < camtranArray.length; b++){
                camtranArray[b] += limelightTable.getEntry("camtran").getDoubleArray(new double[6])[b];
            }
        }
        for(String valueKey : LimelightConstants.valueKeysPnP) {
            data.set(valueKey, camtranArray[index] / numSamples);
            index++;
        }
        
        return data;
    }

    public double getApproximateDistance(double targetHeight, int n) {
        if (!hasValidTarget()) return -1;

        return calculateDistanceFromTY(getValue("ty", n), targetHeight);
    }

    public double calculateDistanceFromTY(double ty, double targetHeight) {
        return  (targetHeight - cameraHeight) / RobotMath.tan(ty + cameraAngle);
    }
    
    public CalculatedData doMath(LimelightData inputData, double targetHeight) {
        CalculatedData outputData = new CalculatedData();
        double targetArcAngle = inputData.boxWidth() * LimelightConstants.horizFOV/LimelightConstants.screenWidth;

        outputData.theta0 = inputData.tx() - targetArcAngle / 2; 
        outputData.theta1 = inputData.tx() + targetArcAngle / 2;

        outputData.dY = (targetHeight - cameraHeight) / RobotMath.tan(inputData.ty() + cameraAngle);
        outputData.dX = outputData.dY * RobotMath.tan(inputData.tx());

        outputData.d = outputData.dY / RobotMath.cos(inputData.tx());

        double d0SqrtMultiplier = (outputData.theta0 > 0) ? -1 : 1;
        double d1SqrtMultiplier = (outputData.theta1 > 0) ? 1 : -1;
        
        outputData.d0 = outputData.d * RobotMath.cos(inputData.tx() - outputData.theta0) + d0SqrtMultiplier * 0.5 * Math.sqrt(RobotMath.square(targetWidth) - RobotMath.square(2 * outputData.d * RobotMath.sin(inputData.tx() - outputData.theta0)));
        outputData.d1 = outputData.d * RobotMath.cos(outputData.theta1 - inputData.tx()) + d1SqrtMultiplier * 0.5 * Math.sqrt(RobotMath.square(targetWidth) - RobotMath.square(2 * outputData.d * RobotMath.sin(outputData.theta1 - inputData.tx())));

        outputData.theta = RobotMath.asin((outputData.d1*RobotMath.cos(outputData.theta1) - outputData.d0*RobotMath.cos(outputData.theta0))/targetWidth);

        nd.setString(("" + outputData.d));
        nd0.setString(("" + outputData.d0));
        nd1.setString(("" + outputData.d1));
        ntheta.setString(("" + outputData.theta));
        ntheta0.setString(("" + outputData.theta0));
        ntheta1.setString(("" + outputData.theta1));
        ndeltax.setString(("" + outputData.dX));
        ndeltay.setString(("" + outputData.dY));



        return outputData;
    }

    public void turnOnLED() {
        limelightTable.getEntry("ledMode").setNumber(3);
    }
    public void turnOffLED() {
        limelightTable.getEntry("ledMode").setNumber(1);
    }
    public void blinkLED() {
        limelightTable.getEntry("ledMode").setNumber(2);
    }
    public void driverMode(int setting){
        if(setting >=0 && setting <=2){
            limelightTable.getEntry("stream").setNumber(setting);
        } else {
            limelightTable.getEntry("stream").setNumber(0);
        }
    }
}
