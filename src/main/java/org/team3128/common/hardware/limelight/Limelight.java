package org.team3128.common.hardware.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.team3128.common.util.Log;
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
    public double targetHeight;
    public double targetWidth;

    public NetworkTable limelightTable, calcValsTable;
    public NetworkTableEntry nd, nd0, nd1, ntheta, ntheta0, ntheta1, ndeltax, ndeltay;
    
    public Limelight(double cameraAngle, double cameraHeight, double targetHeight, double targetWidth){
        this.cameraAngle = cameraAngle;
        this.cameraHeight = cameraHeight;
        this.targetHeight = targetHeight;
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

    public LimelightData getValues(int numSamples) {
        LimelightData data = new LimelightData();

        double runningTotal;

        for(String valueKey : LimelightConstants.valueKeys) {
            runningTotal = 0;

            for(int a = 0; a < numSamples; a++){
                runningTotal += limelightTable.getEntry(valueKey).getDouble(0.0);
            }

            data.set(valueKey, runningTotal / numSamples);
        }

        data.set("tx", -1 * data.tx());

        return data;
    }
    
    public CalculatedData doMath(LimelightData inputData) {
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
}
