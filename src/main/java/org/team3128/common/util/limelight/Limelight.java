package org.team3128.common.util.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.awt.Robot;

import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.units.Angle;

/**
 * 
 * @author Adham Elarabawy, Mason Holst, Jude Lifset
 *
 */
public class Limelight
{    
    public double camAngle;
    public double camHeight;
    public double hatchHeight;
    public double w;

    public boolean debugMode;

    public NetworkTableInstance defInst;
    public NetworkTable table;
    public NetworkTable calcValsTable;
    public NetworkTableEntry nd, nd0, nd1, ntheta, ntheta0, ntheta1, ndeltax, ndeltay;
    
    public Limelight(double passedAngle, double passedCamHeight, double passedHatchHeight, double passedW, boolean passedDebugMode){
        //passed vals
        camAngle = passedAngle;
        camHeight = passedCamHeight;
        hatchHeight = passedHatchHeight;
        passedW = w;
        debugMode = passedDebugMode;

        //setting up networktables
        defInst = NetworkTableInstance.getDefault();
        table = defInst.getTable("limelight");
        calcValsTable = defInst.getTable("calculatedLimelight");
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
                runningTotal += table.getEntry(valueKey).getDouble(0.0);
            }

            data.set(valueKey, runningTotal / numSamples);
        }

        data.set("tx", -1 * data.tx());

        return data;
    }
    public CalculatedData doMath(LimelightData inData) {
        CalculatedData outData = new CalculatedData();
        double targetArcAngle = inData.boxWidth() * LimelightConstants.horizFOV/(LimelightConstants.screenWidth);

        outData.theta0 = inData.tx() - targetArcAngle / 2; 
        outData.theta1 = inData.tx() + targetArcAngle / 2;

        outData.d = (hatchHeight - camHeight) / RobotMath.tan(inData.ty() + camAngle);

        //d0 = ((2*d*Math.cos(tx-theta0)-Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta0)),2)-(4*d*d)+(w*w))))/2;
        //d1 = ((2*d*Math.cos(tx-theta1)+Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta1)),2)-(4*d*d)+(w*w))))/2;

        outData.d0 = (2 * outData.d * Math.cos(inData.tx() - outData.theta0) - Math.sqrt(RobotMath.square(w) - 4 * RobotMath.square(outData.d) * RobotMath.square(RobotMath.sin(inData.tx() - outData.theta0)))) / 2;
        outData.d1 = (2 * outData.d * Math.cos(outData.theta1 - inData.tx()) - Math.sqrt(RobotMath.square(w) - 4 * RobotMath.square(outData.d) * RobotMath.square(RobotMath.sin(outData.theta1 - inData.tx())))) / 2;

        outData.dX = outData.d * RobotMath.sin(inData.tx());
        outData.dY = outData.d * RobotMath.cos(inData.tx());

        outData.theta = RobotMath.asin((outData.d1*RobotMath.cos(outData.theta1) - outData.d0*RobotMath.cos(outData.theta0))/w);

        // nd.setString(String.valueOf(d));
        // nd0.setString(String.valueOf(d0));
        // nd1.setString(String.valueOf(d1));
        // ntheta.setString(String.valueOf(theta));
        // ntheta0.setString(String.valueOf(theta0));
        // ntheta1.setString(String.valueOf(theta1));
        // ndeltax.setString(String.valueOf(deltax));
        // ndeltay.setString(String.valueOf(deltay));

        return outData;
    }

    public void ledOn(){
        table.getEntry("ledMode").setNumber(3);
    }
    public void ledOff(){
        table.getEntry("ledMode").setNumber(1);
    }
    public void ledBlink(){
        table.getEntry("ledMode").setNumber(2);
    }
}
