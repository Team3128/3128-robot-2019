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
        w = passedW;
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
        double targetArcAngle = inData.boxWidth() * LimelightConstants.horizFOV/LimelightConstants.screenWidth;

        outData.theta0 = inData.tx() - targetArcAngle / 2; 
        outData.theta1 = inData.tx() + targetArcAngle / 2;

        outData.dY = (hatchHeight - camHeight) / RobotMath.tan(inData.ty() + camAngle);
        outData.dX = outData.dY / RobotMath.tan(90 - inData.tx());

        outData.d = outData.dY / RobotMath.cos(inData.tx());

        //d0 = ((2*d*Math.cos(tx-theta0)-Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta0)),2)-(4*d*d)+(w*w))))/2;
        //d1 = ((2*d*Math.cos(tx-theta1)+Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta1)),2)-(4*d*d)+(w*w))))/2;

        Log.info("elkghiweurg", w + "");
        Log.info("Limelight", "inroot = " + (RobotMath.square(w) - RobotMath.square(2 * outData.d * RobotMath.sin(inData.tx() - outData.theta0))));

        outData.d0 = outData.d * RobotMath.cos(inData.tx() - outData.theta0) - 0.5 * Math.sqrt(RobotMath.square(w) - RobotMath.square(2 * outData.d * RobotMath.sin(inData.tx() - outData.theta0)));
        outData.d1 = outData.d * RobotMath.cos(outData.theta1 - inData.tx()) + 0.5 * Math.sqrt(RobotMath.square(w) - RobotMath.square(2 * outData.d * RobotMath.sin(outData.theta1 - inData.tx())));

        outData.dX = outData.dY * RobotMath.sin(inData.tx());

        outData.theta = RobotMath.asin((outData.d1*RobotMath.cos(outData.theta1) - outData.d0*RobotMath.cos(outData.theta0))/w);

        nd.setString(("" + outData.d));
        nd0.setString(("" + outData.d0));
        nd1.setString(("" + outData.d1));
        ntheta.setString(("" + outData.theta));
        ntheta0.setString(("" + outData.theta0));
        ntheta1.setString(("" + outData.theta1));
        ndeltax.setString(("" + outData.dX));
        ndeltay.setString(("" + outData.dY));

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
