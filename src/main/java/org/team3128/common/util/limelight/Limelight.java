package org.team3128.common.util.limelight;

import edu.wpi.first.networktables.NetworkTable;
import org.team3128.common.util.Log;

/**
 * 
 * @author Adham, Mason
 *
 */
public class Limelight
{
    public double tx, ty, ts, ta, thoriz, tvert, tshort, tlong;
    public double d, d0, d1, theta, theta0, theta1, deltax, deltay, ratio; //derived values
    public double currentAv;
    public String[] valIds = {"tx", "ty", "ts", "ta", "thor", "tvert", "tshort", "tlong"};
    public static final double screenWidth = 320;
    public static final double screenHeight = 240;
    public static final double horizFOV = 59.6;
    public static final double vertFOV = 45.7;
    public double camAngle;
    public double camHeight;
    public double hatchHeight;
    public double w;

    public double[] calculatedVals = new double[4];
    public double[] resultVals = new double[8];
    public static NetworkTable table;
    
    public Limelight(NetworkTable passedTable, double passedAngle, double passedCamHeight, double passedHatchHeight, double passedW){
        table = passedTable;
        camAngle = passedAngle;
        camHeight = passedCamHeight;
        hatchHeight = passedHatchHeight;
        passedW = w;
    }

    public double[] getValues(int loopNumber){
        for(int i=0; i<valIds.length; i++){
            for(int a=0; a<loopNumber; a++){
                currentAv = table.getEntry(valIds[i]).getDouble(0.0);
            }
            currentAv = currentAv/loopNumber;
            resultVals[i] = currentAv;
            switch(i){
                case 1: tx = currentAv;
                        break;
                case 2: ty = currentAv;
                        break;
                case 3: ts = currentAv;
                        break;
                case 4: ta = currentAv;
                        break;
                case 5: thoriz = currentAv;
                        break;
                case 6: tvert = currentAv;
                        break;
                case 7: tshort = currentAv;
                        break;
                case 8: tlong = currentAv;
                        break;
            }
        }
        return resultVals;
    }
    public double[] doMath(){
        
        theta0 = (tx - (horizFOV*thoriz)/(2*screenWidth))*Math.PI/180; 
        theta1 = (tx + (horizFOV*thoriz)/(2*screenWidth))*Math.PI/180;
        tx = tx*Math.PI/180;

        d = (hatchHeight-camHeight)/Math.tan((Math.PI/180)*(ty+camAngle)); 
        d0 = ((2*d*Math.cos(tx-theta0)-Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta0)),2)-(4*d*d)+(w*w))))/2;
        d1 = ((2*d*Math.cos(tx-theta1)+Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta1)),2)-(4*d*d)+(w*w))))/2;
        Log.info("d0", String.valueOf(d0));
        Log.info("d1", String.valueOf(d1));
        Log.info("d", String.valueOf(d));
        deltax = d*Math.sin(tx);
        deltay = d*Math.cos(tx);

        theta = Math.asin(((d1*Math.cos(theta1)) - (d0*Math.cos(theta0)))/w);

        calculatedVals[0] = deltax;
        calculatedVals[1] = deltay;
        calculatedVals[2] = theta;
        calculatedVals[3] = d;
        return calculatedVals;
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
