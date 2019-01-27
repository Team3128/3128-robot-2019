package org.team3128.common.util.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * 
 * @author Adham, Mason
 *
 */
public class Limelight
{
    public double tx, ty, ts, ta, thoriz, tvert, tshort, tlong;
    public static final double screenWidth = 320;
    public static final double screenHeight = 240;
    public static final double horizFOV = 59.6;
    public  static final double vertFOV = 45.7;
    
    public void doMath(NetworkTableInstance table){
        
        theta0 = (tx - (Limelight.horizFOV*thoriz)/(2*Limelight.screenWidth))*Math.PI/180; 
        theta1 = (tx + (Limelight.horizFOV*thoriz)/(2*Limelight.screenWidth))*Math.PI/180;
        tx = tx*Math.PI/180;

        d = (hatchHeight-camHeight)/Math.tan((Math.PI/180)*(ty+camAngle)); 
        d0 = ((2*d*Math.cos(tx-theta0)-Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta0)),2)-(4*d*d)+(w*w))))/2;
        d1 = ((2*d*Math.cos(tx-theta1)+Math.sqrt(Math.pow((-2*d*Math.cos(tx-theta1)),2)-(4*d*d)+(w*w))))/2;

        deltax = d*Math.sin(tx);
        deltay = d*Math.cos(tx);

        theta = Math.asin((d1*Math.cos(theta1) - d0*Math.cos(theta0))/w);

        Log.info("thetaCalc", String.valueOf(theta));
        Log.info("dxCalc", String.valueOf(deltax));
        Log.info("dyCalc", String.valueOf(deltay));
    }
}
