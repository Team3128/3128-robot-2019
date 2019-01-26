package org.team3128.testbench.main;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.util.units.Length;
import org.team3128.testbench.autonomous.*;

import edu.wpi.first.wpilibj.Joystick;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.BufferedWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MainTestBench extends NarwhalRobot {

    public TalonSRX boi1, boi2;
    public ListenerManager listenerLeft, listenerRight;
    public Joystick leftJoystick, rightJoystick;
    public double wheelCirc;
    public int lowGearMaxSpeed;
    public SRXTankDrive drive;
    public NetworkTable table;
    double tx, ty, ta, ts, taL, taR, ratio, thoriz, tvert, thorizL, tvertL, thorizR, tvertR;

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
    String currentLine;
    String newLine;


    private int errorCase = 0;
    private int counter = 0;

	@Override
	protected void constructHardware()
	{
		table = NetworkTableInstance.getDefault().getTable("limelight");

        try {
            f = new File("/media/sda1/limelightLog.csv");
            if(!f.exists()) {
                f.createNewFile();
            }
            fw = new FileWriter(f);
            fr = new FileReader(f);
        } catch (IOException e) {
            //e.printStackTrace();
            errorCase = 1;
            Log.info("ERROR", "------------------------------------------------");
            Log.info("ERROR", "Limelight Log File not found or not connected!");
        }

        if (errorCase == 0) {
            bw = new BufferedWriter(fw);
            br = new BufferedReader(fr);
            try{
                bw.write("tx,ty,ts,ta,taL,taR,ratio,thoriz,tvert,thorizL,tvertL,thorizR,tvertR\r\n");
                //bw.newLine();
                //bw.close();
            } catch(IOException e){
                e.printStackTrace();
            }
        } else {
            Log.info("ERROR", "File capabilities offline as no file was found!");
            Log.info("ERROR", "------------------------------------------------");
        }
        NarwhalDashboard.put("counter", String.valueOf(counter));
        boi1 = new TalonSRX(1);
        boi2 = new TalonSRX(2);

        SRXTankDrive.initialize(boi1, boi2, wheelCirc, 1, 25.25 * Length.in, lowGearMaxSpeed);
        drive = SRXTankDrive.getInstance();
        
        leftJoystick = new Joystick(0);
        rightJoystick = new Joystick(1);

        listenerLeft = new ListenerManager(leftJoystick);
        listenerRight = new ListenerManager(rightJoystick);

        addListenerManager(listenerLeft);
        addListenerManager(listenerRight);

    }

    @Override
    protected void setupListeners() {

        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");

		listenerRight.addMultiListener(() ->
		{
			double x = listenerRight.getAxis("MoveForwards");
			double y = listenerRight.getAxis("MoveTurn");
			double t = listenerRight.getAxis("Throttle") * -1;
			drive.arcadeDrive(x, y, t, true);
        }, "MoveForwards", "MoveTurn", "Throttle");
        
        listenerRight.nameControl(new Button(2), "LightOn");
		listenerRight.addButtonDownListener("LightOn", () -> {
            table.getEntry("ledMode").setNumber(3);
            Log.debug("Limelight Latency", String.valueOf(table.getEntry("tl").getDouble(0.0)));
  
        });
        /*listenerRight.nameControl(new Button(2), "LightOff");
		listenerRight.addButtonUpListener("LightOff", () -> {
		    table.getEntry("ledMode").setNumber(1);
        });*/
        listenerRight.nameControl(new Button(12), "ResetCounter");
        listenerRight.addButtonDownListener("ResetCounter", () -> {
            counter = 0;
        });

		listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "LightBlink");
		listenerRight.addButtonDownListener("LightBlink", () -> {
            newLine = ""; 
            try {
                //fw = new FileWriter(f, true);
                //bw = new BufferedWriter(fw);
                Log.info("trigger", "trigger triggered");
                tx = 0.0;
                ty = 0.0;
                ts = 0.0;
                ta = 0.0;
                taL = 0.0;
                taR = 0.0;
                thoriz = 0.0;
                tvert = 0.0;
                thorizL = 0.0;
                tvertL = 0.0;
                thorizR = 0.0;
                tvertR = 0.0;
                for(int i = 0; i<2000; i++){
                    tx = tx + table.getEntry("tx").getDouble(0.0);
                    ty = ty + table.getEntry("ty").getDouble(0.0);
                    ts = ts + table.getEntry("ts").getDouble(0.0);
                    ta = ta + table.getEntry("ta").getDouble(0.0);
                    thoriz = thoriz + table.getEntry("tshort").getDouble(0.0);
                    tvert = tvert + table.getEntry("tlong").getDouble(0.0);
                }
                tx = tx/2000;
                ty = ty/2000;
                ts = ts/2000;
                ta = ta/2000;
                thoriz = thoriz/2000;
                tvert = tvert/2000;
                table.getEntry("pipeline").setDouble(1.0);
                for(int i = 0; i<2000; i++){
                    taL = taL + table.getEntry("ta").getDouble(0.0);
                    thorizL = thorizL + table.getEntry("tshort").getDouble(0.0);
                    tvertL = tvertL + table.getEntry("tlong").getDouble(0.0);
                }
                taL = taL/2000;
                thorizL = thorizL/2000;
                tvertL = tvertL/2000;
                table.getEntry("pipeline").setDouble(2.0);
                for(int i = 0; i<2000; i++){
                    taR = taR + table.getEntry("ta").getDouble(0.0);
                    thorizR = thorizR + table.getEntry("tshort").getDouble(0.0);
                    tvertR = tvertR + table.getEntry("tlong").getDouble(0.0);
                }
                taR = taR/2000;
                thorizR = thorizR/2000;
                tvertR = tvertR/2000;
                table.getEntry("pipeline").setDouble(0.0);
                ratio = taL/taR;
                newLine = newLine + String.valueOf(tx) + ",";
                newLine = newLine + String.valueOf(ty) + ",";
                newLine = newLine + String.valueOf(ts) + ",";
                newLine = newLine + String.valueOf(ta) + ",";
                newLine = newLine + String.valueOf(taL) + ",";
                newLine = newLine + String.valueOf(taR) + ",";
                newLine = newLine + String.valueOf(ratio) + ",";
                newLine = newLine + String.valueOf(thoriz) + ",";
                newLine = newLine + String.valueOf(tvert) + ",";
                newLine = newLine + String.valueOf(thorizL) + ",";
                newLine = newLine + String.valueOf(tvertL) + ",";
                newLine = newLine + String.valueOf(thorizR) + ",";
                newLine = newLine + String.valueOf(tvertR);
                NarwhalDashboard.put("counter", String.valueOf(counter));
                NarwhalDashboard.put("txav", String.valueOf(tx));
                NarwhalDashboard.put("tyav", String.valueOf(ty));
                NarwhalDashboard.put("tzav", String.valueOf(ts));
                NarwhalDashboard.put("taav", String.valueOf(ta));
                /*NarwhalDashboard.put("ratio", String.valueOf(valCurrent3));
                NarwhalDashboard.put("thoriz", String.valueOf(valCurrent4));
                NarwhalDashboard.put("txav", String.valueOf(valCurrent1));
                NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
                NarwhalDashboard.put("tzav", String.valueOf(valCurrent3));
                NarwhalDashboard.put("taav", String.valueOf(valCurrent4));
                NarwhalDashboard.put("txav", String.valueOf(valCurrent1));
                NarwhalDashboard.put("tyav", String.valueOf(valCurrent2));
                NarwhalDashboard.put("tzav", String.valueOf(valCurrent3));
                NarwhalDashboard.put("taav", String.valueOf(valCurrent4));
                NarwhalDashboard.put("asdfadsf", String.valueOf(asdgasdg));
                bw.write(String.valueOf(tx));
                bw.write(",");
                bw.write(String.valueOf(ty));
                bw.write(",");
                bw.write(String.valueOf(ts));
                bw.write(",");
                bw.write(String.valueOf(ta));
                bw.write(",");
                bw.write(String.valueOf(taL));
                bw.write(",");
                bw.write(String.valueOf(taR));
                bw.write(",");
                bw.write(String.valueOf(ratio));*/
                bw.write(newLine + "\r\n");
                //bw.newLine();
                //bw.close();
                Log.info("line", newLine);
                Log.info("tx", String.valueOf(tx));
                Log.info("ty", String.valueOf(ty));
                Log.info("ts", String.valueOf(ts));
                Log.info("ta", String.valueOf(ta));
                Log.info("taL", String.valueOf(taL));
                Log.info("taR", String.valueOf(taR));
                Log.info("ratio", String.valueOf(ratio));
                Log.info("thoriz", String.valueOf(thoriz));
                Log.info("tvert", String.valueOf(tvert));
                Log.info("thorizL", String.valueOf(thorizL));
                Log.info("tvertL", String.valueOf(tvertL));
                Log.info("thorizR", String.valueOf(thorizR));
                Log.info("tvertR", String.valueOf(tvertR));
                Log.info("INFO", "Successfully Calculated and stored Data!");
                Log.info("INFO", "Counter: " + counter);
            } catch (IOException e) {
                e.printStackTrace();
            }
            /*try {
                bw.close();
            } catch (IOException e) {
                e.printStackTrace();
            }*/
        });
        
        listenerRight.nameControl(new Button(9), "deleteLastLine");
        listenerRight.addButtonDownListener("deleteLastLine", () -> {
            try {
                bw.close();
            } catch(IOException e) {
                e.printStackTrace();
            }
            try {
                //fw = new FileWriter(f);
                //bw = new BufferedWriter(fw);
                ftemp = new File("/media/sda1/limelightLog_temp.csv");
                if(!ftemp.exists()) {
                    ftemp.createNewFile();
                }
                fwtemp = new FileWriter(ftemp);
                frtemp = new FileReader(ftemp);
                bwtemp = new BufferedWriter(fwtemp);
                brtemp = new BufferedReader(frtemp);
                
                while((currentLine = br.readLine()) != null){
                    String trimmedLine = currentLine.trim();
                    if(trimmedLine.equals(newLine)){
                        Log.info("here", currentLine);
                        currentLine = "";
                    }
                    bwtemp.write(currentLine);
                    //bw = new BufferedWriter(fw);
                    //This doesn't work yet, may need to write to a new file when deleting old lines
                    //bw.write(currentLine + System.getProperty("line.separator"));
                //bw.close();
                }
                if(ftemp.delete()){
                    System.out.println("Files swapped");
                }
            } catch(IOException e){
                e.printStackTrace();
            }
            try {
                fw = new FileWriter(f, true);
            } catch (IOException e) {
                e.printStackTrace();
            }
            bw = new BufferedWriter(fw);
        });
        listenerRight.nameControl(new Button(10), "closeFile");
        listenerRight.addButtonDownListener("closeFile", () -> {
            try {
                bw.close();
            } catch(IOException e) {
                e.printStackTrace();
            }
        });
        listenerRight.nameControl(new Button(5), "pipeline_0");
        listenerRight.addButtonDownListener("pipeline_0", () -> {
            table.getEntry("pipeline").setDouble(0.0);
            Log.info("pipeline", "0");
        });
        listenerRight.nameControl(new Button(3), "pipeline_1");
        listenerRight.addButtonDownListener("pipeline_1", () -> {
            table.getEntry("pipeline").setDouble(1.0);
            Log.info("pipeline", "1");
        });
        listenerRight.nameControl(new Button(4), "pipeline_2");
        listenerRight.addButtonDownListener("pipeline_2", () -> {
            table.getEntry("pipeline").setDouble(2.0);
            Log.info("pipeline", "2");
        });
        listenerRight.nameControl(new Button(7), "CamMode");
        listenerRight.addButtonDownListener("CamMode", () -> {
            table.getEntry("camMode").setNumber(0);
            Log.debug("Limelight Latency", String.valueOf(table.getEntry("tl").getDouble(0.0)));
  
        });

        listenerRight.nameControl(new Button(8), "DriveMode");
        listenerRight.addButtonDownListener("DriveMode", () -> {
            table.getEntry("camMode").setNumber(1);
            Log.debug("Limelight Latency", String.valueOf(table.getEntry("tl").getDouble(0.0)));
  
        });
        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "takeValues");
        listenerLeft.addButtonDownListener("takeValues", () -> {
        });
    }

    @Override
    protected void constructAutoPrograms() {
        NarwhalDashboard.addAuto("Test", new TestBenchTest(drive));
        //Log.debug("hehexd", NarwhalDashboard.getSelectedAuto().getName());
    }

    @Override
    protected void teleopInit() {

    }

    @Override
    protected void teleopPeriodic() {


    }

    @Override
    protected void autonomousInit() {
        //Log.debug("micko dode", "i t w ro k  se?");
    }

    public static void main(String[] args) {
        RobotBase.startRobot(MainTestBench::new);
    }

}