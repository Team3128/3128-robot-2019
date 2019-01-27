package org.team3128.testbench.main;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.limelight.Limelight;
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

    public double camAngle = 26;
    public double camHeight = 6.15;
    public double hatchHeight = 28.5;
    public double w = 14.5;

    public int ledToggle = 0;

    Limelight limelight;

    public double[] limelightVals;
    public double[] calculatedVals;

	@Override
	protected void constructHardware()
	{
        table = NetworkTableInstance.getDefault().getTable("limelight");
        limelight = new Limelight(table, camAngle, camHeight, hatchHeight, w);


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
                bw.write("tx,ty,ts,ta,thoriz,tvert,tshort,tlong,deltax,deltay,theta,calculatedDist\r\n");
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
            if(ledToggle == 0){
                limelight.ledOn();
                ledToggle = 1;
            } else {
                limelight.ledOff();
                ledToggle = 0;
            }
        });
        /*listenerRight.nameControl(new Button(2), "LightOff");
		listenerRight.addButtonUpListener("LightOff", () -> {
		    table.getEntry("ledMode").setNumber(1);
        });*/
        listenerRight.nameControl(new Button(12), "ResetCounter");
        listenerRight.addButtonDownListener("ResetCounter", () -> {
            counter = 0;
        });

		listenerRight.nameControl(ControllerExtreme3D.TRIGGER, "limelightVals");
		listenerRight.addButtonDownListener("limelightVals", () -> {
            newLine = ""; 
            try {
                limelightVals = limelight.getValues(1000);
                calculatedVals = limelight.doMath();
                Log.info("trigger", "trigger triggered");
                newLine = newLine + String.valueOf(limelightVals[0]) + ",";
                newLine = newLine + String.valueOf(limelightVals[1]) + ",";
                newLine = newLine + String.valueOf(limelightVals[2]) + ",";
                newLine = newLine + String.valueOf(limelightVals[3]) + ",";
                newLine = newLine + String.valueOf(limelightVals[4]) + ",";
                newLine = newLine + String.valueOf(limelightVals[5]) + ",";
                newLine = newLine + String.valueOf(limelightVals[6]) + ",";
                newLine = newLine + String.valueOf(limelightVals[7]) + ",";
                newLine = newLine + String.valueOf(calculatedVals[0]) + ",";
                newLine = newLine + String.valueOf(calculatedVals[1]) + ",";
                newLine = newLine + String.valueOf(calculatedVals[2]) + ",";
                newLine = newLine + String.valueOf(calculatedVals[3]) + ",";

                counter++;
                bw.write(newLine + "\r\n");
                Log.info("line", newLine);
                Log.info("INFO", "Successfully Calculated and stored Data!");
                Log.info("INFO", "Counter: " + counter);
            } catch (IOException e) {
                e.printStackTrace();
            }
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
                if(f.delete()){
                    System.out.println("Files swapped");
                }
                f = new File("/media/sda1/limelightLog_temp.csv");
                boolean success = ftemp.renameTo(f);
                if (!success) {
                    Log.debug("fileswap", "temp file was not renamed properly because of reasons");
                }
                fw = new FileWriter(f, true);
                fr = new FileReader(f);
                bw = new BufferedWriter(fw);
                br = new BufferedReader(fr);
                fwtemp.close();
                //frtemp.close();
            } catch(IOException e){
                e.printStackTrace();
            }
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

    @Override
    protected void updateDashboard() {
        NarwhalDashboard.put("ts", String.valueOf(table.getEntry("ts").getDouble(0.0)));

    }

    public static void main(String[] args) {
        RobotBase.startRobot(MainTestBench::new);
    }
}