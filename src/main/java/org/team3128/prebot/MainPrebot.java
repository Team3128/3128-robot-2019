package org.team3128.prebot;

<<<<<<< HEAD
import org.team3128.common.NarwhalRobot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.Constants;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MainPrebot extends NarwhalRobot {
    public TalonSRX rightDriveFront;
    public TalonSRX rightDriveMiddle;
    public TalonSRX rightDriveBack;
    public TalonSRX leftDriveFront;
    public TalonSRX leftDriveMiddle;
    public TalonSRX leftDriveBack;

    public SRXTankDrive tankDrive;

    public Joystick joystick;

    public ListenerManager lm;

    public ADXRS450_Gyro gyro;

    public double wheelDiameter;

    @Override
	protected void constructHardware() {
        rightDriveFront = new TalonSRX(0);
        rightDriveMiddle = new TalonSRX(1);
        rightDriveBack = new TalonSRX(2);
        leftDriveFront = new TalonSRX(3);
        leftDriveMiddle = new TalonSRX(4);
        leftDriveBack = new TalonSRX(5);

        rightDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        leftDriveFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.CAN_TIMEOUT);
        
        rightDriveMiddle.set(ControlMode.Follower, rightDriveFront.getDeviceID());
        leftDriveMiddle.set(ControlMode.Follower, leftDriveFront.getDeviceID());
        rightDriveBack.set(ControlMode.Follower, rightDriveFront.getDeviceID());
        leftDriveBack.set(ControlMode.Follower, rightDriveFront.getDeviceID());

        wheelDiameter = 3.68 * Length.in;
        SRXTankDrive.initialize(rightDriveFront, leftDriveFront, wheelDiameter * Math.PI, 1, 23.70*Length.in, 28.45*Length.in, 400);
        tankDrive = SRXTankDrive.getInstance();
        
        joystick = new Joystick(1);
		lm = new ListenerManager(joystick);
        addListenerManager(lm);
        
        gyro = new ADXRS450_Gyro();
		gyro.calibrate();
    }
    
    @Override
    protected void constructAutoPrograms() {

    }

	@Override
	protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
		lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
		lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");		

        lm.addMultiListener(() -> {
			tankDrive.arcadeDrive(/*.5 * */lm.getAxis("MoveTurn"),
					lm.getAxis("MoveForwards"),
					-1 * lm.getAxis("Throttle"),
					true);		
        }, "MoveTurn", "MoveForwards", "Throttle");
	}
    
    @Override
    protected void updateDashboard() {
        SmartDashboard.putNumber("Gyro Angle", RobotMath.normalizeAngle(gyro.getAngle()));
        SmartDashboard.putNumber("Left Drive Current", leftDriveFront.getOutputCurrent());
        SmartDashboard.putNumber("Right Drive Current", leftDriveFront.getOutputCurrent());
        SmartDashboard.putNumber("Left Distance (in)", tankDrive.encDistanceToCm(leftDriveFront.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU) / Length.in);
		SmartDashboard.putNumber("Right Distance (in)", tankDrive.encDistanceToCm(rightDriveFront.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU) / Length.in);
		SmartDashboard.putNumber("Left Encoder Position", leftDriveFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Encoder Position", rightDriveFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Left Speed (nu)", leftDriveFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Right Speed (nu)", rightDriveFront.getSelectedSensorVelocity(0));
		
    }
=======
import edu.wpi.first.wpilibj.RobotBase;

public final class MainPrebot {
  private MainPrebot() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(PrebotRobot::new);
  }
>>>>>>> d08c47aec9817089955b131e5bee31e258ef1183
}
