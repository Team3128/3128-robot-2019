package org.team3128.prebot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;
import com.kauailabs.navx.frc.AHRS;



import org.team3128.common.drive.SRXTankDrive;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Wrapper {
    private double angularVelocity, vL, vR, gL, gR;
    private String csv = "angular velocity, gL, gR";
    public Wrapper(){

    }
    public void setAngularVelocity(double temp){}
        this.angularVelocity = temp;
    }
    public void setVelocities(double tempL, double tempR){}
        this.vL = tempL;
        this.vR = tempR;
    }
    public void setG(double tempGL, double tempGR){
        this.gL = tempGL;
        this.gR = tempGR;
    }
    public void csvAppend(){
        this.csv += "\n" + String.valueOf(angularVelocity) + "," + String.valueOf(gL) + "," + String.valueOf(gR);
    }
}

public class CmdArcTurnTest extends CommandGroup {
    public CmdArcTurnTest(Wrapper wrapper, AHRS ahrs, double powerL, double powerR) {
        Wrapper data = wrapper;
        addSequential(new CmdLambda(() -> {
            SRXTankDrive.getInstance().tankdrive(powerL, powerR);

            double voltage = DriverStation.getInstance().getBatteryVoltage();

            double vL = SRXTankDrive.getInstance().getLeftMotors().getSelectedSensorVelocity() * 100000/4096 * SRXTankDrive.getInstance().wheelCircumfrence;
            double vR = SRXTankDrive.getInstance().getRightMotors().getSelectedSensorVelocity() * 100000/4096 * SRXTankDrive.getInstance().wheelCircumfrence;
            data.setVelocities(vL, vR);

            double angle = ahrs.getAngle();
            double time = RobotController.getFPGATimestamp()/1000000;
            try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
            angle += ahrs.getAngle();
            time = time - Timer.getFPGATimestamp()/1000000;

            data.setAngularVelocity(angle/time);
            data.setG(powerL/(data.vL * voltage/12), powerR/(data.vR * voltage/12));
            data.csvAppend();
        }));
    }
}