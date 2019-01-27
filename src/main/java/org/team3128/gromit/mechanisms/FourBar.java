package org.team3128.gromit.mechanisms;

import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;


public class FourBar
{
public enum FourBarState{
    BALL_INTAKE(0.0), 
    HATCH_PICKUP(10.0),
    LOW_DROP_OFF(40.0),
    HIGH_DROP_OFF(120.0);

    public double targetAngle;

    private FourBarState(double angle){
        this.targetAngle = angle;
    }
}

TalonSRX fourBarMotor;

}