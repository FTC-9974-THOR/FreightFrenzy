package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ftc9974.thorcore.meta.annotation.Hardware;

public class Turret{

    @Hardware
    DcMotorEx turret;

    int ticksPerRevolution;

    public enum AngleUnit{
        DEGREES,
        RADIANS
    }

    /*public double tickstoAngle(int ticks, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES) {
            return ((ticks) * (360 / ticksPerRevolution));
        }

        if (angleUnit == AngleUnit.RADIANS) {
            return ((ticks) * ((2 * Math.PI) / ticksPerRevolution));
        }

    }*/

    public void setTicksPerRevolution(int ticksPerRev){
        ticksPerRev = ticksPerRevolution;
    }

    public void setLimits(int lowerLimit, int upperLimit, DcMotor motor){
        if((motor.getCurrentPosition()< lowerLimit) || (motor.getCurrentPosition() > upperLimit)){
            motor.setPower(0);
        }
    }
}
