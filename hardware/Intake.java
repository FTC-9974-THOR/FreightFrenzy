package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class Intake {

    @Hardware
    public DcMotorEx intake;

    @Hardware
    public ServoImplEx doorServo;

    public static final double DOOR_CLOSED = MathUtilities.map(2020, 500, 2500, 0,1),
            DOOR_OPEN = MathUtilities.map(1520, 500, 2500, 0,1);// TODO: 1/24/2022 To be experimentally using the servo tester

    public Intake(HardwareMap hardwareMap){
        Realizer.realize(this, hardwareMap);

        doorServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spinIntake(double velocity){
        intake.setVelocity(velocity);
    }

    public void openDoor(){
        doorServo.setPosition(DOOR_OPEN);
    }

    public void closeDoor(){
        doorServo.setPosition(DOOR_CLOSED);
    }
}
