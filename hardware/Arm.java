package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.BooleanEdgeDetector;


public class Arm{

    public boolean homed;

    private int encoderHomeOffset;

    private final BooleanEdgeDetector homingSensorBED;

    @Hardware
    public DcMotorEx shoulder, intake;

    @Hardware
    public TouchSensor homingSwitch;

    public static final int HOME = 0;
    public static final int TOP_PLATE = 1500;
    public static final int MIDDLE_PLATE = 1950;
    public static final int BOTTOM_PLATE = 2285;
    public static final int INTAKE_POSITION = 2700;
    public static final int STRAIGHT_UP = 1200;

    public Arm(HardwareMap hm){
        Realizer.realize(this, hm);

        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shoulder.setTargetPosition(Arm.HOME);

        homingSensorBED = new BooleanEdgeDetector(isAtHomePosition());

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveArm(int position){
        shoulder.setTargetPosition(position);
        shoulder.setPower(0.4);
    }

    public void setTargetPosition(int position){
        if (homed) {
            shoulder.setTargetPosition(position + encoderHomeOffset);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0.6);
        }
    }

    public void spinIntake(double velocity){
        intake.setVelocity(velocity);
    }

    public void setShoulderPower(double power) {
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isAtHomePosition()) {
            power = Math.max(power, 0);
        }
        if (homed && getCurrentPosition() > INTAKE_POSITION) {
            power = Math.min(power, 0);
        }
        shoulder.setPower(power);
    }

    public boolean isHomed() {
        return homed;
    }

    public boolean isAtHomePosition() {
        return homingSwitch.isPressed() || (homed && getCurrentPosition() <= HOME);
    }

    public int getCurrentPosition() {
        return shoulder.getCurrentPosition() - encoderHomeOffset;
    }

    public void update() {
        if (!homed) {
            homingSensorBED.update(isAtHomePosition());
            if (homingSensorBED.isChanging()) {
                homed = true;
                encoderHomeOffset = shoulder.getCurrentPosition();
            }
        }
    }

}
