package org.firstinspires.ftc.teamcode.ffpoc.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.BooleanEdgeDetector;

public class Arm {

    public static final int HOME_POSITION = 0;
    public static final int TOP_PLATE = 1500;
    public static final int MIDDLE_PLATE = 1950;
    public static final int BOTTOM_PLATE = 2360;
    public static final int INTAKE_POSITION = 2700;

    @Hardware
    public DcMotorEx shoulder;

    @Hardware
    public TouchSensor homingSwitch;

    @Hardware
    public DcMotorEx intake;

    private final BooleanEdgeDetector homingDetector;
    private boolean homed;

    private int encoderHomeOffset;

    public Arm(HardwareMap hw) {
        Realizer.realize(this, hw);

        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        homingDetector = new BooleanEdgeDetector(isAtHomePosition());

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isAtHomePosition() {
        return homingSwitch.isPressed() || (homed && getCurrentPosition() <= HOME_POSITION);
    }

    public int getCurrentPosition() {
        return shoulder.getCurrentPosition() - encoderHomeOffset;
    }

    public int getPositionError() {
        return shoulder.getTargetPosition() - shoulder.getCurrentPosition();
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

    public void setTargetPosition(int target) {
        if (homed) {
            shoulder.setTargetPosition(target + encoderHomeOffset);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0.6);
        }
    }

    public boolean isInClosedLoopMode() {
        return shoulder.getMode() == DcMotor.RunMode.RUN_TO_POSITION;
    }

    public boolean isHomed() {
        return homed;
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void update() {
        if (!homed) {
            homingDetector.update(isAtHomePosition());
            if (homingDetector.isChanging()) {
                homed = true;
                encoderHomeOffset = intake.getCurrentPosition();
            }
        }
    }
}
