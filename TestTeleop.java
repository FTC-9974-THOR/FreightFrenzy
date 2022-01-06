package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

import org.ftc9974.thorcore.util.BooleanEdgeDetector;

@TeleOp(name = "Test Teleop", group = "Teleops")
public class TestTeleop extends OpMode {

    MecanumDrive md;

    private boolean manualArmControlActive;

    private BooleanEdgeDetector controlModeSwitchDetector;
    private boolean controlModeSwitch;

    @Override
    public void init() {
        md = new MecanumDrive(hardwareMap);

        controlModeSwitch = false;
        controlModeSwitchDetector = new BooleanEdgeDetector(false);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        controlModeSwitchDetector.update(gamepad1.back);
        if (controlModeSwitchDetector.isRising()) {
            controlModeSwitch = !controlModeSwitch;
        }

        //kathir mode
        if (controlModeSwitch) {
            md.drive(-gamepad1.right_stick_x, -gamepad1.right_stick_y,-gamepad1.left_stick_x);
        } else {
            md.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }

        telemetry.update();
    }

    @Override
    public void stop(){

    }
}

