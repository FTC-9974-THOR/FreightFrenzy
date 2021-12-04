package org.firstinspires.ftc.teamcode.ffpoc.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ffpoc.hardware.Arm;
import org.firstinspires.ftc.teamcode.ffpoc.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.ffpoc.hardware.Drivetrain;
import org.ftc9974.thorcore.util.BooleanEdgeDetector;

@Disabled
@TeleOp(name = "FF Proof Of Concept")
public class FFPOCTeleOp extends OpMode {

    private Drivetrain drivetrain;
    private Arm arm;
    private CarouselSpinner carouselWheel;

    private boolean automaticArmControlActive;

    private BooleanEdgeDetector modeSwitchEdgeDetector;
    private boolean controlModeSwitch;

    @Override
    public void init() {
        for (LynxModule lynxModule : hardwareMap.getAll(LynxModule.class)) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            lynxModule.clearBulkCache();
        }

        drivetrain = new Drivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        carouselWheel = new CarouselSpinner(hardwareMap);

        automaticArmControlActive = false;

        modeSwitchEdgeDetector = new BooleanEdgeDetector(false);
        controlModeSwitch = false;
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        telemetry.addData("Position", arm.getCurrentPosition());
    }

    @Override
    public void loop() {
        clearBulkCache();

        modeSwitchEdgeDetector.update(gamepad1.back);
        if (modeSwitchEdgeDetector.isRising()) {
            controlModeSwitch = !controlModeSwitch;
        }

        if (controlModeSwitch) {
            drivetrain.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
        } else {
            drivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        double armInput = -gamepad2.right_stick_y;
        if (gamepad2.dpad_up) {
            automaticArmControlActive = true;
            arm.setTargetPosition(Arm.TOP_PLATE);
        } else if (gamepad2.dpad_left) {
            automaticArmControlActive = true;
            arm.setTargetPosition(Arm.MIDDLE_PLATE);
        } else if (gamepad2.dpad_down) {
            automaticArmControlActive = true;
            arm.setTargetPosition(Arm.BOTTOM_PLATE);
        }

        if (automaticArmControlActive) {
            if (Math.abs(armInput) > 0.2) {
                automaticArmControlActive = false;
                arm.setShoulderPower(armInput);
            }
        } else {
            arm.setShoulderPower(armInput);
        }

        if (gamepad1.left_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
            arm.setIntakePower(-0.4);
        } else if (gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3) {
            arm.setIntakePower(0.4);
        } else {
            arm.setIntakePower(0);
        }

        if (gamepad1.left_bumper) {
            carouselWheel.setWheelPower(0.65);
        } else if (gamepad1.right_bumper) {
            carouselWheel.setWheelPower(-0.65);
        } else {
            carouselWheel.setWheelPower(0);
        }

        arm.update();
        telemetry.addData("Position", arm.getCurrentPosition());
        telemetry.addData("Homed", arm.isHomed());
    }

    private void clearBulkCache() {
        hardwareMap.getAll(LynxModule.class).forEach(LynxModule::clearBulkCache);
    }
}
