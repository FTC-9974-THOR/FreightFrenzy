package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

import org.ftc9974.thorcore.util.BooleanEdgeDetector;

@Disabled
@TeleOp(name = "Freight Frenzy", group = "Teleops")
public class FreightFrenzy extends OpMode {

    MecanumDrive md;
    Arm arm;
    CarouselSpinner cs;

    private boolean manualArmControlActive;

    private BooleanEdgeDetector controlModeSwitchDetector;
    private boolean controlModeSwitch;

    @Override
    public void init() {
        md = new MecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        cs = new CarouselSpinner(hardwareMap);

        manualArmControlActive = true;

        controlModeSwitch = false;
        controlModeSwitchDetector = new BooleanEdgeDetector(false);

        arm.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("PIDF Coefficients:" ,arm.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        double armInput = -gamepad2.right_stick_y;

        controlModeSwitchDetector.update(gamepad1.back);
        if (controlModeSwitchDetector.isRising()) {
            controlModeSwitch = !controlModeSwitch;
        }

        if (controlModeSwitch) {
            md.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
        } else {
            md.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        if(gamepad1.left_bumper){
            cs.spin(0.75);
        } else if (gamepad1.right_bumper){
            cs.spin(-0.75);
        } else{
            cs.spin(0);
        }

        if (gamepad1.left_trigger > 0.3 || gamepad2.left_trigger > 0.3) {
            arm.spinIntake(-400);
        } else if (gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3) {
            arm.spinIntake(400);
        } else {
            arm.spinIntake(0);
        }

        if(gamepad2.dpad_up){
            manualArmControlActive = false;
            arm.setTargetPosition(Arm.TOP_PLATE);
        } else if(gamepad2.dpad_left){
            manualArmControlActive = false;
            arm.setTargetPosition(Arm.MIDDLE_PLATE);
        } else if (gamepad2.dpad_down){
            manualArmControlActive = false;
            arm.setTargetPosition(Arm.BOTTOM_PLATE);
        } else if (gamepad2.dpad_right){
            manualArmControlActive = false;
            arm.setTargetPosition(Arm.STRAIGHT_UP);
        }

        if (!manualArmControlActive) {
            if (Math.abs(armInput) > 0.2) {
                manualArmControlActive = true;
                arm.setShoulderPower(armInput);
            }
        } else if (arm.shoulder.getCurrentPosition() < 2600 && arm.shoulder.getCurrentPosition() > 200) {
            arm.setShoulderPower(armInput);
        } else if (arm.shoulder.getCurrentPosition() > 2600){
            if(armInput > 0){
                arm.setShoulderPower(0);
            } else{
                arm.setShoulderPower(armInput);
            }
        } else if (arm.shoulder.getCurrentPosition() < 200){
            if(armInput < 0){
                arm.setShoulderPower(0);
            } else{
                arm.setShoulderPower(armInput);
            }
        }

        if(gamepad2.x){
            arm.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Arm position:", arm.shoulder.getCurrentPosition());
        telemetry.addData("Power:", arm.shoulder.getPower());
        telemetry.update();

        arm.update();
    }

    @Override
    public void stop(){

    }
}

