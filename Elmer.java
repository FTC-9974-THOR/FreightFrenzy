package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

import org.ftc9974.thorcore.util.BooleanEdgeDetector;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.List;

@TeleOp(name = "Elmer", group = "Teleops")
public class Elmer extends OpMode {

    MecanumDrive md;
    Turret turret;
    CarouselSpinner cs;

    public enum PivotState{
        AUTOMATIC,
        MANUAL
    }

    public enum UpDownState{
        HOLD,
        MANUAL,
        LOW,
        MIDDLE,
        HIGH
    }

    PivotState pivotState;
    UpDownState upDownState;

    private BooleanEdgeDetector controlModeSwitchDetector;
    private boolean controlModeSwitch;

    private List<LynxModule> lynxModules;

    @Override
    public void init() {
        md = new MecanumDrive(hardwareMap);
        turret = new Turret(hardwareMap);
        cs = new CarouselSpinner(hardwareMap);

        controlModeSwitch = false;
        controlModeSwitchDetector = new BooleanEdgeDetector(false);

        pivotState = PivotState.MANUAL;
        upDownState = UpDownState.MANUAL;

        turret.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //telemetry.addData("PIDF Coefficients:" ,arm.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    @Override
    public void init_loop(){
      clearBulkCache();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        clearBulkCache();

        double pivotInput = MathUtilities.applyDeadband(-gamepad2.left_stick_x, 0.05);
        double upDownInput = MathUtilities.applyDeadband(-gamepad2.right_stick_y, 0.05);


        controlModeSwitchDetector.update(gamepad1.back);
        if (controlModeSwitchDetector.isRising()) {
            controlModeSwitch = !controlModeSwitch;
        }

        //kathir mode
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
            turret.spinIntake(-400);
        } else if (gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3) {
            turret.spinIntake(400);
        } else {
            turret.spinIntake(0);
        }
        if (turret.isUpDownHomed()){
            if(gamepad2.dpad_up){
                upDownState = UpDownState.LOW;
            } else if(gamepad2.dpad_left){
                upDownState = UpDownState.MIDDLE;
            } else if (gamepad2.dpad_down){
                upDownState = UpDownState.HIGH;
            } else if (gamepad2.dpad_right){

            }

            if(Math.abs(upDownInput) > 0){
                upDownState = UpDownState.MANUAL;
            }
        }

        switch(pivotState){
            case AUTOMATIC:
                break;
            case MANUAL:
                turret.setPivotPowerManual(pivotInput);
                break;
            default:
        }

        switch(upDownState){
            case HOLD:
                if (upDownInput != 0) {
                    upDownState = UpDownState.MANUAL;
                }
                break;
            case MANUAL:
                if(upDownInput == 0){
                    upDownState = UpDownState.HOLD;
                }
                turret.setPivotMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setUpDownPowerManual(upDownInput);
                break;
            case LOW:
                turret.setUpDownTargetPosition(40);//degrees
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                break;
            case MIDDLE:
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                break;
            case HIGH:
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                break;
            default:
        }

        turret.update();

        telemetry.addData("Pivot homed:" ,turret.isPivotHomed());
        telemetry.addData("Pivot position in degrees:", turret.getPivotPosition());
        telemetry.addData("Pivot position in ticks:", turret.degreesToTicks(turret.getPivotPosition()));

        telemetry.addData("UpDown homed:", turret.isUpDownHomed());
        telemetry.addData("UpDown position in degrees:", turret.getUpDownPosition());
        telemetry.addData("UpDown position in ticks:", turret.degreesToTicks(turret.getUpDownPosition()));

        telemetry.addData("Pivot State Machine", pivotState);
        telemetry.addData("Up/Down State Machine", upDownState);
        telemetry.update();
    }

    @Override
    public void stop(){

    }

    private void clearBulkCache(){
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.clearBulkCache();
        }
    }
}

