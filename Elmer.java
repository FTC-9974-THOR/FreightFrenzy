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
        HOLD,
        MANUAL,
        FORWARD,
        RED_SHARED_HUB,
        BLUE_SHARED_HUB,
        BACKWARD
    }

    public enum UpDownState{
        HOLD,
        MANUAL,
        DOWN,
        LOW,
        MIDDLE,
        HIGH
    }

    PivotState pivotState;
    UpDownState upDownState;

    private BooleanEdgeDetector controlModeSwitchDetector;
    private boolean controlModeSwitch;

    private double pivotHoldPosition, upDownHoldPosition;

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
            md.drive(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);
        } else {
            md.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }

        if(gamepad1.left_bumper){
            cs.spin(0.75);
        } else if (gamepad1.right_bumper){
            cs.spin(-0.75);
        } else{
            cs.spin(0);
        }

        //analog control for the intake, binary control for the outtake
        if (gamepad1.left_trigger > 0.3){
            turret.spinIntake(MathUtilities.map(gamepad1.left_trigger,0.3, 1,0,-300));//-250
        } else if (gamepad2.left_trigger > 0.3) {
            turret.spinIntake(MathUtilities.map(gamepad2.left_trigger,0.3,1,0,-300));//250
        } else if(gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3){
            turret.spinIntake(400);
        } else{
            turret.spinIntake(0);
        }

        if(turret.isUpDownHomed()){
            if(gamepad2.y){
                upDownState = UpDownState.HIGH;
            } else if(gamepad2.b){
                upDownState = UpDownState.LOW;
            } else if(gamepad2.a){
                upDownState = UpDownState.DOWN;
            } else if(gamepad2.x){
                upDownState = UpDownState.MIDDLE;
            }

            if(Math.abs(upDownInput) > 0.01){
                upDownState = UpDownState.MANUAL;
            }
        }

        if (turret.isPivotHomed()){
            if (turret.getUpDownPosition() > 25) {
                if(gamepad2.dpad_up){
                    pivotState = PivotState.FORWARD;
                } else if(gamepad2.dpad_left){
                    pivotState = PivotState.BLUE_SHARED_HUB;
                } else if (gamepad2.dpad_down){
                    pivotState = PivotState.BACKWARD;
                } else if (gamepad2.dpad_right){
                    pivotState = PivotState.RED_SHARED_HUB;
                }
            }

            if(Math.abs(upDownInput) > 0){
                upDownState = UpDownState.MANUAL;
            }
        }


        switch(pivotState){
            case HOLD:

                turret.setPivotTargetPosition(pivotHoldPosition);
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(1);

                if (Math.abs(pivotInput)> 0.01) {
                    pivotState = pivotState.MANUAL;
                }

                break;
            case MANUAL:

                turret.setPivotMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setPivotPowerManual(pivotInput);

                if(Math.abs(pivotInput) < 0.01){
                    pivotState = pivotState.HOLD;
                    turret.setPivotTargetPosition(turret.getPivotPosition());
                }

                break;
            case FORWARD:
                turret.setPivotTargetPosition(0);//degrees
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(1);
                break;
            case RED_SHARED_HUB:
                turret.setPivotTargetPosition(-97);//degrees
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(1);
                break;
            case BLUE_SHARED_HUB:
                turret.setPivotTargetPosition(97);//degrees
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(1);
                break;
            case BACKWARD:
                turret.setPivotTargetPosition(Math.copySign(180, turret.getPivotPosition()));//degrees
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(1);
                break;
        }

        switch(upDownState){
            case HOLD:

                turret.setUpDownTargetPosition(upDownHoldPosition);
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.7);

                if (Math.abs(upDownInput) > 0.01){
                    upDownState = UpDownState.MANUAL;
                }

                break;
            case MANUAL:

                if(Math.abs(upDownInput) < 0.01){
                    upDownState = UpDownState.HOLD;
                    upDownHoldPosition = turret.getUpDownPosition();
                }

                turret.setUpDownMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turret.setUpDownPowerManual(upDownInput);

                break;

            case LOW:
                turret.setUpDownTargetPosition(27);//degrees
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(1);
                break;
            case MIDDLE:
                turret.setUpDownTargetPosition(32);//degrees
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                break;
            case HIGH:
                turret.setUpDownTargetPosition(57);//degrees
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                break;
            case DOWN:
                turret.setUpDownTargetPosition(0);//degrees
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.9);
                break;
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

        telemetry.addData("UpDown Velocity in degrees", turret.getUpDownVelocity());
        telemetry.addData("Pivot Input", pivotInput);
        telemetry.addData("UpDown Input", upDownInput);


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

