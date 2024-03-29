package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MiniArm;
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
    MiniArm miniArm;
    Intake intake;

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
        HIGH,
        STRAIGHT_UP
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
        miniArm = new MiniArm(hardwareMap);
        intake = new Intake(hardwareMap);

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
            cs.spin(DcMotorSimple.Direction.FORWARD);
        } else if (gamepad1.right_bumper){
            cs.spin(DcMotorSimple.Direction.REVERSE);
        } else {
            cs.stop();
        }


        //analog control for the intake, binary control for the outtake
        if (gamepad1.left_trigger > 0.3){
            intake.spinIntake(MathUtilities.map(gamepad1.left_trigger,0.3, 1,0,-300));//-250
        } else if (gamepad2.left_trigger > 0.3) {
            intake.spinIntake(MathUtilities.map(gamepad2.left_trigger,0.3,1,0,-300));//250
        } else if(gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3){
            intake.spinIntake(400);
        } else{
            intake.spinIntake(0);
        }

        //NEW 1/25/2022
        if(gamepad2.left_bumper){
            intake.openDoor();
        } else if(gamepad2.right_bumper){
            intake.closeDoor();
        }


        if(gamepad1.dpad_up){
            miniArm.fullyRetractLinServos();
            miniArm.upTurn();
            upDownState = UpDownState.STRAIGHT_UP;
        } else if(gamepad1.dpad_down){
            upDownState = UpDownState.STRAIGHT_UP;
            miniArm.middleLinServos();
            miniArm.straightDownTurn();
            miniArm.openClaw();
        } else if(gamepad1.dpad_left){
            miniArm.middleLinServos();
            miniArm.stowTurn();
        } else if(gamepad1.dpad_right){
            miniArm.linServoPlaceCapstone();
        } else if(gamepad1.x){
            turret.upDown.setPower(0);
        }

        if(gamepad1.b){
            miniArm.openClaw();
        } else if (gamepad1.a){
            miniArm.closeClawTSE();
        } else if (gamepad1.y){
            miniArm.closeClawBlock();
        }

        if(turret.isUpDownHomed()){
            if(gamepad2.y){
                intake.doorParOpen();
                upDownState = UpDownState.HIGH;
            } else if(gamepad2.b){
                intake.closeDoor();
                upDownState = UpDownState.LOW;
            } else if(gamepad2.a){
                intake.closeDoor();
                upDownState = UpDownState.DOWN;
            } else if(gamepad2.x){
                intake.closeDoor();
                upDownState = UpDownState.MIDDLE;
            }

            if(Math.abs(upDownInput) > 0.01){//0.01
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

            if(Math.abs(pivotInput) > 0){
                pivotState = PivotState.MANUAL;
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

                if(Math.abs(pivotInput) < 0.01){
                    pivotState = pivotState.HOLD;
                    pivotHoldPosition = turret.getPivotPosition();
                }

                //1/21/2022 I switched the order of the if statement and the two lines below
                turret.setPivotMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setPivotPowerManual(pivotInput);

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
            case STRAIGHT_UP:
                turret.setUpDownTargetPosition(90);//degrees
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
        cs.update();

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
