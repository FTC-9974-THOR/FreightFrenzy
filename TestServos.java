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

@TeleOp(name = "Test Servos", group = "Teleops")
public class TestServos extends OpMode {

    Turret turret;
    MiniArm miniArm;

    public enum UpDownState{
        HOLD,
        MANUAL,
        DOWN,
        LOW,
        MIDDLE,
        HIGH,
        STRAIGHT_UP
    }

    UpDownState upDownState;

    private BooleanEdgeDetector controlModeSwitchDetector;
    private boolean controlModeSwitch;

    private double upDownHoldPosition;

    private List<LynxModule> lynxModules;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        miniArm = new MiniArm(hardwareMap);

        controlModeSwitch = false;
        controlModeSwitchDetector = new BooleanEdgeDetector(false);
        upDownState = UpDownState.MANUAL;

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

        double upDownInput = MathUtilities.applyDeadband(-gamepad2.right_stick_y, 0.05);


        if(gamepad1.dpad_up){
            miniArm.fullyRetractLinServos();
        } else if(gamepad1.dpad_down){
            miniArm.fullyExtendLinServos();
        } else if(gamepad1.dpad_left){
            miniArm.middleLinServos();
        } else if(gamepad1.dpad_right){
            miniArm.lowerRetractLinServos();
        }

        if(gamepad1.b){
            miniArm.upTurn();
        } else if (gamepad1.a){
            miniArm.straightOutTurn();
        } else if (gamepad1.x){
            miniArm.downTurn();
        } else if (gamepad1.y){
            miniArm.straightDownTurn();
        }

        if(gamepad1.left_bumper){
            miniArm.closeClawBlock();
        } else if (gamepad1.right_bumper){
            miniArm.openClaw();
        }


        if(turret.isUpDownHomed()){
            if(gamepad2.y){
                upDownState = UpDownState.STRAIGHT_UP;//DIFFERENT
            } else if(gamepad2.b){
                upDownState = UpDownState.LOW;
            } else if(gamepad2.a){
                upDownState = UpDownState.DOWN;
            } else if(gamepad2.x){
                upDownState = UpDownState.MIDDLE;
            }

            if(Math.abs(upDownInput) > 0.01){//0.01
                upDownState = UpDownState.MANUAL;
            }
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

        telemetry.addData("UpDown homed:", turret.isUpDownHomed());
        telemetry.addData("UpDown position in degrees:", turret.getUpDownPosition());
        telemetry.addData("UpDown position in ticks:", turret.degreesToTicks(turret.getUpDownPosition()));
        telemetry.addData("Up/Down State Machine", upDownState);

        telemetry.addData("UpDown Velocity in degrees", turret.getUpDownVelocity());
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

