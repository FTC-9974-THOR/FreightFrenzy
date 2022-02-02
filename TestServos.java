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

    private List<LynxModule> lynxModules;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        miniArm = new MiniArm(hardwareMap);


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


        turret.update();

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

