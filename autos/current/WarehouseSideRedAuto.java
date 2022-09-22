package org.firstinspires.ftc.teamcode.autos.current;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RedVisionPipeline;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.MiniArm;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Warehouse Side Red Auto", group = "autonomous")
public class WarehouseSideRedAuto extends LinearOpMode{

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;
    private CarouselSpinner cs;
    private Turret turret;
    private Intake intake;
    private MiniArm miniarm;

    public ElapsedTime et;

    RedVisionPipeline pipeline;
    RedVisionPipeline.DuckPosition position;

    private OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException{

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"webcam"));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // for some reason, streaming at 1280x720 causes EasyOpenCV to stop working. It
                // throws errors indicating it can't find a USB Streaming Endpoint. Since the FTC
                // SDK only supports streaming in uncompressed YUV420 (according to EasyOpenCV),
                // I suspect that the camera can't transfer data fast enough to stream HD YUV420.
                // if that's the case, it would explain why no endpoint was found, as the camera
                // simply doesn't have one that supports streaming in that configuration.
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //webcam.startStreaming(1280,720,OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                return;
            }
        });
        //webcam.setPipeline(new StackVisionPipeline());
        // pipeline was never initialized and webcam.setPipeline() was being given a new pipeline
        // instance. pipeline stayed null and there was no way to access the pipeline that was
        // actually doing stuff.
        pipeline = new RedVisionPipeline();
        webcam.setPipeline(pipeline);


        et = new ElapsedTime();

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(21, 96);
        navSource = new IMUNavSource(hardwareMap, 2);
        f2 = new Fusion2(this, md, calculator, navSource,new PIDFCoefficients(0.8,0,0,0), et);
        cs = new CarouselSpinner(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        miniarm = new MiniArm(hardwareMap);


        //the speed the robot will start moving at
        f2.setStartSpeed(0.4);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(10);//was 100
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(1);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(10);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(30);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.4);

        f2.setMinTurningSpeed(0.3);


        telemetry.log().add("Ready");
        telemetry.addData("Duck Position", position);
        telemetry.update();

        miniarm.closeClawBlock();
        miniarm.turnAllTheWayBack();
        miniarm.middleLinServos();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pipeline Has Analysis", pipeline.hasAnalysis());
            telemetry.addData("UpDown PID Coefficients:", turret.upDown.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            if (pipeline.hasAnalysis()) {
                telemetry.addData("Duck Position", pipeline.getAnalysis());
            }

            telemetry.addLine("STARTING POSITION: Eyes towards warehouse, side against wall, back wheels ~1 inch in front of teeth on tile closest to barrier");
            telemetry.update();
        }
        if (isStopRequested()) return;
        //waitForStart();

        md.setAxisInversion(true, true, true);
        md.setEncoderInversion(false, false, false, false);//all true
        navSource.setInverted(false);

        TimingUtilities.blockUntil(this, pipeline::hasAnalysis, null, null);
        if (isStopRequested()) return;
        // at this point we know that the pipeline must have a result. the only ways for the
        // blockUntil() to return is for isStopRequested() or pipeline.hasAnalysis() to return true.
        // if opmode stop was requested, the if statement would return before this line was reached.
        position = pipeline.getAnalysis();


        switch(position){
            case LEFT:

                miniarm.downTurn();
                miniarm.middleLinServos();

                f2.driveToPoint(new Vector2(-500, -300));
                if(isStopRequested()){
                    return;
                }

                turret.setUpDownTargetPosition(90);//straight up
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                if(isStopRequested()){
                    return;
                }

                sleep(1000);

                turret.setPivotTargetPosition(-90);
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(0.5);
                if(isStopRequested()){
                    return;
                }

                sleep(1250);

                break;
            case CENTER:

                miniarm.straightOutTurn();
                miniarm.middleLinServos();

                f2.driveToPoint(new Vector2(-500, -200));
                if(isStopRequested()){
                    return;
                }

                turret.setUpDownTargetPosition(90);//straight up
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                if(isStopRequested()){
                    return;
                }

                sleep(1000);

                turret.setPivotTargetPosition(-90);
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(0.7);
                if(isStopRequested()){
                    return;
                }

                sleep(1000);

                break;
            case RIGHT:

                miniarm.stowTurn();

                //1. drive to the alliance hub
                f2.driveToPoint(new Vector2(-750, -200));
                if(isStopRequested()){
                    return;
                }

                //2. raise the arm
                turret.setUpDownTargetPosition(90);//straight up
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                if(isStopRequested()){
                    return;
                }

                sleep(1000);

                turret.setPivotTargetPosition(-90);
                turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPivotPowerAutomatic(0.9);
                if(isStopRequested()){
                    return;
                }

                sleep(2000);

                break;
        }
        miniarm.openClaw();

        sleep(500);

        miniarm.stowTurn();
        miniarm.middleLinServos();

        turret.setPivotTargetPosition(0);
        turret.setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPivotPowerAutomatic(0.9);
        if(isStopRequested()){
            return;
        }

        sleep(1000);

        turret.setUpDownTargetPosition(27);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);
        if(isStopRequested()){
            return;
        }

        sleep(1500);

        intake.spinIntake(300);
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(1000,0), null,2000);
        if(isStopRequested()){
            return;
        }

        turret.setUpDownTargetPosition(0);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(0,1400), null,4000);
        if(isStopRequested()){
            return;
        }

        sleep(1000);

        intake.spinIntake(0);

        f2.driveToPoint(new Vector2(100,0));
        if(isStopRequested()){
            return;
        }

        turret.setUpDownTargetPosition(18);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(0,-1300));
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(200,0), null,1500);
        if(isStopRequested()){
            return;
        }

        //drives to place second freight
        f2.driveToPoint(new Vector2(-700,0));
        if(isStopRequested()){
            return;
        }

        //raises the arm
        turret.setUpDownTargetPosition(68);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);
        if(isStopRequested()){
            return;
        }

        sleep(1000);

        turret.setPivotTargetPosition(100);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);
        if(isStopRequested()){
            return;
        }

        sleep(1000);


        intake.spinIntake(-200);
        if(isStopRequested()){
            return;
        }

        sleep(2000);

        intake.spinIntake(0);
        if(isStopRequested()){
            return;
        }

        turret.setPivotTargetPosition(0);
        if(isStopRequested()){
            return;
        }

        sleep(1000);

        turret.setUpDownTargetPosition(32);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);
        if(isStopRequested()){
            return;
        }

        sleep(1000);

        f2.setStartSpeed(1);
        f2.setCruiseSpeed(1);
        f2.setCrawlSpeed(1);

        f2.driveToPoint(new Vector2(0,1200));
        if(isStopRequested()){
            return;
        }

        turret.setUpDownTargetPosition(0);
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.7);

        sleep(1000);
    }
}