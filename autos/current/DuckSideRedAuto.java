package org.firstinspires.ftc.teamcode.autos.current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RedVisionPipeline2;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
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

@Autonomous(name = "Duck Side Red Auto", group = "autonomous")
public class DuckSideRedAuto extends LinearOpMode{

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;
    private CarouselSpinner cs;
    private Turret turret;
    private MiniArm miniArm;

    public ElapsedTime et;

    RedVisionPipeline2 pipeline;
    RedVisionPipeline2.DuckPosition position;

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
        pipeline = new RedVisionPipeline2();
        webcam.setPipeline(pipeline);

        et = new ElapsedTime();

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(21, 96);
        navSource = new IMUNavSource(hardwareMap, 2);
        f2 = new Fusion2(this, md, calculator, navSource,new PIDFCoefficients(0.8,0,0,0), et);
        cs = new CarouselSpinner(hardwareMap);
        turret = new Turret(hardwareMap);
        miniArm = new MiniArm(hardwareMap);

        //the speed the robot will start moving at
        f2.setStartSpeed(0.7);
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

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Pipeline Has Analysis", pipeline.hasAnalysis());

            if (pipeline.hasAnalysis()) {
                telemetry.addData("Duck Position", pipeline.getAnalysis());
            }

            telemetry.addData("UpDown PID Coefficients:", turret.upDown.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addLine("STARTING POSITION: eyes AWAY FROM warehouse, side against wall, back wheels three inches in front of teeth");
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

        turret.setUpDownTargetPosition(20);//straight up
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.5);
        if(isStopRequested()){
            return;
        }


        switch(position){
            case LEFT:

                miniArm.downTurn();
                miniArm.lowLinServos();

                f2.driveToPoint(new Vector2(400, 200));
                if(isStopRequested()){
                    return;
                }

                f2.turnToHeading(Math.toRadians(35));
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

                f2.setStartSpeed(0.2);
                f2.setCruiseSpeed(0.3);
                f2.setCrawlSpeed(0.2);

                f2.driveToPoint(new Vector2(0, -400));
                if(isStopRequested()){
                    return;
                }

                miniArm.openClaw();

                sleep(500);

                f2.driveToPoint(new Vector2(0, 300));
                if(isStopRequested()){
                    return;
                }

                miniArm.stowTurn();
                miniArm.middleLinServos();

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

                f2.turnToHeading(Math.toRadians(-90));
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(0.7);
                f2.setCruiseSpeed(1);
                f2.setCrawlSpeed(0.4);

                f2.driveToPoint(new Vector2(-900, 0), null, 3500);
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(0.2);
                f2.setCruiseSpeed(0.3);
                f2.setCrawlSpeed(0.2);

                f2.driveToPoint(new Vector2(0, -200));
                if(isStopRequested()){
                    return;
                }

                cs.spinner.setPower(-0.55);

                sleep(3000);

                cs.spinner.setPower(0);

                f2.driveToPoint(new Vector2(0, 400));
                if(isStopRequested()){
                    return;
                }

                turret.setUpDownTargetPosition(0);//straight up
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                if(isStopRequested()){
                    return;
                }

                sleep(1500);

                break;
            case CENTER:

                miniArm.straightOutTurn();
                miniArm.middleLinServos();

                f2.driveToPoint(new Vector2(200, 0));
                if(isStopRequested()){
                    return;
                }

                f2.turnToHeading(Math.toRadians(50));
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

                f2.setStartSpeed(0.2);
                f2.setCruiseSpeed(0.3);
                f2.setCrawlSpeed(0.2);

                f2.driveToPoint(new Vector2(0, -300));
                if(isStopRequested()){
                    return;
                }

                miniArm.openClaw();

                sleep(500);

                f2.driveToPoint(new Vector2(0, 100));
                if(isStopRequested()){
                    return;
                }

                miniArm.stowTurn();
                miniArm.middleLinServos();

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

                f2.turnToHeading(Math.toRadians(-90), null, 2000);
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(0.7);
                f2.setCruiseSpeed(1);
                f2.setCrawlSpeed(0.4);

                f2.driveToPoint(new Vector2(-1100, 200), null, 3500);
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(0.2);
                f2.setCruiseSpeed(0.3);
                f2.setCrawlSpeed(0.2);

                f2.driveToPoint(new Vector2(0, -500));
                if(isStopRequested()){
                    return;
                }

                cs.spinner.setPower(-0.55);

                sleep(3000);

                cs.spinner.setPower(0);

                f2.driveToPoint(new Vector2(0, 350));
                if(isStopRequested()){
                    return;
                }

                turret.setUpDownTargetPosition(0);//straight up
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.5);
                if(isStopRequested()){
                    return;
                }

                sleep(1500);

                break;
            case RIGHT:

                f2.driveToPoint(new Vector2(0, 300));
                if(isStopRequested()){
                    return;
                }

                //1. drive to the alliance hub
                f2.driveToPoint(new Vector2(1100, 0));
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

                f2.driveToPoint(new Vector2(0, -250));
                if(isStopRequested()){
                    return;
                }

                miniArm.openClaw();

                sleep(500);

                miniArm.stowTurn();
                miniArm.middleLinServos();

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

                f2.driveToPoint(new Vector2(0, 300));
                if(isStopRequested()){
                    return;
                }

                f2.turnToHeading(Math.toRadians(-90));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(-550, 0), null, 3500);
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, -500));
                if(isStopRequested()){
                    return;
                }

                f2.setStartSpeed(0.2);
                f2.setCruiseSpeed(0.3);
                f2.setCrawlSpeed(0.2);

                f2.driveToPoint(new Vector2(0, -200));
                if(isStopRequested()){
                    return;
                }

                //spins the ducks off
                cs.spinner.setPower(-0.55);

                sleep(3000);

                cs.spinner.setPower(0);

                f2.driveToPoint(new Vector2(0, 350));
                if(isStopRequested()){
                    return;
                }

                turret.setUpDownTargetPosition(0);
                turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setUpDownPowerAutomatic(0.7);
                if(isStopRequested()){
                    return;
                }

                sleep(2000);

                break;
        }
    }
}