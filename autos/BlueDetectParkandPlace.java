package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RedVisionPipeline;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Detect Park and Place", group = "autonomous")
public class BlueDetectParkandPlace extends LinearOpMode {

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;

    private OpenCvCamera webcam;

    private Arm arm;
    private CarouselSpinner cs;

    RedVisionPipeline pipeline;
    RedVisionPipeline.DuckPosition position;

    ElapsedTime et;

    @Override
    public void runOpMode() throws InterruptedException {

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

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

        pipeline = new RedVisionPipeline();
        webcam.setPipeline(pipeline);

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(21, 96);
        navSource = new IMUNavSource(hardwareMap);
        f2 = new Fusion2(this, md, calculator, navSource,new PIDFCoefficients(0.8,0,0,0));

        arm = new Arm(hardwareMap);
        cs = new CarouselSpinner(hardwareMap);

        et = new ElapsedTime();

        //the speed the robot will start moving at
        f2.setStartSpeed(0.5);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(10);//was 100
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(1);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(10);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(30);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.3);

        f2.setMinTurningSpeed(0.4);

        //arm.intake.setVelocityPIDFCoefficients(5);

         /*double hMinimum = 30;
        double hMaximum = 50;//this was 29

        double sMinimum = 120;//this was 50
        double sMaximum = 200;*/

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Front Left Position", md.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", md.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position", md.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position", md.backRight.getCurrentPosition());
            telemetry.addData("Arm Position", arm.shoulder.getCurrentPosition());
            telemetry.addData("Duck Position", pipeline.getAnalysis());
            telemetry.update();
        }
        if (isStopRequested()) return;

        md.setAxisInversion(false, false, false);
        md.setEncoderInversion(true, true, true, true);
        navSource.setInverted(false);

        arm.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        et.reset();

        TimingUtilities.blockUntil(this, pipeline::hasAnalysis, null, null);
        if (isStopRequested()) return;

        position = pipeline.getAnalysis();

        arm.homed = true;

        switch (position) {
            case LEFT:

                f2.driveToPoint(new Vector2(0, -225));//was 250
                if (isStopRequested()) {
                    return;
                }

                arm.setTargetPosition(Arm.BOTTOM_PLATE);
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(180));
                if (isStopRequested()) {
                    return;
                }

                arm.spinIntake(-300);

                sleep(3500);

                arm.spinIntake(0);

                arm.setTargetPosition(Arm.HOME);
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(90));
                if (isStopRequested()) {
                    return;
                }

                f2.driveToPoint(new Vector2(0, 550));
                if (isStopRequested()) {
                    return;
                }


                f2.turnToHeading(Math.toRadians(30));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, 300));
                if (isStopRequested()) {
                    return;
                }

                cs.spin(0.5);

                sleep(5000);

                cs.spin(0);

                f2.turnToHeading(Math.toRadians(90));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, -1700));
                if (isStopRequested()) {
                    return;
                }

                break;

            case CENTER:

                f2.driveToPoint(new Vector2(0, -325));
                if (isStopRequested()) {
                    return;
                }

                arm.spinIntake(-300);

                sleep(3500);

                arm.spinIntake(0);

                f2.driveToPoint(new Vector2(0, 150));
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(90));
                if (isStopRequested()) {
                    return;
                }

                //drives to carousel
                f2.driveToPoint(new Vector2(0, 650));
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(30));
                if(isStopRequested()){
                    return;
                }

                //drives into carousel
                f2.driveToPoint(new Vector2(0, 300));//was -200,0
                if (isStopRequested()) {
                    return;
                }


                cs.spin(0.5);

                sleep(5000);

                cs.spin(0);

                f2.turnToHeading(Math.toRadians(90));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, -1700));
                if (isStopRequested()) {
                    return;
                }

                break;

            case RIGHT:

                f2.driveToPoint(new Vector2(0, -275));
                if (isStopRequested()) {
                    return;
                }

                arm.setTargetPosition(Arm.TOP_PLATE);
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(180));
                if (isStopRequested()) {
                    return;
                }

                f2.driveToPoint(new Vector2(0, 50));
                if (isStopRequested()) {
                    return;
                }

                arm.spinIntake(-350);

                sleep(3500);

                arm.spinIntake(0);

                f2.driveToPoint(new Vector2(0, -50));
                if (isStopRequested()) {
                    return;
                }

                arm.setTargetPosition(Arm.HOME);
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(90));
                if (isStopRequested()) {
                    return;
                }

                f2.driveToPoint(new Vector2(0, 550));
                if (isStopRequested()) {
                    return;
                }

                f2.turnToHeading(Math.toRadians(30));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, 300));
                if (isStopRequested()) {
                    return;
                }

                cs.spin(0.5);

                sleep(5000);

                cs.spin(0);

                f2.turnToHeading(Math.toRadians(90));
                if(isStopRequested()){
                    return;
                }

                f2.driveToPoint(new Vector2(0, -1700));
                if (isStopRequested()) {
                    return;
                }

                break;
        }
    }
}


