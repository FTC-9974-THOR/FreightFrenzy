package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@Autonomous(name = "Duck Side Blue Auto", group = "autonomous")
public class DuckSideBlueAuto extends LinearOpMode{

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;
    private CarouselSpinner cs;
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException{

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(21, 96);
        navSource = new IMUNavSource(hardwareMap, 2);
        f2 = new Fusion2(this, md, calculator, navSource,new PIDFCoefficients(0.8,0,0,0));
        cs = new CarouselSpinner(hardwareMap);
        turret = new Turret(hardwareMap);

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
            telemetry.addData("Front Left Position", md.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", md.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position", md.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position", md.backRight.getCurrentPosition());
            telemetry.addData("Number of Targets: ", f2.numTargets);
            telemetry.addData("UpDown PID Coefficients:", turret.upDown.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addLine("STARTING POSITION: eyes towards warehouse, side against wall, back of front wheels on teeth between second and third tiles");
            telemetry.update();
        }
        if (isStopRequested()) return;
        //waitForStart();

        md.setAxisInversion(true, true, true);
        md.setEncoderInversion(false, false, false, false);//all true
        navSource.setInverted(false);

        //1. raise the arm
        turret.setUpDownTargetPosition(72);//high level
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.5);
        if(isStopRequested()){
            return;
        }
        //2. drive to the alliance hub
        f2.driveToPoint(new Vector2(1100, 0));
        if(isStopRequested()){
            return;
        }

        turret.spinIntake(-300);
        if(isStopRequested()){
            return;
        }

        sleep(2000);

        turret.spinIntake(0);
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(0,-800));
        if(isStopRequested()){
            return;
        }

        turret.setUpDownTargetPosition(0);//high level
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.5);
        if(isStopRequested()){
            return;
        }

        sleep(1500);

        f2.turnToHeading(Math.toRadians(-90));
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(0, -750));
        if(isStopRequested()){
            return;
        }

        cs.spin(0.75);

        sleep(3000);

        cs.spin(0);

        f2.driveToPoint(new Vector2(-200, 200));
        if(isStopRequested()){
            return;
        }

        f2.turnToHeading(Math.toRadians(-179));
        if(isStopRequested()){
            return;
        }

        turret.setUpDownTargetPosition(32);//high level
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.5);
        if(isStopRequested()){
            return;
        }

        sleep(9000);

        f2.driveToPoint(new Vector2(0, -2300));
        if(isStopRequested()){
            return;
        }

        turret.setUpDownTargetPosition(0);//high level
        turret.setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setUpDownPowerAutomatic(0.5);
        if(isStopRequested()){
            return;
        }

        sleep(1500);
    }
}




