package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@Disabled
@Autonomous(name = "Warehouse Park Near Blue", group = "autonomous")
public class WarehouseParkNearBlue extends LinearOpMode {

    private MecanumDrive md;
    private MecanumEncoderCalculator calculator;
    private IMUNavSource navSource;
    private Fusion2 f2;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        md = new MecanumDrive(hardwareMap);
        calculator = new MecanumEncoderCalculator(21, 96);
        navSource = new IMUNavSource(hardwareMap);
        f2 = new Fusion2(this, md, calculator, navSource,new PIDFCoefficients(0.8,0,0,0));

        //the speed the robot will start moving at
        f2.setStartSpeed(1);
        //how long it will take for the robot to reach "cruise speed"
        f2.setRampUpDistance(1);//was 100
        //the cruise speed (you can also think of this as max speed)
        f2.setCruiseSpeed(1);
        //how long it will take for the robot to slow down to crawl speed
        f2.setRampDownDistance(1);
        //how long the robot will "crawl", or move slowly
        f2.setCrawlDistance(1);
        //the speed at which the robot will crawl
        f2.setCrawlSpeed(0.2);

        if (isStopRequested()) {
            return;
        }

        md.setAxisInversion(false, false, false);
        md.setEncoderInversion(true, true, true, true);
        navSource.setInverted(false);

        waitForStart();

        f2.driveToPoint(new Vector2(0, -650));
        if (isStopRequested()) {
            return;
        }

        f2.driveToPoint(new Vector2(600, 0));
        if(isStopRequested()){
            return;
        }

        f2.driveToPoint(new Vector2(0, -400));
        if(isStopRequested()){
            return;
        }
    }
}
