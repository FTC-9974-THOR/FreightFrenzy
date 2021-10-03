package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.control.TrapezoidalMotionProfile;
import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

//magic angle: 35.3 degrees (used to be 33.7 degrees)
@TeleOp(name = "Freight Frenzy", group = "Teleops")
public class FreightFrenzy extends OpMode {

    MecanumDrive md;

    private boolean lastAState;
    TrapezoidalMotionProfile motionProfile;

    @Override
    public void init() {
        md = new MecanumDrive(hardwareMap);

        md.setAxisInversion(true,false, true);

        /*imu = new IMUNavSource(hardwareMap);
        f2 = new Fusion2();*/

        //et = new ElapsedTime();

        motionProfile = new TrapezoidalMotionProfile(
                new TrapezoidalMotionProfile.Node(0,0),
                new TrapezoidalMotionProfile.Node(0.5, 0.3),
                new TrapezoidalMotionProfile.Node(1,1)
        );

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

        double t = Math.copySign(motionProfile.apply(Math.abs(gamepad1.left_stick_x)), -gamepad1.left_stick_x);
        md.drive(gamepad1.right_stick_y, -gamepad1.right_stick_x,t);

    }

    @Override
    public void stop(){

    }
}

