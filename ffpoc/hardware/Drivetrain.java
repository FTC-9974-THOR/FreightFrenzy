package org.firstinspires.ftc.teamcode.ffpoc.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.navigation.Fusion2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

public class Drivetrain {

    private final MecanumDrive drive;
    private final IMUNavSource imu;

    // it's a *very* bad idea to have multiple instances of Fusion2 connected to the same drivetrain,
    // so cache the first one we create just in case constructFusion2() gets called multiple times.
    private Fusion2 fusion2;

    public Drivetrain(HardwareMap hw) {
        drive = new MecanumDrive(hw);
        drive.setEncoderInversion(true, true, false, false);

        imu = new IMUNavSource(hw);
        imu.setInverted(true);
    }

    public void drive(double x, double y, double rot) {
        drive.drive(x, y, rot);
    }

    public MecanumDrive getMecanumDriveObject() {
        return drive;
    }

    public Fusion2 constructFusion2(LinearOpMode root) {
        if (fusion2 == null) {
            fusion2 = new Fusion2(root, drive, new MecanumEncoderCalculator(27.4, 96), imu, new PIDFCoefficients(0.8, 0, 0, 0));
        }
        return fusion2;
    }
}
