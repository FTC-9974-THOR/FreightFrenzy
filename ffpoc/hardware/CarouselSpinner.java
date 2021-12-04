package org.firstinspires.ftc.teamcode.ffpoc.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class CarouselSpinner {

    @Hardware
    public DcMotor spinner;

    public CarouselSpinner(HardwareMap hw) {
        Realizer.realize(this, hw);
    }

    public void setWheelPower(double power) {
        spinner.setPower(power);
    }
}
