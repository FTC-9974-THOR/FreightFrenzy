package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class CarouselSpinner{

    @Hardware
    public DcMotorEx spinner;

    public CarouselSpinner(HardwareMap hm){
        Realizer.realize(this, hm);
    }

    public void spin(double power){
        spinner.setPower(power);
    }
}
