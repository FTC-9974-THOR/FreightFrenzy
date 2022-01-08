package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SpeedRamp;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class CarouselSpinner{

    @Hardware
    public DcMotorEx spinner;

    private final SpeedRamp speedRamp;

    public CarouselSpinner(HardwareMap hm){
        Realizer.realize(this, hm);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speedRamp = new SpeedRamp(400);
    }

    public void spin(double speed){
        speedRamp.setSetpoint(speed);
    }

    public double getCurrentSpeed(){
        return speedRamp.getCurrent();
    }

    public void stop(){
        speedRamp.setSetpoint(0);
        speedRamp.setCurrent(0);
    }

    public void update(){
        spinner.setVelocity(speedRamp.update() * 7 * 4 * 20 / 60);//7 poles, 4 edges, 20:1 gear ratio, 60 seconds per minute
    }
}
