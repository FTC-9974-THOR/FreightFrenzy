package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SpeedRamp;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class CarouselSpinner{


    @Hardware
    public DcMotorEx spinner;

    private final SpeedRamp speedRamp;

    //static coefficient of friction may need to be changed, we guessed and said 0.4, radius is in METERS
    private final double linearAcceleration = 0.4 * 9.807, radius = 0.17145;
    private final double maxAngularAcceleration = linearAcceleration/radius;//radians/seconds squared
    private final double maxAngularVelocity = Math.sqrt(maxAngularAcceleration);//radians/seconds

    public CarouselSpinner(HardwareMap hm){
        Realizer.realize(this, hm);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speedRamp = new SpeedRamp(1);
    }

    public void spin(DcMotor.Direction direction){
        speedRamp.setSetpoint((direction == DcMotorSimple.Direction.REVERSE ? -1 : 1) * maxAngularVelocity);
    }

    public double getCurrentSpeed(){
        return speedRamp.getCurrent();
    }

    public void stop(){
        speedRamp.setSetpoint(0);
        speedRamp.setCurrent(0);
    }

    public void update(){

        //diameter of wheel is 96 mm, 20:1 gear ratio, diameter of carousel is 15 inches = 381mm
        double angularVelocity = (spinner.getVelocity(AngleUnit.RADIANS) * 96)/(20 * 381);

        double maxAngularAccelerationCarousel = Math.sqrt((Math.pow(linearAcceleration, 2)/Math.pow(radius,2)) - Math.pow(angularVelocity, 4));
        double maxAngularAccelerationMotor = (maxAngularAccelerationCarousel * 381 * 20)/96;

        if(Double.isNaN(maxAngularAccelerationMotor)){
            maxAngularAccelerationMotor = 0.1;
        }

        speedRamp.setAcceleration(maxAngularAccelerationMotor);

        spinner.setVelocity(speedRamp.update(), AngleUnit.RADIANS);
    }
}
