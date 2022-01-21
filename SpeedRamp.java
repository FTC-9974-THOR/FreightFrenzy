package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SpeedRamp{

    private double setpoint, current, acceleration;
    private final ElapsedTime timer;

    public SpeedRamp(double accel){
        acceleration = accel;
        timer = new ElapsedTime();
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getSetPoint(){
        return setpoint;
    }

    public double getAcceleration(){
        return acceleration;
    }

    public void setCurrent(double current){
        this.current = current;
    }

    public void setAcceleration(double acceleration){
        this.acceleration = acceleration;
    }

    public double getCurrent(){
        return current;
    }

    public double update(){
        double error = setpoint - current;
        double delta = Math.copySign(timer.seconds() * acceleration, error);//positive if setpoint > current, negative if setpoint < current
        timer.reset();

        if(Math.abs(delta) > Math.abs(error)){
            current = setpoint;
        } else {
            current += delta;
        }
        return current;
    }

}
