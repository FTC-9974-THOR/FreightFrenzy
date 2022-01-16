package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

//linear range: 1000 to 1800
//claw and turn: 500 to 2500
public class MiniArm {

    public static final double TURN_STOWED = MathUtilities.map(500, 500, 2500, 0, 1),
            TURN_UP = MathUtilities.map(700, 500, 2500, 0, 1),
            TURN_STRAIGHT_OUT = MathUtilities.map(1200, 500, 2500, 0,1),
            TURN_DOWN = MathUtilities.map(2500, 500, 2500, 0, 1),
            TURN_STRAIGHT_DOWN = MathUtilities.map(2500, 500, 2500, 0,1),
            LIN_SERVO_RETRACTED = MathUtilities.map(1000, 1000, 1800, 0,1),
            LIN_SERVO_LOWER_RETRACTED = MathUtilities.map(1200, 1000, 1800, 0,1),
            LIN_SERVO_MIDDLE = MathUtilities.map(1600, 1000, 1800, 0,1),
            LIN_SERVO_EXTENDED = MathUtilities.map(1800, 1000, 1800, 0,1),
            CLAW_OPEN = MathUtilities.map(1270, 500, 2500, 0,1),
            CLAW_CLOSED_BLOCK = MathUtilities.map(1800, 500, 2500, 0,1),
            CLAW_CLOSED_TSE = MathUtilities.map(2200, 500, 2500, 0,1);


    @Hardware
    public ServoImplEx claw, turn, linServoUp, linServoDown;

    public MiniArm(HardwareMap hardwareMap){
        Realizer.realize(this, hardwareMap);

        claw.setPwmRange(new PwmControl.PwmRange(500,2500));
        turn.setPwmRange(new PwmControl.PwmRange(500,2500));

        linServoDown.setPwmRange(new PwmControl.PwmRange(1000, 1800));
        linServoUp.setPwmRange(new PwmControl.PwmRange(1000, 1800));
    }

    public void setClawPosition(double handPosition){
        claw.setPosition(handPosition);
    }

    public void setTurnPosition(double armPosition){
        turn.setPosition(armPosition);
    }

    public void setLinServoPosition(double linServoPosition){
        linServoUp.setPosition(linServoPosition);
        linServoDown.setPosition(linServoPosition);
    }


    public void closeClawTSE(){
        setClawPosition(CLAW_CLOSED_TSE);
    }

    public void closeClawBlock(){
        setClawPosition(CLAW_CLOSED_BLOCK);
    }

    public void openClaw() {
        setClawPosition(CLAW_OPEN);
    }


    public void fullyRetractLinServos(){
        setLinServoPosition(LIN_SERVO_RETRACTED);
    }

    public void fullyExtendLinServos(){
        setLinServoPosition(LIN_SERVO_EXTENDED);
    }

    public void lowerRetractLinServos(){
        setLinServoPosition(LIN_SERVO_LOWER_RETRACTED);
    }

    public void middleLinServos(){
        setLinServoPosition(LIN_SERVO_MIDDLE);
    }


    public void stowTurn(){
        setTurnPosition(TURN_STOWED);
    }

    public void upTurn(){
        setTurnPosition(TURN_UP);
    }

    public void straightOutTurn(){
        setTurnPosition(TURN_STRAIGHT_OUT);
    }

    public void downTurn(){
        setTurnPosition(TURN_DOWN);
    }

    public void straightDownTurn(){
        setTurnPosition(TURN_STRAIGHT_DOWN);
    }
}
