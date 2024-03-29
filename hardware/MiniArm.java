package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

//linear range: 1000 to 1900
//claw: 500 to 2500
//turn: 885 to 2010
public class MiniArm {

    public static final double TURN_ALL_THE_WAY_BACK = MathUtilities.map(610, 500, 2500, 0, 1),
            TURN_STOWED = MathUtilities.map(860, 500, 2500, 0,1),
            TURN_UP = MathUtilities.map(950, 500, 2500, 0, 1),//all were 700 - 2010 2/11/2022
            TURN_STRAIGHT_OUT = MathUtilities.map(1330, 500, 2500, 0,1),
            TURN_DOWN = MathUtilities.map(1610, 500, 2500, 0, 1),//not 1512!
            TURN_STRAIGHT_DOWN = MathUtilities.map(1930, 500, 2500, 0,1),
            LIN_SERVO_RETRACTED = MathUtilities.map(1100, 1000, 1900, 0,1),
            LIN_SERVO_LOWER_RETRACTED = MathUtilities.map(1300, 1000, 1900, 0,1),
            LIN_SERVO_PLACE_CAPSTONE = MathUtilities.map(1210,1000,1900, 0,1 ),
            LIN_SERVO_MIDDLE = MathUtilities.map(1650, 1000, 1900, 0,1),
            LIN_SERVO_LOW = MathUtilities.map(1760, 1000, 1900, 0, 1),
            LIN_SERVO_EXTENDED = MathUtilities.map(1880, 1000, 1900, 0,1),
            CLAW_OPEN = MathUtilities.map(800, 500, 2500, 0,1),
            CLAW_CLOSED_BLOCK = MathUtilities.map(1350, 500, 2500, 0,1),
            CLAW_CLOSED_TSE = MathUtilities.map(1550, 500, 2500, 0,1);


    @Hardware
    public ServoImplEx claw, turn, linServoUp, linServoDown;

    public MiniArm(HardwareMap hardwareMap){
        Realizer.realize(this, hardwareMap);

        claw.setPwmRange(new PwmControl.PwmRange(500,2500));
        turn.setPwmRange(new PwmControl.PwmRange(500,2500));//700, 2010

        linServoDown.setPwmRange(new PwmControl.PwmRange(1000, 1880));
        linServoUp.setPwmRange(new PwmControl.PwmRange(1000, 1880));
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

    public void lowLinServos(){
        setLinServoPosition(LIN_SERVO_LOW);
    }
    public void linServoPlaceCapstone(){
        setLinServoPosition(LIN_SERVO_PLACE_CAPSTONE);
    }

    public void stowTurn(){
        setTurnPosition(TURN_STOWED);
    }

    public void turnAllTheWayBack(){
        setTurnPosition(TURN_ALL_THE_WAY_BACK);
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
