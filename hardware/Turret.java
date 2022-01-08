package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.BooleanEdgeDetector;

public class Turret{

    public boolean pivotHomed, upDownHomed;

    private double pivotHomeOffset, upDownHomeOffset;

    public int pivotLowerLimit = -215, pivotUpperLimit = 215, upDownLowerLimit = -7, upDownUpperLimit = 90;

    private BooleanEdgeDetector pivotBED, upDownBED;

    @Hardware
    public TouchSensor pivotHomingSensor, upDownHomingSensor;

    @Hardware
    public DcMotorEx pivot, upDown, intake;

    boolean encoderInversion, motorInversion;
    double ret;
    int ticksPerRevolution;

    //pivot positive edge in degrees: 10, 9.18, 9.09, 7.9, 8.46, 8.46, 8.73, 8.6 average as 8.8
    //pivot negative edge in degrees:

    static final double pivotNegativeEdge = -11, pivotPositiveEdge = 8.8;//these need changed
    static final double stop = 6;//this also needs changed

    public static final int[] HOME = {0,0};//pivot first, then upDown
    public static final int[] TOP_PLATE = {};
    public static final int[] MIDDLE_PLATE = {};
    public static final int[] BOTTOM_PLATE = {};
    public static final int[] SHARED_HUB = {};

    public static final double ticksPerDegreePivot = ((7 * 4 * 51*(28/10.0))/(360));
    public static final double ticksPerDegreeUpDown = ((7 * 4 * 188)/360.0);

    public Turret(HardwareMap hardwareMap){
        Realizer.realize(this, hardwareMap);

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        upDown.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotBED = new BooleanEdgeDetector(pivotHomingSensor.isPressed());
        upDownBED = new BooleanEdgeDetector(upDownHomingSensor.isPressed());

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public enum AngleUnit{
        DEGREES,
        RADIANS
    }

    public void setEncoderInversion(boolean encInv) {
        encInv = encoderInversion;
    }

    public void setMotorInversion(boolean motorInv) {
        motorInv = motorInversion;
    }

    public double tickstoAngle(int ticks, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES) {
            return ((ticks) * (360 / ticksPerRevolution));
        }

        if (angleUnit == AngleUnit.RADIANS) {
            return ((ticks) * ((2 * Math.PI) / ticksPerRevolution));
        }

        return ret;
    }

    public int degreesToTicks(double degrees){
        return (int) (ticksPerDegreePivot * degrees + pivotHomeOffset);
    }

    public void setTicksPerRevolution(int ticksPerRev){
        ticksPerRev = ticksPerRevolution;
    }

    public void setPivotPowerManual(double power){
        if(pivotHomed){
            //Inner left
            if(getPivotPosition() > 10 && getPivotPosition() < 33 + 1 && getUpDownPosition() < 25 && upDownHomed){
                pivot.setPower(Math.min(power, 0));
            }
            //Outer left
            else if(getPivotPosition() > 33 - 1 && getPivotPosition() < 60 && getUpDownPosition() < 25 && upDownHomed){
                pivot.setPower(Math.max(power,0));
            }
            //Inner right
            else if(getPivotPosition() < -2 && getPivotPosition() > -26 - 1 && getUpDownPosition() < 25 && upDownHomed){
                pivot.setPower(Math.max(power,0));
            }
            //Outer right
            else if(getPivotPosition() < -26 + 1 && getPivotPosition() > -58 && getUpDownPosition() < 25 && upDownHomed){
                pivot.setPower(Math.min(power,0));
            } else if (getPivotPosition() < 10 && getPivotPosition() > -2 && getUpDownPosition() < 25 && upDownHomed){
                pivot.setPower(0);
            } else if((getPivotPosition() < pivotLowerLimit)){
                pivot.setPower(Math.max(power, 0));
            } else if((getPivotPosition() > pivotUpperLimit)){
                pivot.setPower(Math.min(power,0));
            } else {
                pivot.setPower(power);
            }
        } else {
            pivot.setPower(0.3 * power);
        }
    }

    //0 is directly in front, clockwise is negative, counterclockwise is positive
    public double getPivotPosition(){
        return (pivot.getCurrentPosition() - pivotHomeOffset)/ ticksPerDegreePivot;//returns degrees
    }

    public double getUpDownPosition(){
        return (upDown.getCurrentPosition() - upDownHomeOffset)/ ticksPerDegreeUpDown;//returns degrees
    }

    public double getUpDownVelocity(){   //returns degrees
        return upDown.getVelocity()/ticksPerDegreeUpDown;
    }

    public void spinIntake(double velocity){
        intake.setVelocity(velocity);
    }

    public void setTargetPosition(int [] positions, double pivotPower, double upDownPower){
        if (pivotHomed) {
            setPivotTargetPosition(degreesToTicks(positions[0]));
            setPivotMode(DcMotor.RunMode.RUN_TO_POSITION);
            setUpDownTargetPosition(degreesToTicks(positions[1]));
            setUpDownMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(pivotPower);
            upDown.setPower(upDownPower);
        }
    }

    public void setPivotMode(DcMotor.RunMode mode){
        pivot.setMode(mode);
    }

    public void setUpDownMode(DcMotor.RunMode mode){
        upDown.setMode(mode);
    }

    public void setPivotTargetPosition(double degrees){
        pivot.setTargetPosition(degreesToTicks(degrees));
    }

    public void setUpDownTargetPosition(double degrees){
        upDown.setTargetPosition((int) (upDownHomeOffset + ticksPerDegreeUpDown * degrees));
    }

    public void calculatePivotOffset(double positionInDegrees){
        pivotHomeOffset = pivot.getCurrentPosition() - (ticksPerDegreePivot * positionInDegrees);
        pivotHomed = true;
    }

    public void calculateUpDownOffset(double positionInDegrees){
        upDownHomeOffset = upDown.getCurrentPosition() - (ticksPerDegreeUpDown * positionInDegrees);
        upDownHomed = true;
    }

    public void setUpDownPowerManual(double power) {
        if (upDownHomed) {
            //Left
            if(getPivotPosition() < -2 && getPivotPosition() > -50 && getUpDownPosition() < 25){
                power = Math.max(power, 0);
            }
            //Right
            else if(getPivotPosition() > 3 && getPivotPosition() < 55 && getUpDownPosition() < 25){
                power = Math.max(power, 0);
            } else if ((upDownHomingSensor.isPressed())) {
                power = Math.max(power, 0);
            } else if ((getUpDownPosition() > upDownUpperLimit)) {
                power = Math.min(power, 0);
            } else {
                //upDown.setPower(power);
            }

            power += 0.5 * Math.cos(getUpDownPosition());
            upDown.setPower(power);
        } else {
            upDown.setPower(power);
        }

    }

    public void setUpDownPowerAutomatic(double power){
        if(upDown.getTargetPosition() < upDown.getCurrentPosition()){
            power *= 0.3;
        }
        upDown.setPower(power);
    }

    public void setPivotPowerAutomatic(double power){
        pivot.setPower(power);
    }

    public boolean isPivotHomed() {
        return pivotHomed;
    }

    public boolean isUpDownHomed(){
        return upDownHomed;
    }

    public void update(){
        pivotBED.update(pivotHomingSensor.isPressed());
        upDownBED.update(upDownHomingSensor.isPressed());

        if(!pivotHomed){
            if(pivotBED.isRising()){
                if(pivot.getVelocity() > 0){
                    calculatePivotOffset(pivotNegativeEdge);
                } else if (pivot.getVelocity() < 0){
                    calculatePivotOffset(pivotPositiveEdge);
                }
            } else if (pivotBED.isFalling()){
                if(pivot.getVelocity() > 0){
                    calculatePivotOffset(pivotPositiveEdge);
                } else if (pivot.getVelocity() < 0){
                    calculatePivotOffset(pivotNegativeEdge);
                }
            }
        }

        if(!upDownHomed){
            if(upDownBED.isChanging()){
                calculateUpDownOffset(stop);
            }
        }
    }
}
