package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SlidePositionSetter {

    private DcMotorEx slide1;
    private DcMotorEx slide2;
    private int position;
    private int offset;
    private double speed;

    public SlidePositionSetter(DcMotorEx slide1, DcMotorEx slide2, double ampLimit, boolean reversed) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.position = 0;
        this.offset = 0;
        this.speed = 1.0;
        updatePosition();
        this.slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slide1.setCurrentAlert(ampLimit, CurrentUnit.AMPS);
        this.slide1.setTargetPositionTolerance(50);
        this.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slide2.setCurrentAlert(ampLimit, CurrentUnit.AMPS);
        this.slide2.setTargetPositionTolerance(50);

        if(reversed) {
            this.slide1.setDirection(DcMotor.Direction.REVERSE);
        } else {
            this.slide2.setDirection(DcMotor.Direction.REVERSE);
        }
        updatePower();
    }

    public SlidePositionSetter(DcMotorEx slide1, DcMotorEx slide2) {
        this(slide1, slide2, 4, false);
    }

    public void reset() {
        this.slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.position = 0;
        this.offset = 0;
        this.speed = 1.0;
        updatePosition();
        this.slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        updatePower();
    }

    private void updatePosition() {
        slide1.setTargetPosition(position + offset);
        slide2.setTargetPosition(position + offset);
    }

    private void updatePower() {
        slide1.setPower(speed);
        slide2.setPower(speed);
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        updatePower();
    }

    public boolean getAreMotorsOverCurrent() {
        return slide1.isOverCurrent() || slide2.isOverCurrent();
    }

    public double getMotor1Current() {
        return slide1.getCurrent(CurrentUnit.AMPS);
    }

    public double getMotor2Current() {
        return slide2.getCurrent(CurrentUnit.AMPS);
    }

    public int setPosition(int position) {
        this.position = position;
        updatePosition();
        return position;
    }

    public int getTargetPosition() {
        return position + offset;
    }

    public void setOffset(int offset) {
        this.offset = offset;
        updatePosition();
    }

    public int getOffset() {
        return offset;
    }

    public int getActualSlideMotor1Position() {
        return slide1.getCurrentPosition();
    }

    public int getActualSlideMotor2Position() {
        return slide2.getCurrentPosition();
    }

    public int getActualPosition() {
        return (getActualSlideMotor1Position() + getActualSlideMotor2Position()) / 2;
    }

    public void incrementPosition(int increment) {
        position += increment;
        updatePosition();
    }

    public void decrementPosition(int decrement) {
        position -= decrement;
        updatePosition();
    }

}
