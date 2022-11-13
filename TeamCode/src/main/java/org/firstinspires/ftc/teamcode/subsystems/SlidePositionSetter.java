package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlidePositionSetter {

    private DcMotorEx slide;
    private int position;
    private int offset;
    private double speed;

    public SlidePositionSetter(DcMotorEx slide, boolean reversed) {
        this.slide = slide;
        this.position = 0;
        this.offset = 0;
        this.speed = 1.0;
        updatePosition();
        this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(reversed) {
            this.slide.setDirection(DcMotor.Direction.REVERSE);
        }
        updatePower();
    }

    public SlidePositionSetter(DcMotorEx slide) {
        this(slide, false);
    }

    public void reset() {
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.position = 0;
        this.offset = 0;
        this.speed = 1.0;
        updatePosition();
        this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        updatePower();
    }

    private void updatePosition() {
        slide.setTargetPosition(position + offset);
    }

    private void updatePower() {
        slide.setPower(speed);
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        updatePower();
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

    public int getActualPosition() {
        return slide.getCurrentPosition();
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
