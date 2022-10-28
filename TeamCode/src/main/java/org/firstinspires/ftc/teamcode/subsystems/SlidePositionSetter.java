package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlidePositionSetter {

    private DcMotorEx slide;
    private int position;
    private double speed;

    public SlidePositionSetter(DcMotorEx slide) {
        this.slide = slide;
        this.position = 0;
        this.speed = 1.0;
        updatePosition();
        this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void updatePosition() {
        slide.setTargetPosition(position);
    }

    private void updatePower() {
        slide.setPower(speed);
    }
    public int setPosition(int position) {
        this.position = position;
        updatePosition();
        return position;
    }

    public int getTargetPosition() {
        return position;
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
