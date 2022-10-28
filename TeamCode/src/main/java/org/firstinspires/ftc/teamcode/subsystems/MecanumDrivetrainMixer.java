package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrivetrainMixer {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double rearLeftPower = 0;
    private double rearRightPower = 0;

    public MecanumDrivetrainMixer(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMovement(double x, double y, double rotation, double heading) {
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        double maxPower = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);

        frontLeftPower = (rotY + rotX + rotation) / maxPower;
        frontRightPower = (rotY - rotX - rotation) / maxPower;
        rearLeftPower = (rotY - rotX + rotation) / maxPower;
        rearRightPower = (rotY + rotX - rotation) / maxPower;
    }

    public void setMovement(double x, double y, double rotation) {
        setMovement(x, y, rotation, 0);
    }

    private static class Vector2d {
        private final double x;
        private final double y;

        public Vector2d(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }
}
