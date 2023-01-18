package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;

@Config
public class MecanumDrivetrainMixer {

    public static boolean SHIM_AUTO = false;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double rearLeftPower = 0;
    private double rearRightPower = 0;

    SampleMecanumDrive mecanumDrive;

    public MecanumDrivetrainMixer(HardwareMap map) {
        frontLeft = (DcMotorEx) map.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) map.dcMotor.get("frontRight");
        backLeft = (DcMotorEx) map.dcMotor.get("rearLeft");
        backRight = (DcMotorEx) map.dcMotor.get("rearRight");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if(!SHIM_AUTO) { // only do this when we're not relying on autonomous, otherwise that handles it for us.

            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            mecanumDrive = new SampleMecanumDrive(map);
        }
    }

    private void updatePowers() {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(rearLeftPower);
        backRight.setPower(rearRightPower);
    }
    public void setMovement(double x, double y, double rotation, double heading) {
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        double maxPower = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);

        frontLeftPower = (rotY + rotX + rotation) / maxPower;
        frontRightPower = (rotY - rotX - rotation) / maxPower;
        rearLeftPower = (rotY - rotX + rotation) / maxPower;
        rearRightPower = (rotY + rotX - rotation) / maxPower;

        updatePowers();
    }

    public void setMovement(double x, double y, double rotation) {
        if(SHIM_AUTO) {
            mecanumDrive.setWeightedDrivePower(new Pose2d(y, -x, -rotation));

        } else {
            setMovement(x, y, rotation, 0);
        }
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
