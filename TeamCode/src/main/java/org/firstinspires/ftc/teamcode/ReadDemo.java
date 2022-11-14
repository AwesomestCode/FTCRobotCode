
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

@TeleOp(name="Deadwheel Read Demo", group="Demo")
public class ReadDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);

        double initialHeading = -imu.getAngularOrientation().firstAngle;

        waitForStart();

        int strafeStart = frontRight.getCurrentPosition();
        int leftStart = frontLeft.getCurrentPosition();
        int rightStart = rearLeft.getCurrentPosition();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle - initialHeading;

            telemetry.addData("First Angle", imu.getAngularOrientation().firstAngle);
            telemetry.addData("Second Angle", imu.getAngularOrientation().secondAngle);
            telemetry.addData("Third Angle", imu.getAngularOrientation().thirdAngle);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double maxPower = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            telemetry.addData("Max Power", maxPower);

            double frontLeftPower = (rotY + rotX + rx) / maxPower;
            double frontRightPower = (rotY - rotX - rx) / maxPower;
            double rearLeftPower = (rotY - rotX + rx) / maxPower;
            double rearRightPower = (rotY + rotX - rx) / maxPower;

            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Rear Left Power", rearLeftPower);
            telemetry.addData("Rear Right Power", rearRightPower);

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            rearLeft.setPower(rearLeftPower);
            rearRight.setPower(rearRightPower);

            telemetry.addData("Front Left Amperage", frontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Front Right Amperage", frontRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Rear Left Amperage", rearLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Rear Right Amperage", rearRight.getCurrent(CurrentUnit.AMPS));

            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

            telemetry.addData("Strafe", ((double) (strafeStart - frontRight.getCurrentPosition()) / 8192) * 35 * Math.PI);
            telemetry.addData("X1", ((double) (leftStart - frontLeft.getCurrentPosition()) / 8192) * 35 * Math.PI);
            telemetry.addData("X2", ((double) (rightStart - rearLeft.getCurrentPosition()) / 8192) * 35 * Math.PI);

            if(gamepad1.a) {
                strafeStart = frontRight.getCurrentPosition();
                leftStart = frontLeft.getCurrentPosition();
                rightStart = rearLeft.getCurrentPosition();
            }

            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                telemetry.addData(hub.getDeviceName() + " Input Voltage", hub.getInputVoltage(VoltageUnit.VOLTS));
                telemetry.addData(hub.getDeviceName() + " Current", hub.getCurrent(CurrentUnit.AMPS));
            }

            telemetry.update();
        }
    }
}
