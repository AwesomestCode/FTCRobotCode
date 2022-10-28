package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

@TeleOp(name="Full Demo", group="Demo")
public class FullDemo extends LinearOpMode {
    int multiplier = 10;
    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");

        frontLeft.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        frontRight.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        rearLeft.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        rearRight.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);

        double initialHeading = -imu.getAngularOrientation().firstAngle;

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "linearSlide");
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean xButtonWasPressed = false;
        boolean yButtonWasPressed = false;
        boolean bButtonWasPressed = false;
        boolean aButtonWasPressed = false;
        CRServoImplEx intake = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        if (isStopRequested()) return;

        double totalCurrent = 0;

        while (opModeIsActive()) {
            //IF THIS CODE ENDS UP IN PRODUCTION RAYMOND IS GOING TO BE VERY IRRITATED. THIS IS TESTING CODE THAT SHOULD **NEVER** BE USED AT A COMPETITION.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //double botHeading = -imu.getAngularOrientation().firstAngle - initialHeading;
            double botHeading = 0.0;

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



            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                telemetry.addData(hub.getDeviceName() + " Input Voltage", hub.getInputVoltage(VoltageUnit.VOLTS));
                telemetry.addData(hub.getDeviceName() + " Current", hub.getCurrent(CurrentUnit.AMPS));
            }

            telemetry.update();

            telemetry.addData("Position", position);
            telemetry.addData("Multiplier", multiplier);
            telemetry.addData("Encoder Position", slideMotor.getCurrentPosition());
            if(!yButtonWasPressed && gamepad1.y) {
                multiplier *= 10;
            }
            if(!aButtonWasPressed && gamepad1.a) {
                multiplier *= 0.1;
            }
            if(!xButtonWasPressed && gamepad1.x) {
                position += multiplier;
                slideMotor.setPower(1.0);
            }
            if(!bButtonWasPressed && gamepad1.b) {
                position -= multiplier;
                slideMotor.setPower(0.5);
            }
            yButtonWasPressed = gamepad1.y;
            aButtonWasPressed = gamepad1.a;
            xButtonWasPressed = gamepad1.x;
            bButtonWasPressed = gamepad1.b;
            telemetry.update();
            slideMotor.setTargetPosition(position);
            sleep(100);

            if(gamepad1.dpad_up) {
                intake.setPower(-1.0);
            } else if(gamepad1.dpad_down) {
                intake.setPower(1.0);
            } else if(gamepad1.dpad_left) {
                intake.setPower(0.0);
            }
            // if toggle is false, set power to 0
            // toggle = !toggle
            sleep(5);
        }
    }
}
