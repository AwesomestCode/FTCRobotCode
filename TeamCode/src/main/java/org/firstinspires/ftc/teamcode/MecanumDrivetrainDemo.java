package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;

import java.util.List;

@TeleOp(name="Mecanum Demo", group="Demo")
public class MecanumDrivetrainDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");

        MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(frontLeft, frontRight, rearLeft, rearRight);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            mixer.setMovement(x, y, rx);

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
        }
    }
}
