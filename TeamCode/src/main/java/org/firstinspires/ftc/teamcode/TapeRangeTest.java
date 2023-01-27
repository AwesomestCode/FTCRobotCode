package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TapePositionSensor;

@TeleOp(group="Tests")
public class TapeRangeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TapePositionSensor tapeSensor = new TapePositionSensor(hardwareMap, TapePositionSensor.TapeColour.RED);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("In Range", tapeSensor.isInRange());
            telemetry.update();

            tapeSensor.align((double strafeAmount) -> {
                drive.setWeightedDrivePower(new Pose2d(0, -strafeAmount, 0));
                multipleTelemetry.addData("Strafe", strafeAmount);
                multipleTelemetry.update();
            }, multipleTelemetry);
        }
    }
}
