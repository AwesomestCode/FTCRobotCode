package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.JunctionPositionSensor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;

@TeleOp(group="Demo")
public class JunctionAlignmentDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        JunctionPositionSensor sensor = new JunctionPositionSensor(hardwareMap);

        MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            int estimate = sensor.getEstimate();
            telemetry.addData("Estimate", estimate);
            telemetry.addData("Left Sensor CM", sensor.getLeftSensorRaw());
            telemetry.addData("Centre Sensor CM", sensor.getCentreSensorRaw());
            telemetry.addData("Right Sensor CM", sensor.getRightSensorRaw());
            if(gamepad1.touchpad) {
                mixer.setMovement(0, 0, estimate * 0.4);
            } else {
                mixer.setMovement(0, 0, 0);
            }
            telemetry.update();
            Thread.sleep(10);
        }
    }
}
