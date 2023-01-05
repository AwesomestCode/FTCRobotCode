package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.TapePositionSensor;

@TeleOp(group="Demo")
public class AlignmentDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TapePositionSensor sensor = new TapePositionSensor(hardwareMap, TapePositionSensor.TapeColour.RED);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Position", sensor.getPosition());

            if (gamepad1.touchpad) {
                telemetry.addData("Touchpad", "Pressed");
            } else {
                telemetry.addData("Touchpad", "Not Pressed");
            }

            telemetry.update();
            Thread.sleep(10);
        }
    }
}
