package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;
import org.firstinspires.ftc.teamcode.subsystems.TapePositionSensor;

@TeleOp(group="Demo")
public class AlignmentDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(hardwareMap);

        TapePositionSensor sensor = new TapePositionSensor(hardwareMap, TapePositionSensor.TapeColour.RED);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Position", sensor.getPosition());

            if (gamepad1.touchpad) {
                telemetry.addData("Touchpad", "Pressed");
                telemetry.addData("Suggested Power", sensor.getSuggestedPower());
                mixer.setMovement(sensor.getSuggestedPower(), 0, 0);

            } else {
                telemetry.addData("Touchpad", "Not Pressed");
                mixer.setMovement(0, 0,0);
                sensor.getPidController().resetPIDState();
            }

            if(gamepad1.circle) {
                sensor.setColour(TapePositionSensor.TapeColour.RED);
            }

            if(gamepad1.cross) {
                sensor.setColour(TapePositionSensor.TapeColour.BLUE);
            }

            if(gamepad1.right_bumper) {
                sensor.getPidController().resetPIDState();
            }

            telemetry.update();
            Thread.sleep(10);
        }
    }
}
