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
        DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");

        MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(frontLeft, frontRight, rearLeft, rearRight);

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
            }

            telemetry.update();
            Thread.sleep(10);
        }
    }
}
