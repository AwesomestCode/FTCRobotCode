package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Position Demo", group="Concept")
public class MotorPositionDemo extends LinearOpMode {
    int multiplier = 10;
    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "frontLeft");
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean xButtonWasPressed = false;
        boolean yButtonWasPressed = false;
        boolean bButtonWasPressed = false;
        boolean aButtonWasPressed = false;

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Position", position);
            telemetry.addData("Multiplier", multiplier);
            telemetry.addData("Encoder Position", motor.getCurrentPosition());
            if(!yButtonWasPressed && gamepad1.y) {
                multiplier *= 10;
            }
            if(!aButtonWasPressed && gamepad1.a) {
                multiplier *= 0.1;
            }
            if(!xButtonWasPressed && gamepad1.x) {
                position += multiplier;
                motor.setPower(1.0);
            }
            if(!bButtonWasPressed && gamepad1.b) {
                position -= multiplier;
                motor.setPower(0.5);
            }
            yButtonWasPressed = gamepad1.y;
            aButtonWasPressed = gamepad1.a;
            xButtonWasPressed = gamepad1.x;
            bButtonWasPressed = gamepad1.b;
            telemetry.update();
            motor.setTargetPosition(position);
            sleep(100);
        }
    }
}
