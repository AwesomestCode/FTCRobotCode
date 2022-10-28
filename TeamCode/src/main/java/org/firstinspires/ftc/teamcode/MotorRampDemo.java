package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Ramp Demo", group="Concept")
@Disabled
public class MotorRampDemo extends LinearOpMode {
    int speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "frontLeft");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean xButtonWasPressed = false;
        boolean yButtonWasPressed = false;
        boolean bButtonWasPressed = false;
        boolean aButtonWasPressed = false;

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Speed", speed);
            if(!yButtonWasPressed && gamepad1.y) {
                speed += 10;
            }
            if(!aButtonWasPressed && gamepad1.a) {
                speed -= 10;
            }
            if(!xButtonWasPressed && gamepad1.x) {
                speed -= 1;
            }
            if(!bButtonWasPressed && gamepad1.b) {
                speed += 1;
            }
            yButtonWasPressed = gamepad1.y;
            aButtonWasPressed = gamepad1.a;
            xButtonWasPressed = gamepad1.x;
            bButtonWasPressed = gamepad1.b;
            telemetry.update();
            motor.setPower(((double) speed) / 100.0);
            sleep(100);
        }
    }
}
