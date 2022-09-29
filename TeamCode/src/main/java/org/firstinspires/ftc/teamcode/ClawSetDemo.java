package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Motor Ramp Demo", group="Concept")
public class ClawSetDemo extends LinearOpMode {
    double leftClawPos = 0.0;
    double rightClawPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        boolean xButtonWasPressed = false; // left
        boolean yButtonWasPressed = false; // up
        boolean bButtonWasPressed = false; // right
        boolean aButtonWasPressed = false; // down

        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        boolean leftWasPressed = false;
        boolean upWasPressed = false;
        boolean rightWasPressed = false;
        boolean downWasPressed = false;

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Left Claw", leftClawPos);
            telemetry.addData("Right Claw", rightClawPos);

            if(!yButtonWasPressed && gamepad1.y) {
                rightClawPos += 0.5;
            }
            if(!aButtonWasPressed && gamepad1.a) {
                rightClawPos -= 0.5;
            }
            if(!xButtonWasPressed && gamepad1.x) {
                rightClawPos -= 0.1;
            }
            if(!bButtonWasPressed && gamepad1.b) {
                rightClawPos += 0.1;
            }
            yButtonWasPressed = gamepad1.y;
            aButtonWasPressed = gamepad1.a;
            xButtonWasPressed = gamepad1.x;
            bButtonWasPressed = gamepad1.b;

            if(!upWasPressed && gamepad1.dpad_up) {
                rightClawPos += 0.5;
            }
            if(!downWasPressed && gamepad1.dpad_down) {
                rightClawPos -= 0.5;
            }
            if(!leftWasPressed && gamepad1.dpad_left) {
                rightClawPos -= 0.1;
            }
            if(!rightWasPressed && gamepad1.dpad_right) {
                rightClawPos += 0.1;
            }
            upWasPressed = gamepad1.dpad_up;
            downWasPressed = gamepad1.dpad_down;
            leftWasPressed = gamepad1.dpad_left;
            rightWasPressed = gamepad1.dpad_right;


            telemetry.update();
            leftClaw.setPosition(leftClawPos);
            rightClaw.setPosition(rightClawPos);
            sleep(100);
        }
    }
}
