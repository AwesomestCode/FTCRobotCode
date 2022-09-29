package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw Demo", group="Demo")
public class ClawDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        waitForStart();

        boolean clawState = false;

        while(opModeIsActive()) {
            if(gamepad1.x) {
                leftClaw.setPosition(-0.5);
                rightClaw.setPosition(-0.5);
                clawState = true;
            }
            if(gamepad1.b) {
                leftClaw.setPosition(0.0);
                rightClaw.setPosition(0.0);
                clawState = false;
            }
            telemetry.addData("Claw opened state", clawState);
            telemetry.addData("Left Claw State", leftClaw.getPosition());
            telemetry.addData("Right Claw State", rightClaw.getPosition());
            telemetry.update();
        }
    }
}
