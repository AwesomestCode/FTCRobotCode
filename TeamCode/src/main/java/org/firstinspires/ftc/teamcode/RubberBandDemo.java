package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Rubber Band Demo", group="Demo")
@Disabled
public class RubberBandDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServoImplEx motor = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.dpad_up) {
                motor.setPower(-1.0);
            } else if(gamepad1.dpad_down) {
                motor.setPower(1.0);
            } else if(gamepad1.dpad_left) {
                motor.setPower(0.0);
            }
            // if toggle is false, set power to 0
            // toggle = !toggle
            sleep(5);
        }
    }
}
