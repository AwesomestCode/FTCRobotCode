package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;

@TeleOp(name="Motor Position Demo", group="Concept")
public class MotorPositionDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int multiplier = 1;
        SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        waitForStart();

        Gamepad oldGamepad1 = gamepad1;
        Gamepad oldGamepad2 = gamepad2;

        while(opModeIsActive()) {
            if(!oldGamepad1.x && gamepad1.x) {
                slideSystem.decrementPosition(100);
            }
            if(!oldGamepad1.b && gamepad1.b) {
                slideSystem.incrementPosition(100);
            }
            if(!oldGamepad1.a && gamepad1.a) {
                slideSystem.decrementPosition(10);
            }
            if(!oldGamepad1.y && gamepad1.y) {
                slideSystem.incrementPosition(10);
            }
            sleep(10);
        }
    }
}
