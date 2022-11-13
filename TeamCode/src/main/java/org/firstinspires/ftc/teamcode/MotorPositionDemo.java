package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;

@TeleOp(name="Motor Position Demo", group="Concept")
public class MotorPositionDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // int multiplier = 1;
        SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide"), true);

        waitForStart();

        Gamepad oldGamepad1 = new Gamepad();
        Gamepad newGamepad1 = new Gamepad();

        while(opModeIsActive()) {
            try {
                oldGamepad1.copy(newGamepad1);
                newGamepad1.copy(gamepad1);
                sleep(10);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            if (!oldGamepad1.x && newGamepad1.x) {
                slideSystem.decrementPosition(50);
            }
            if (!oldGamepad1.b && newGamepad1.b) {
                slideSystem.incrementPosition(50);
            }
            if (!oldGamepad1.a && newGamepad1.a) {
                slideSystem.decrementPosition(500);
            }
            if (!oldGamepad1.y && newGamepad1.y) {
                slideSystem.incrementPosition(500);
            }
            telemetry.addData("Target Position", slideSystem.getTargetPosition());
            telemetry.addData("Actual Position", slideSystem.getActualPosition());
            telemetry.update();
        }
    }
}
