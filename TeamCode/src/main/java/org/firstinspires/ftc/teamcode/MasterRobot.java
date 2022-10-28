package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;

@TeleOp(name="Master Robot", group="Full Code")
public class MasterRobot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
                slideSystem.setPosition(4200);
            }
            if (!oldGamepad1.b && newGamepad1.b) {
                slideSystem.setPosition(3000);
            }
            if (!oldGamepad1.a && newGamepad1.a) {
                slideSystem.setPosition(0);
            }
            if (!oldGamepad1.y && newGamepad1.y) {
                slideSystem.setPosition(1000);
            }
            telemetry.addData("Target Position", slideSystem.getTargetPosition());
            telemetry.addData("Actual Position", slideSystem.getActualPosition());
            telemetry.update();
        }
    }
}
