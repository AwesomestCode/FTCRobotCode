package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TennisBot Demo", group="Bots")
@Disabled
public class RollerPushbot extends LinearOpMode {

    int speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //DcMotor rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            rearLeft.setPower(((double) speed) / 100.0);

            frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            frontRight.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
            sleep(100);
        }
    }
}
