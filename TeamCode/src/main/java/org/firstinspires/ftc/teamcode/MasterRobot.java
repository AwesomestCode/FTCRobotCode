package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Master Robot 1.2", group="Concept")
public class MasterRobot extends LinearOpMode {
    int speed = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap, HardwareRobot.Mode.TELEOP);
        CRServo input = hardwareMap.get(CRServo.class, "input");

        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "linSlide");
        //Servo input = hardwareMap.get(Servo.class, "input");



        waitForStart();





        while(opModeIsActive()) {

            //drivetrain
            double divider=10;
            if (gamepad1.left_trigger!=0) {
                divider+= gamepad1.left_trigger * 10;
                telemetry.addData("Multiplier 1", divider);
                robot.movePower(new HardwareRobot.PowerVector(Math.min(gamepad1.left_stick_y, 0.8) / divider, Math.min(gamepad1.left_stick_x, 0.8) / divider, gamepad1.right_stick_x / divider));
            }else if (gamepad1.right_trigger != 0) {
                divider -= gamepad1.right_trigger*7;
                telemetry.addData("Multiplier 2", divider);
                robot.movePower(new HardwareRobot.PowerVector(Math.min(gamepad1.left_stick_y, 0.8)/divider, Math.min(gamepad1.left_stick_x, 0.8)/divider, gamepad1.right_stick_x/divider));
            }else {
                robot.movePower(new HardwareRobot.PowerVector(Math.min(gamepad1.left_stick_y, 0.8), Math.min(gamepad1.left_stick_x, 0.8), gamepad1.right_stick_x));
            }

            telemetry.addData("Multiplier", gamepad1.right_trigger);
            telemetry.addData("Speed", speed);

            //linear slide

            linearSlide.setPower(gamepad2.left_stick_y);

            input.setPower(gamepad2.right_stick_y);

            telemetry.addData("servo power", input.getPower());



            //input

         //   double position = input.getPosition();

          //  position+=gamepad2.right_stick_y;

         //   input.setPosition(position);



         //   telemetry.addData("servo position", input.getPosition());
            telemetry.update();








        }
    }
}
