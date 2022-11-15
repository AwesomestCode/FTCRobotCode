package org.firstinspires.ftc.teamcode.autolib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ActuallyMoveTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d vel = new Pose2d(0, 0, 0);
        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                vel = new Pose2d(-0.5, 0, 0);
            } if(gamepad1.cross) {
                vel = new Pose2d(0.5, 0, 0);
            } if(gamepad1.square) {
                vel = new Pose2d(0, -0.5, 0);
            } if(gamepad1.circle) {
                vel = new Pose2d(0, 0.5, 0);
            } if(gamepad1.dpad_right) {
                vel = new Pose2d(0, 0, 0.5);
            } if(gamepad1.dpad_left) {
                vel = new Pose2d(0, 0, -0.5);
            } if(gamepad1.touchpad) {
                vel = new Pose2d(0, 0, 0);
            }
            drive.setDrivePower(vel);
            drive.update();
        }
    }
}
