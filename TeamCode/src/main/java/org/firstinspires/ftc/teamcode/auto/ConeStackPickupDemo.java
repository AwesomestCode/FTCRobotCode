package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;

@Autonomous(group="Demos")
public class ConeStackPickupDemo extends LinearOpMode {
    static int ZONE = 3;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.speak("Initialising. Please load cones");
        CRServoImplEx intake = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        telemetry.speak("Stand back drivers. I have commenced the demo.");

        SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide1"), hardwareMap.get(DcMotorEx.class, "linearSlide2"), 20, true);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degree
        Pose2d startPose = new Pose2d(36, -62.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence clearSignal = drive.trajectorySequenceBuilder(startPose)
                .forward(20 + 12)
                .back(12)
                .forward(0.01)
                .build();

        TrajectorySequence getToJunction = drive.trajectorySequenceBuilder(clearSignal.end())
                .splineTo(new Vector2d(12 - 8, -36 + 8), Math.toRadians(AutoConstants.FIRST_JUNCTION_ROT))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence goToStack1 = drive.trajectorySequenceBuilder(getToJunction.end())
                .back(8)
                .lineToLinearHeading(new Pose2d(16, -14, Math.toRadians(30)))
                .splineTo(new Vector2d(60, -12), Math.toRadians(0))
                //.lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(0)))
                //.splineTo(new Vector2d(60, -12), Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();

        TrajectorySequence goToJunction2 = drive.trajectorySequenceBuilder(goToStack1.end())
                .lineToLinearHeading(new Pose2d(40, -12, Math.toRadians(180)))
                .splineTo(new Vector2d(36 - 8, -12 + 8), Math.toRadians(130))
                .waitSeconds(0.5)
                .build();

        ZONE = org.firstinspires.ftc.teamcode.auto.DetectAprilTagZoneUtil.getZone(hardwareMap, telemetry);

        telemetry.speak("I have detected the zone as " + ZONE);


        Pose2d parkStart = new Pose2d(0, 0, Math.toRadians(90));

        TrajectorySequence goLeft = drive.trajectorySequenceBuilder(parkStart)
                .strafeLeft(24)
                .build();

        TrajectorySequence goRight = drive.trajectorySequenceBuilder(parkStart)
                .strafeRight(24)
                .build();

        intake.setPower(0.5);
        drive.followTrajectorySequence(clearSignal);
        intake.setPower(0.3);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
        drive.followTrajectorySequence(getToJunction);
        slideSystem.setPosition(SlidePositions.TOP.getPosition() - 600);
        sleep(500);
        intake.setPower(-0.3);
        sleep(1000);
        //drive.followTrajectorySequence(returnToOrigin);
        drive.followTrajectorySequence(goToStack1);
        slideSystem.setPosition(600);
        intake.setPower(0.5);
        sleep(2000);
        intake.setPower(0.1);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());

        //drive.setPoseEstimate(parkStart);

        //if(ZONE == 1) {
        //    drive.followTrajectorySequence(goLeft);
        //} else if(ZONE == 3) {
        //    drive.followTrajectorySequence(goRight);
        //}

        //drive.followTrajectorySequence(goToStack1);
        //slideSystem.setPosition(100);
        //sleep(1000);
        //drive.followTrajectorySequence(goToJunction2);


    }
}
