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
import org.firstinspires.ftc.teamcode.subsystems.JunctionPositionSensor;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;

@Autonomous(group="Demos")
public class ConeStackDepoDemo extends LinearOpMode {
    SampleMecanumDrive drive;

    public void rotate(double rotation) {
        drive.setWeightedDrivePower(new Pose2d(0, 0, rotation));
    }

    static int ZONE = 3;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.speak("Initialising. Please load cones");
        JunctionPositionSensor sensor = new JunctionPositionSensor(hardwareMap);
        CRServoImplEx intake = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        telemetry.speak("Stand back drivers. I have commenced the demo.");

        SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide1"), hardwareMap.get(DcMotorEx.class, "linearSlide2"), 20, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        this.drive = drive;

        // We want to start the bot at x: 10, y: -8, heading: 90 degree
        Pose2d startPose = new Pose2d(36, -62.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence clearSignal = drive.trajectorySequenceBuilder(startPose)
                .forward(20 + 12)
                .back(12)
                .forward(0.01)
                .build();

        TrajectorySequence getToJunction = drive.trajectorySequenceBuilder(clearSignal.end())
                .splineTo(new Vector2d(12 - 9, -36 + 9), Math.toRadians(AutoConstants.FIRST_JUNCTION_ROT))
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

        ZONE = DetectAprilTagZoneUtil.getZone(hardwareMap, telemetry);

        telemetry.speak("I have detected the zone as " + ZONE);

        intake.setPower(0.5);
        drive.followTrajectorySequence(clearSignal);
        intake.setPower(0.3);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
        drive.followTrajectorySequence(getToJunction);
        sensor.align(this::rotate);
        telemetry.speak("Finished re-aligning");
        slideSystem.setPosition(SlidePositions.TOP.getPosition() - 200);
        sleep(250);
        intake.setPower(-0.3);
        sleep(1000);
        //drive.followTrajectorySequence(returnToOrigin);
        intake.setPower(0);
        slideSystem.setPosition(SlidePositions.LOW.getPosition() - 100);
        intake.setPower(0.3);
        drive.followTrajectorySequence(goToStack1);
        slideSystem.setPosition(SlidePositions.LOW.getPosition() - 300);
        sleep(250);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());


        //drive.setPoseEstimate(parkStart);

        if(ZONE == 1) {
            //drive.followTrajectorySequence(goLeft);
        } else if(ZONE == 3) {
            //drive.followTrajectorySequence(goRight);
        }

        //drive.followTrajectorySequence(goToStack1);
        //slideSystem.setPosition(100);
        //sleep(1000);
        //drive.followTrajectorySequence(goToJunction2);


    }
}
