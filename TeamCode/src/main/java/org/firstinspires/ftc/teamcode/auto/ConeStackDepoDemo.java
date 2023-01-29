package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.generators.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.subsystems.JunctionPositionSensor;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;
import org.firstinspires.ftc.teamcode.subsystems.TapePositionSensor;

@Autonomous(group="Demos")
public class ConeStackDepoDemo extends LinearOpMode {
    SampleMecanumDrive drive;

    public void rotate(double rotation) {
        drive.setWeightedDrivePower(new Pose2d(0, 0, rotation));
    }

    static int ZONE = 3;
    @Override
    public void runOpMode() throws InterruptedException {

        DigitalChannel led0 = hardwareMap.get(DigitalChannel.class, "led0");
        DigitalChannel led1 = hardwareMap.get(DigitalChannel.class, "led1");

        led0.setMode(DigitalChannel.Mode.OUTPUT);
        led1.setMode(DigitalChannel.Mode.OUTPUT);

        led0.setState(true);
        led1.setState(true);

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TapePositionSensor tapeSensor = new TapePositionSensor(hardwareMap, TapePositionSensor.TapeColour.RED);

        telemetry.speak("Initialising. Please load cones");
        JunctionPositionSensor sensor = new JunctionPositionSensor(hardwareMap);
        CRServoImplEx intake = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");

        SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide1"), hardwareMap.get(DcMotorEx.class, "linearSlide2"), 20, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        this.drive = drive;

        // We want to start the bot at x: 10, y: -8, heading: 90 degree

        drive.setPoseEstimate(TrajectoryGenerator.getInstance().getStartPose());

        TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();

        TrajectoryGenerator.getInstance().initialise(TrajectoryGenerator.Alliance.RED, TrajectoryGenerator.Side.RIGHT, drive, slideSystem);

        DetectAprilTagZoneUtil.initialise(hardwareMap, telemetry);

        waitForStart();

        telemetry.speak("Stand back drivers. I have commenced the demo.");

        ZONE = DetectAprilTagZoneUtil.getZone(hardwareMap, telemetry);

        telemetry.speak("I have detected the zone as " + ZONE);
        telemetry.clearAll();
        telemetry.addLine("Finished detecting zone, going to junction");
        telemetry.update();

        led0.setState(false);

        clearSignalFromOrigin(led0, intake, slideSystem, drive, trajectoryGenerator.getClearSignal());

        goFirstJunction(sensor, intake, slideSystem, drive, trajectoryGenerator.getGoToJunction1());

        telemetry.clearAll();
        telemetry.addLine("Deposited, going to stack");
        telemetry.update();

        goStackJunction1(led1, multipleTelemetry, tapeSensor, intake, slideSystem, drive, trajectoryGenerator.getAtConePose(), trajectoryGenerator.getGoToStack1(), trajectoryGenerator.getGoForwardToStack());

        goSecondJunctionFromStack(sensor, intake, slideSystem, drive, trajectoryGenerator.getGoToStack1());

        parkFromJunction2(drive, trajectoryGenerator.getParkLeft(), trajectoryGenerator.getParkCentre(), trajectoryGenerator.getParkRight());

    }

    private void parkFromJunction2(SampleMecanumDrive drive, TrajectorySequence parkLeft, TrajectorySequence parkCentre, TrajectorySequence parkRight) {
        if(ZONE == 1) {
            telemetry.speak("I am parking in the left zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in left zone");
            telemetry.update();
            drive.followTrajectorySequence(parkLeft);
        } else if(ZONE == 2) {
            telemetry.speak("I am parking in the centre zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in centre zone");
            telemetry.update();
            drive.followTrajectorySequence(parkCentre);
        } else if(ZONE == 3) {
            telemetry.speak("I am parking in the right zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in right zone");
            telemetry.update();
            drive.followTrajectorySequence(parkRight);
        } else {
            telemetry.speak("I have failed to detect the zone, so I am parking in the centre zone");
            telemetry.clearAll();
            telemetry.addLine("Parking in centre zone");
            telemetry.update();
            drive.followTrajectorySequence(parkCentre);
        }
    }

    private void goSecondJunctionFromStack(JunctionPositionSensor sensor, CRServoImplEx intake, SlidePositionSetter slideSystem, SampleMecanumDrive drive, TrajectorySequence goToJunction2) {
        drive.followTrajectorySequence(goToJunction2);
        telemetry.clearAll();
        telemetry.addLine("At junction, attempting alignment");
        telemetry.update();
        sensor.align(this::rotate);

        drive.updatePoseEstimate();

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(2)
                .build());

        telemetry.speak("Finished re-aligning");
        slideSystem.setPosition(SlidePositions.TOP.getPosition() - 200);
        sleep(250);
        intake.setPower(-0.3);
        sleep(1000);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
    }

    private void goStackJunction1(DigitalChannel led1, MultipleTelemetry multipleTelemetry, TapePositionSensor tapeSensor, CRServoImplEx intake, SlidePositionSetter slideSystem, SampleMecanumDrive drive, Pose2d atConePose, TrajectorySequence goToStack1, TrajectorySequence goForwardToStack) {
        drive.followTrajectorySequence(goToStack1);

        telemetry.clearAll();
        telemetry.addLine("At stack, attempting to align");
        telemetry.update();

        telemetry.speak("Attempting to align");
        boolean spoken = false;

        while(!tapeSensor.isInRange()) {
            if(!spoken) telemetry.speak("Sensor not in range");
            spoken = true;
            drive.setWeightedDrivePower(new Pose2d(0, -0.4, 0));
            multipleTelemetry.update();
            //telemetry.update();
        }

        telemetry.clearAll();
        telemetry.addLine("In Range");
        telemetry.update();

        if(spoken) {
            telemetry.speak("Now in range");
        } else {
            telemetry.speak("Already in range");
        }

        doAlignment(multipleTelemetry, tapeSensor, drive);
        telemetry.addLine("Alignment successful");
        telemetry.update();

        drive.setPoseEstimate(new Pose2d(55, -12));

        telemetry.addLine("Trying to get to stack");
        telemetry.update();

        drive.followTrajectorySequence(goForwardToStack);

        telemetry.speak("Attempting to align");
        spoken = false;

        while (!tapeSensor.isInRange()) {
            if (!spoken) telemetry.speak("Sensor not in range");
            spoken = true;
            drive.setWeightedDrivePower(new Pose2d(0, -0.4, 0));
            multipleTelemetry.update();
            //telemetry.update();
        }

        telemetry.clearAll();
        telemetry.addLine("In Range");
        telemetry.update();

        doAlignment(multipleTelemetry, tapeSensor, drive);

        slideSystem.setPosition(SlidePositions.WALL.getPosition() - 250);
        intake.setPower(0.3);
        sleep(750);
        slideSystem.setPosition(SlidePositions.WALL.getPosition() + 500);
        led1.setState(false);
        sleep(250);
        led1.setState(true);

        drive.setPoseEstimate(atConePose);
    }

    private void goFirstJunction(JunctionPositionSensor sensor, CRServoImplEx intake, SlidePositionSetter slideSystem, SampleMecanumDrive drive, TrajectorySequence getToJunction) {
        drive.followTrajectorySequence(getToJunction);
        telemetry.clearAll();
        telemetry.addLine("At junction, attempting alignment");
        telemetry.update();
        sensor.align(this::rotate);
        drive.updatePoseEstimate();
        //drive.setPoseEstimate(getToJunction.end());
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(1)
                .build());
        telemetry.speak("Finished re-aligning");
        slideSystem.setPosition(SlidePositions.TOP.getPosition() - 200);
        sleep(250);
        intake.setPower(-1);
        sleep(500);
        intake.setPower(0);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
        intake.setPower(0.3);
        sleep(200);
        intake.setPower(0);
    }

    private static void clearSignalFromOrigin(DigitalChannel led0, CRServoImplEx intake, SlidePositionSetter slideSystem, SampleMecanumDrive drive, TrajectorySequence clearSignal) {
        intake.setPower(0.5);
        drive.followTrajectorySequence(clearSignal);
        intake.setPower(0.3);
        slideSystem.setPosition(SlidePositions.TOP.getPosition());
        led0.setState(true);
    }

    private void doAlignment(MultipleTelemetry multipleTelemetry, TapePositionSensor tapeSensor, SampleMecanumDrive drive) {
        boolean res = tapeSensor.align((double strafeAmount) -> {
            drive.setWeightedDrivePower(new Pose2d(0, strafeAmount * -AutoConstants.TAPE_STRAFE_COEFFICIENT, 0));
            multipleTelemetry.addData("Strafe", strafeAmount * AutoConstants.TAPE_STRAFE_COEFFICIENT);
            multipleTelemetry.update();
        }, multipleTelemetry);

        if(res) {
            telemetry.speak("Aligned successfully");
        } else {
            telemetry.speak("Failed to align");
        }


        telemetry.clearAll();
    }
}
