package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autotools.StepRunner;
import org.firstinspires.ftc.teamcode.autotools.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.subsystems.JunctionPositionSensor;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.TapePositionSensor;

@Autonomous(group="Demos")
public class LeftBlueConeStackDepoDemo extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        DigitalChannel led0 = hardwareMap.get(DigitalChannel.class, "led0");
        DigitalChannel led1 = hardwareMap.get(DigitalChannel.class, "led1");

        led0.setMode(DigitalChannel.Mode.OUTPUT);
        led1.setMode(DigitalChannel.Mode.OUTPUT);

        led0.setState(true);
        led1.setState(true);

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TapePositionSensor tapeSensor = new TapePositionSensor(hardwareMap, TapePositionSensor.TapeColour.BLUE);

        telemetry.speak("Initialising. Please load cones");
        JunctionPositionSensor sensor = new JunctionPositionSensor(hardwareMap);
        CRServoImplEx intake = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");

        SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide1"), hardwareMap.get(DcMotorEx.class, "linearSlide2"), 20, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        this.drive = drive;

        TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator.getInstance();

        TrajectoryGenerator.getInstance().initialise(TrajectoryGenerator.Alliance.BLUE, TrajectoryGenerator.Side.LEFT, drive, slideSystem);

        drive.setPoseEstimate(TrajectoryGenerator.getInstance().getStartPose());

        DetectAprilTagZoneUtil.initialise(hardwareMap, telemetry);

        waitForStart();

        telemetry.speak("Stand back drivers. I have commenced the demo.");

        int zone = DetectAprilTagZoneUtil.getZone(hardwareMap, telemetry);

        telemetry.speak("I have detected the zone as " + zone);
        telemetry.clearAll();
        telemetry.addLine("Finished detecting zone, going to junction");
        telemetry.update();

        led0.setState(false);

        StepRunner runner = StepRunner.getInstance(telemetry, drive, TrajectoryGenerator.Side.LEFT);

        runner.clearSignalFromOrigin(led0, intake, slideSystem, trajectoryGenerator.getClearSignal());

        runner.goFirstJunction(sensor, intake, slideSystem, trajectoryGenerator.getGoToJunction1());

        telemetry.clearAll();
        telemetry.addLine("Deposited, going to stack");
        telemetry.update();

        runner.goStackJunction1(led1, multipleTelemetry, tapeSensor, intake, slideSystem, trajectoryGenerator.getAtConePose(), trajectoryGenerator.getGoToStack1(), trajectoryGenerator.getGoForwardToStack());

        runner.goSecondJunctionFromStack(sensor, intake, slideSystem, trajectoryGenerator.getGoToJunction2());

        runner.parkFromJunction2(trajectoryGenerator.getParkLeft(), trajectoryGenerator.getParkCentre(), trajectoryGenerator.getParkRight(), zone, slideSystem);
        sleep(1500);

    }

}
