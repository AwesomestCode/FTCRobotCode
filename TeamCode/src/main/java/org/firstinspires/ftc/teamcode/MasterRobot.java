package org.firstinspires.ftc.teamcode;

import androidx.annotation.ColorInt;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Master Robot", group="Full Code")
@Config
public class MasterRobot extends LinearOpMode {

    SlidePositionSetter slideSystem;

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    MecanumDrivetrainMixer mixer;

    CRServoImplEx intake;

    IsGitEnabled isGitEnabled;

    public static double POWER_ADJUSTMENT = 0.4;
    public static double MOVEMENT_ADJUSTMENT = 0.2;
    public static int SLIDE_MAX_OFFSET = 500;
    public static double DEFAULT_INTAKE_POWER = 0.067;
    public static boolean FIELD_CENTRIC = false;
    public static boolean DUAL_CONTROLLER = true;
    public static double DRIVE_MODIFIER_EXPONENT = 2;
    public static double MAX_SLIDE_MOTOR_AMP = 4;

    BNO055IMU imu;

    double initialHeading;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
        rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");
        mixer = new MecanumDrivetrainMixer(frontLeft, frontRight, rearLeft, rearRight);
        slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide1"), hardwareMap.get(DcMotorEx.class, "linearSlide2"), MAX_SLIDE_MOTOR_AMP, true);
        intake = (CRServoImplEx) hardwareMap.get(CRServo.class, "intake");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        initialHeading = -imu.getAngularOrientation().firstAngle;

        waitForStart();

        Gamepad oldGamepad1 = new Gamepad();
        Gamepad newGamepad1 = new Gamepad();
        Gamepad oldGamepad2 = new Gamepad();
        Gamepad newGamepad2 = new Gamepad();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while(opModeIsActive()) {
            try {
                oldGamepad1.copy(newGamepad1);
                newGamepad1.copy(gamepad1);

                oldGamepad2.copy(newGamepad2);
                newGamepad2.copy(gamepad2);
                sleep(10);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            handleDrive(newGamepad1);
            handleSlide(DUAL_CONTROLLER ? oldGamepad2 : oldGamepad1, DUAL_CONTROLLER ? newGamepad2 : newGamepad1);
            handleIntake(DUAL_CONTROLLER ? newGamepad2 : newGamepad1);

            if(gamepad1.left_stick_button && gamepad1.right_bumper) { //reset initial heading for recalibrating
                initialHeading = -imu.getAngularOrientation().firstAngle;
                blinkColor(0x00f0d4);
            }

            if(gamepad2.left_stick_button && gamepad2.right_stick_button) { //reset zero position for slide motor
                slideSystem.reset();
                blinkColor(0xff0000);
            }

            if(slideSystem.getAreMotorsOverCurrent()) {
                telemetry.speak("Please let my slide take a break, it's aching.");
                gamepad2.rumble(1, 1, 500);
                telemetry.addLine("Slides are over current");
            }

            for (LynxModule hub : allHubs) {
                telemetry.addData(hub.getDeviceName() + " Input Voltage", hub.getInputVoltage(VoltageUnit.VOLTS));
                telemetry.addData(hub.getDeviceName() + " Current", hub.getCurrent(CurrentUnit.AMPS));
            }

            telemetry.addData("Strafing Deadwheel", frontLeft.getCurrentPosition()); //encoder is plugged into the port of frontLeft

            telemetry.update();
        }
    }
    private void handleSlide(Gamepad oldGamepad2, Gamepad newGamepad2) {
        slideSystem.setSpeed(POWER_ADJUSTMENT);
        if (!oldGamepad2.y && newGamepad2.y) {
            slideSystem.setPosition(SlidePositions.TOP.getPosition());
        }
        if (!oldGamepad2.x && newGamepad2.x) {
            slideSystem.setPosition(SlidePositions.LOW.getPosition());
        }
        if (!oldGamepad2.b && newGamepad2.b) {
            slideSystem.setPosition(SlidePositions.MEDIUM.getPosition());
        }
        if (!oldGamepad2.a && newGamepad2.a) {
            slideSystem.setPosition(SlidePositions.GROUND.getPosition());
        }
        slideSystem.setOffset((int) ((newGamepad2.right_trigger - newGamepad2.left_trigger) * SLIDE_MAX_OFFSET));
        telemetry.addData("Target Position", slideSystem.getTargetPosition());
        telemetry.addData("Slide 1 Actual Position", slideSystem.getActualSlideMotor1Position());
        telemetry.addData("Slide 2 Actual Position", slideSystem.getActualSlideMotor2Position());
    }

    private void handleIntake(Gamepad intakeGamepad) {
        if (intakeGamepad.dpad_up) {
            intake.setPower(-1.0);
        } else if (intakeGamepad.dpad_down) {
            intake.setPower(1.0);
        } else {
            intake.setPower(DEFAULT_INTAKE_POWER);
        }
    }

    private void handleDrive(Gamepad driveGamepad) {
        double y = MOVEMENT_ADJUSTMENT * (gamepad1.left_stick_y > 0 ? -Math.abs(Math.pow(gamepad1.left_stick_y, DRIVE_MODIFIER_EXPONENT)) : Math.abs(Math.pow(gamepad1.left_stick_y, DRIVE_MODIFIER_EXPONENT)));
        double x = MOVEMENT_ADJUSTMENT * (gamepad1.left_stick_x > 0 ? Math.abs(Math.pow(gamepad1.left_stick_x, DRIVE_MODIFIER_EXPONENT)) : -Math.abs(Math.pow(gamepad1.left_stick_x, DRIVE_MODIFIER_EXPONENT)));
        double rx = MOVEMENT_ADJUSTMENT * (gamepad1.right_stick_x > 0 ? Math.abs(Math.pow(gamepad1.right_stick_x, DRIVE_MODIFIER_EXPONENT)): - Math.abs(Math.pow(gamepad1.right_stick_x, DRIVE_MODIFIER_EXPONENT)));

        if(FIELD_CENTRIC) {
            mixer.setMovement(x, y, rx, -imu.getAngularOrientation().firstAngle - initialHeading);
        }
        else {
            mixer.setMovement(x, y, rx);
        }

        telemetry.addData("Front Left Amperage", frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Right Amperage", frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear Left Amperage", rearLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear Right Amperage", rearRight.getCurrent(CurrentUnit.AMPS));
    }

    private void blinkColor(@ColorInt int color) {
        ArrayList<Blinker.Step> steps = new ArrayList<>();
        steps.add(new Blinker.Step(color, 1, TimeUnit.SECONDS));
        steps.add(new Blinker.Step(0x00ff88, 1, TimeUnit.SECONDS));

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs) {
            hub.pushPattern(steps);
        }
    }
}
