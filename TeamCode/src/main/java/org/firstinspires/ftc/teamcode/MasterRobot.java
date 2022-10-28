package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrainMixer;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositionSetter;
import org.firstinspires.ftc.teamcode.subsystems.SlidePositions;

import java.util.List;

@TeleOp(name="Master Robot", group="Full Code")
public class MasterRobot extends LinearOpMode {

    SlidePositionSetter slideSystem = new SlidePositionSetter(hardwareMap.get(DcMotorEx.class, "linearSlide"), true);

    DcMotorEx frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
    DcMotorEx frontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
    DcMotorEx rearLeft = (DcMotorEx) hardwareMap.dcMotor.get("rearLeft");
    DcMotorEx rearRight = (DcMotorEx) hardwareMap.dcMotor.get("rearRight");

    MecanumDrivetrainMixer mixer = new MecanumDrivetrainMixer(frontLeft, frontRight, rearLeft, rearRight);
    @Override
    public void runOpMode() throws InterruptedException {


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

            handleSlide(oldGamepad1, newGamepad1);

            telemetry.update();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            mixer.setMovement(x, y, rx);

            telemetry.addData("Front Left Amperage", frontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Front Right Amperage", frontRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Rear Left Amperage", rearLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Rear Right Amperage", rearRight.getCurrent(CurrentUnit.AMPS));

            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                telemetry.addData(hub.getDeviceName() + " Input Voltage", hub.getInputVoltage(VoltageUnit.VOLTS));
                telemetry.addData(hub.getDeviceName() + " Current", hub.getCurrent(CurrentUnit.AMPS));
            }
        }
    }
    private void handleSlide(Gamepad oldGamepad1, Gamepad newGamepad1) {
        if (!oldGamepad1.x && newGamepad1.y) {
            slideSystem.setPosition(SlidePositions.TOP.getPosition());
        }
        if (!oldGamepad1.x && newGamepad1.x) {
            slideSystem.setPosition(SlidePositions.LOW.getPosition());
        }
        if (!oldGamepad1.b && newGamepad1.b) {
            slideSystem.setPosition(SlidePositions.MEDIUM.getPosition());
        }
        if (!oldGamepad1.a && newGamepad1.a) {
            slideSystem.setPosition(SlidePositions.GROUND.getPosition());
        }
        telemetry.addData("Target Position", slideSystem.getTargetPosition());
        telemetry.addData("Actual Position", slideSystem.getActualPosition());
    }
}
