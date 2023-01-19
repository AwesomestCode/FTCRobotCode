package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.ColourSensor;

@TeleOp(group="Demo")
public class ColourSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ColourSensor sensor1 = new ColourSensor(hardwareMap, "cs0", 20);
        ColourSensor sensor2 = new ColourSensor(hardwareMap, "cs1", 20);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Sensor 1 Hue", sensor1.getHsvValues()[0]);
            telemetry.addData("Sensor 1 Saturation", sensor1.getHsvValues()[1]);
            telemetry.addData("Sensor 1 Value", sensor1.getHsvValues()[2]);
            telemetry.addData("Sensor 2 Hue", sensor2.getHsvValues()[0]);
            telemetry.addData("Sensor 2 Saturation", sensor2.getHsvValues()[1]);
            telemetry.addData("Sensor 2 Value", sensor2.getHsvValues()[2]);
            telemetry.update();

            Thread.sleep(10);
        }
    }
}
