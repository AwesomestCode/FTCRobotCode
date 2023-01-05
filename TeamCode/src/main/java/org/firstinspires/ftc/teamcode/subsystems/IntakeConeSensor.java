package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeConeSensor {
    private ColourSensor sensor;

    public IntakeConeSensor(HardwareMap map) {
        sensor = new ColourSensor(map, "intakeSensor");
    }

    private boolean hasCone() {
        return sensor.getDistance() < 5;
    }

    public boolean getColour() {
        return (sensor.getHsvValues()[0] + 85) % 255 < 128;
    }
}
