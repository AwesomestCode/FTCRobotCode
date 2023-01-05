package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TapePositionSensor {
    public enum TapeColour {
        RED,
        BLUE
    }

    private ColourSensor leftSensor;
    private ColourSensor rightSensor;
    private TapeColour colour;

    public TapePositionSensor(HardwareMap map, TapeColour colour) {
        leftSensor = new ColourSensor(map, "cs0");
        rightSensor = new ColourSensor(map, "cs1");
        this.colour = colour;
    }

    public float getPosition() {
        if(colour == TapeColour.RED) {
            return (leftSensor.getHsvValues()[0] - rightSensor.getHsvValues()[0])/ 255.0f;
        } else {
            throw new UnsupportedOperationException("Blue tape not supported yet");
        }
    }
}
