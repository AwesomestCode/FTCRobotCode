package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class JunctionPositionSensor {

    private DistanceSensor leftSensor;
    private DistanceSensor centreSensor;
    private DistanceSensor rightSensor;

    public JunctionPositionSensor(HardwareMap map) {
        leftSensor = map.get(DistanceSensor.class, "ds0");
        centreSensor = map.get(DistanceSensor.class, "ds1");
        rightSensor = map.get(DistanceSensor.class, "ds2");
    }

    /*public float getPosition() {
        if(colour == TapeColour.RED) {
            return (rightSensor.getHsvValues()[0] - leftSensor.getHsvValues()[0])/ 255.0f;
        } else {
            throw new UnsupportedOperationException("Blue tape not supported yet");
        }
    }*/
    public int getEstimate() {
        // return the number of the sensor that's closest
        // -1 = left, 0 = centre, 1 = right
        double left = leftSensor.getDistance(DistanceUnit.CM);
        double centre = centreSensor.getDistance(DistanceUnit.CM);
        double right = rightSensor.getDistance(DistanceUnit.CM);

        if (left < centre && left < right) {
            return -1;
        } else if (centre < left && centre < right) {
            return 0;
        } else {
            return 1;
        }
    }

    public void align(DoubleConsumer rotater) {
        int estimate;
        do {
            estimate = getEstimate();
            rotater.accept(-estimate * 0.2);
        } while (estimate != 0);
    }

    /*public static class PIDController {
        //private double kp;
        //private double ki;
        //private double kd;
        private double lastError;
        private double integral;
        private double derivative;
        private static final double SETPOINT = 0;

        DoubleSupplier input;


        private ElapsedTime timer = new ElapsedTime();

        public PIDController(DoubleSupplier input) {
            timer.reset();
            lastError = SETPOINT - input.getAsDouble();
            this.input = input;
            Timer timer = new Timer();
            timer.schedule(new java.util.TimerTask() {
                @Override
                public void run() {
                    updateValues();
                }
            }, 0, 10);
        }


        public void updateValues() {
            // Calculate error
            double error = SETPOINT - input.getAsDouble();
            // Calculate integral
            integral = this.integral + (timer.nanoseconds() * error)/Math.pow(10, 9);
            // Calculate derivative
            derivative = ((error - lastError) / timer.nanoseconds()) * Math.pow(10, 9);
            // Set last error
            lastError = error;
            timer.reset();
        }

        public void resetPIDState() {
            derivative = 0;
            integral = 0;
            timer.reset();
        }


        public double getOutput() {
            return kp * lastError + ki * integral + kd * derivative;
        }
        }
        */
}
