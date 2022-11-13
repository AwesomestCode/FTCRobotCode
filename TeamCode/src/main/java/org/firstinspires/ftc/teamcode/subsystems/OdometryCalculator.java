package org.firstinspires.ftc.teamcode.subsystems;

// taken from https://github.com/FTCLib/FTCLib/blob/master/core/src/main/java/com/arcrobotics/ftclib/kinematics/OdometryCalculator.java

import android.annotation.SuppressLint;
import org.firstinspires.ftc.teamcode.lib.Pose2d;
import org.firstinspires.ftc.teamcode.lib.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.Twist2d;

import java.util.function.DoubleSupplier;

public class OdometryCalculator {

    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    private double centerWheelOffset;

    /**
     * The {@link Pose2d} of the robot.
     */
    protected Pose2d robotPose;

    /**
     * The trackwidth of the odometers
     */
    protected double trackWidth;

    // the suppliers
    DoubleSupplier m_left, m_right, m_horizontal;

    public OdometryCalculator(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                             DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset) {
        this(trackWidth, centerWheelOffset);
        m_left = leftEncoder;
        m_right = rightEncoder;
        m_horizontal = horizontalEncoder;
    }

    public OdometryCalculator(Pose2d initialPose, double trackwidth, double centerWheelOffset) {
        //super(initialPose, trackwidth);
        previousAngle = initialPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
    }

    public OdometryCalculator(double trackwidth, double centerWheelOffset) {
        this(new Pose2d(), trackwidth, centerWheelOffset);
    }

    /**
     * This handles all the calculations for you.
     */
    @SuppressLint("NewApi")
    public void updatePose() {
        update(m_left.getAsDouble(), m_right.getAsDouble(), m_horizontal.getAsDouble());
    }

    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftEncoder - deltaRightEncoder) / trackWidth
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

}