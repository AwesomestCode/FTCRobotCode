package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public enum SlidePositions {
    TOP(2800),
    MEDIUM(2000),
    LOW(1200),
    GROUND(0)
    ;

    int position;

    SlidePositions(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
