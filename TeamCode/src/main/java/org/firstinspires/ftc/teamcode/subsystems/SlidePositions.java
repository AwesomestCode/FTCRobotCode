package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public enum SlidePositions {
    TOP(4200),
    MEDIUM(2800),
    LOW(1500),
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
