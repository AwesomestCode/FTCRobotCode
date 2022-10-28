package org.firstinspires.ftc.teamcode.subsystems;

public enum SlidePositions {
    TOP(4200),
    MEDIUM(3000),
    LOW(1000),
    GROUND(0)
    ;

    final int position;

    SlidePositions(int position) {
        this.position = position;
    }

    public int getPosition() {
        return position;
    }
}
