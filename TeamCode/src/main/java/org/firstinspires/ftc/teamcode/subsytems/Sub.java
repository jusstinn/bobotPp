package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.canvas.Canvas;

public interface Sub {
    void update();

    default void stop() { }
}
