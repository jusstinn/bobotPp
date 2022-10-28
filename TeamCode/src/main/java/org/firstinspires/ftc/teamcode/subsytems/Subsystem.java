package org.firstinspires.ftc.teamcode.subsytems;

public interface Subsystem {
    void update();
    default void stop() { }
}
