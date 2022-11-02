package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake implements Sub{
    public static double CLOSED_POSITION = 0.82;
    public static double OPEN_POSITION = 1;

    public enum  ClawState {
        OPEN,
        CLOSED
    }

    public ClawState clawState;

    public Servo clawServo;

    public Outtake(HardwareMap hardwareMap, Robot robot){
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawState = ClawState.CLOSED;
    }

    @Override
    public void update() {
        switch (clawState) {
            case CLOSED:
                clawServo.setPosition(CLOSED_POSITION);
                break;
            case OPEN:
                clawServo.setPosition(OPEN_POSITION);
                break;
        }
    }
}
