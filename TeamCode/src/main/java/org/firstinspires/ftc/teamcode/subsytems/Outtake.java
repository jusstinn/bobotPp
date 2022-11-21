package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake implements Sub{
    public static double CLOSED_POSITION_RIGHT = 0.04;
    public static double OPEN_POSITION_RIGHT = 0.25;
    public static double CLOSED_POSITION_LEFT = 0.15;
    public static double OPEN_POSITION_LEFT = 0;

    public enum  ClawState {
        OPEN,
        CLOSED
    }

    public ClawState clawState;

    public Servo clawServo1;
    public Servo clawServo2;

    public Outtake(HardwareMap hardwareMap, Robot robot){
        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");

        clawState = ClawState.OPEN;
    }

    @Override
    public void update() {
        switch (clawState) {
            case CLOSED:
                clawServo2.setPosition(OPEN_POSITION_LEFT);
                clawServo1.setPosition(OPEN_POSITION_RIGHT);
                break;
            case OPEN:
                clawServo2.setPosition(CLOSED_POSITION_LEFT);
                clawServo1.setPosition(CLOSED_POSITION_RIGHT);
                break;
        }
    }
}
