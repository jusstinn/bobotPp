package org.firstinspires.ftc.teamcode.subsytems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class Glider implements Sub{
    public static double IDLE_POWER = 0.05;

    public static double EXTEND_POWER_FAST = 0.8;
    public static double EXTEND_POWER_NORMAL = 0.7;
    public static double EXTEND_POWER_SLOW = 0.6;
    public static double RETRACT_POWER_FAST = -0.5;
    public static double RETRACT_POWER_NORMAL = -0.4;
    public static double RETRACT_POWER_SLOW = -0.3;

    double thresholdFast = 500; // TODO
    double thresholdNormal = 300; // TODO
    double thresholdSlow = 200; // TODO
    double thresholdIdle = 25; // TODO

    public enum SliderState {
        IDLE, // height 0
        LOW, // low stick
        MIDDLE, // middle height stick
        HIGH, // big height stick
        STACKED_CONES, // the height needed to get stacked cones (autonomy shit)
        STACKED_CONES2 // the height needed to get stacked cones (autonomy shit)
    }
    double pozIDLE = 0; // TODO
    double pozLOW = 1700; // TODO
    double pozMIDDLE = 2950; // TODO
    double pozHIGH = 3900; // TODO
    double pozSTACKED = 475;
    double pozSTACKED2 = 400;

    public boolean triggerOn = false;

    public DcMotor slideLeft;

    public DcMotor slideRight;

    public SliderState sliderState;


    public Glider(HardwareMap hardwareMap, Robot robot){
        slideLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideMotorRight");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderState = SliderState.IDLE;
    }

    public double getCurrentPosition(){
        return slideLeft.getCurrentPosition();
    }

    public double getTargetHeight() {
        switch (sliderState){
            case IDLE:
                return pozIDLE;
            case LOW:
                return pozLOW;
            case MIDDLE:
                return pozMIDDLE;
            case HIGH:
                return pozHIGH;
            case STACKED_CONES:
                return pozSTACKED;
            case STACKED_CONES2:
                return pozSTACKED2;
        }
        return -1; // never happens
    }

    @Override
    public void update(){
        if (triggerOn) return;

        double currPoz = slideLeft.getCurrentPosition();
        double targetPoz = getTargetHeight();
        if (currPoz < targetPoz) {
            double diff = targetPoz - currPoz;
            if (diff >= thresholdFast) {
                slideLeft.setPower(EXTEND_POWER_FAST);
                slideRight.setPower(EXTEND_POWER_FAST);
            } else if (diff >= thresholdNormal) {
                slideLeft.setPower(EXTEND_POWER_NORMAL);
                slideRight.setPower(EXTEND_POWER_NORMAL);
            } else if (diff >= thresholdSlow) {
                slideLeft.setPower(EXTEND_POWER_SLOW);
                slideRight.setPower(EXTEND_POWER_SLOW);
            }else if (diff <= thresholdIdle) {
                slideLeft.setPower(IDLE_POWER);
                slideRight.setPower(IDLE_POWER);
            }
        }
        if (currPoz > targetPoz) {
            double diff = currPoz - targetPoz;
            if (diff >= thresholdFast) {
                slideLeft.setPower(RETRACT_POWER_FAST);
                slideRight.setPower(RETRACT_POWER_FAST);
            } else if (diff >= thresholdNormal) {
                slideLeft.setPower(RETRACT_POWER_NORMAL);
                slideRight.setPower(RETRACT_POWER_NORMAL);
            } else if (diff >= thresholdSlow) {
                slideLeft.setPower(RETRACT_POWER_SLOW);
                slideRight.setPower(RETRACT_POWER_SLOW);
            } else if (diff <= thresholdIdle) {
                slideLeft.setPower(IDLE_POWER);
                slideRight.setPower(IDLE_POWER);
            }
        }
    }
}
