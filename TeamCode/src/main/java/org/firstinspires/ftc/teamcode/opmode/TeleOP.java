package org.firstinspires.ftc.teamcode.opmode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsytems.Glider;
import org.firstinspires.ftc.teamcode.subsytems.Outtake;
import org.firstinspires.ftc.teamcode.subsytems.Robot;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@Config
@TeleOp
public class TeleOP extends OpMode {
    Robot robot;

    enum DriveMode {
        FAST,
        SLOW
    }

    StickyGamepad stickyGamepad1;
    StickyGamepad stickyGamepad2;

    MultipleTelemetry telemetry;

    DriveMode driveMode;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(this, false);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        driveMode = DriveMode.FAST;

        telemetry.log().add("Ready!");
    }
    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        if (!robot.drive.isBusy()) {
            switch (driveMode) {
                case FAST:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.7);
                    break;
                case SLOW:
                    robot.drive.setMotorPowersFromGamepad(gamepad1, 0.4);
                    break;
            }
        }
        if(stickyGamepad1.y){
            switch (driveMode){
                case FAST:
                    driveMode = DriveMode.SLOW;
                    break;
                case SLOW:
                    driveMode = DriveMode.FAST;
                    break;
            }
        }

        telemetry.addData("shess", robot.glider.slide.getCurrentPosition());
        telemetry.addData("shess222", robot.glider.sliderState);
        if(gamepad1.a){
            robot.glider.triggerOn = false;
            robot.glider.sliderState = Glider.SliderState.HIGH;
        } else if(gamepad1.b){
            robot.glider.triggerOn = false;
            robot.glider.sliderState = Glider.SliderState.MIDDLE;
        } else if(gamepad1.x){
            robot.glider.triggerOn = false;
            robot.glider.sliderState = Glider.SliderState.LOW;
        } else if (gamepad1.y){
            robot.glider.triggerOn = false;
            robot.glider.sliderState = Glider.SliderState.IDLE;
        }

        if(gamepad1.right_trigger > 0){
            robot.glider.slide.setPower(robot.glider.EXTEND_POWER_FAST);
            robot.glider.triggerOn = true;
        } else if(gamepad1.left_trigger > 0){
            robot.glider.slide.setPower(robot.glider.RETRACT_POWER_FAST);
            robot.glider.triggerOn = true;
        } else if (robot.glider.triggerOn) {
            robot.glider.slide.setPower(robot.glider.IDLE_POWER);
        }
        if(gamepad2.a){
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
        }
        else if (gamepad2.b){
            robot.outtake.clawState = Outtake.ClawState.OPEN;
        }
    }
}
