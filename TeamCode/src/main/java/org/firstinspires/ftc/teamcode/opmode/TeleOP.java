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

        telemetry.addData("claw position ", robot.outtake.clawServo.getPosition());
        telemetry.addData("shessh left pos ", robot.glider.slideLeft.getCurrentPosition());
        telemetry.addData("shessh right pos ", robot.glider.slideRight.getCurrentPosition());
        telemetry.addData("slider state ", robot.glider.sliderState);
        if (gamepad2.y) {
            if(gamepad2.dpad_right){
                robot.glider.triggerOn = false;
                robot.glider.sliderState = Glider.SliderState.HIGH;
            }
            if(gamepad2.dpad_up){
                robot.glider.triggerOn = false;
                robot.glider.sliderState = Glider.SliderState.MIDDLE;
            }
            if(gamepad2.dpad_left){
                robot.glider.triggerOn = false;
                robot.glider.sliderState = Glider.SliderState.LOW;
            }
            if (gamepad2.dpad_down){
                robot.glider.triggerOn = false;
                robot.glider.sliderState = Glider.SliderState.IDLE;
            }
        }

        if(gamepad2.right_trigger > 0){
            robot.glider.slideLeft.setPower(robot.glider.EXTEND_POWER_FAST);
            robot.glider.slideRight.setPower(robot.glider.EXTEND_POWER_FAST);
            robot.glider.triggerOn = true;
        } else if(gamepad2.left_trigger > 0){
            robot.glider.slideLeft.setPower(robot.glider.RETRACT_POWER_FAST);
            robot.glider.slideRight.setPower(robot.glider.RETRACT_POWER_FAST);
            robot.glider.triggerOn = true;
        } else if (robot.glider.triggerOn) {
            robot.glider.slideLeft.setPower(robot.glider.IDLE_POWER);
            robot.glider.slideRight.setPower(robot.glider.IDLE_POWER);
        }
        if(gamepad2.a){
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
        } else if (gamepad2.b){
            robot.outtake.clawState = Outtake.ClawState.OPEN;
        }
        if(gamepad1.a){
            robot.outtake.clawState = Outtake.ClawState.CLOSED;
        } else if (gamepad1.b){
            robot.outtake.clawState = Outtake.ClawState.OPEN;
        }
    }
}
