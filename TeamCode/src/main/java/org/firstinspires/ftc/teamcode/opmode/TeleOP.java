package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public void loop(){
        if (!robot.drive.isBusy()) {
            switch (driveMode) {
                // TODO: tune this to liking
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

    }
}