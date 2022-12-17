package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.trajectories.TrajectoryBlueLeft;
import org.firstinspires.ftc.teamcode.subsytems.Glider;
import org.firstinspires.ftc.teamcode.subsytems.Outtake;
import org.firstinspires.ftc.teamcode.subsytems.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoBlueLeftStanga extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);

        robot.start();
        robot.drive.setPoseEstimate(TrajectoryBlueLeft.START_POSE);

        int readFromCamera = -1;

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Tag ID 1,2,3 from the 36h11 family
        /*EDIT IF NEEDED!!!*/

        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        if (tag.id == LEFT) {
                            readFromCamera = 1;
                        }
                        if (tag.id == MIDDLE) {
                            readFromCamera = 2;
                        }
                        if (tag.id == RIGHT) {
                            readFromCamera = 3;
                        }
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addData("tele1 ", tagOfInterest.id);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        telemetry.addData("tele1 ", tagOfInterest.id);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    telemetry.addData("telemetry ", tagOfInterest.id);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) {
            robot.stop();
            return;
        }

        // 1 middle
        // 2 left
        // 3 right
        List<Trajectory> trajectories = TrajectoryBlueLeft.getTrajectories(readFromCamera);
        telemetry.addData("caemera vede", readFromCamera);

        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(0.4);

        robot.glider.sliderState = Glider.SliderState.LOW;

        // se duce in colt
        robot.drive.followTrajectory(trajectories.get(0));

        // mere in fata la lasat de preload + rotit
        robot.drive.followTrajectory(trajectories.get(1));

        robot.glider.triggerOn = true;
        robot.glider.slideLeft.setPower(robot.glider.RETRACT_POWER_SLOW);
        robot.glider.slideRight.setPower(robot.glider.RETRACT_POWER_SLOW);
        robot.sleep(0.35);


        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(1);
        robot.glider.triggerOn = false;
        robot.sleep(0.2);

        // dat in spate
        robot.drive.followTrajectory(trajectories.get(2));
        robot.glider.sliderState = Glider.SliderState.STACKED_CONES;

        // rotit la conuri
        robot.drive.followTrajectory(trajectories.get(3));
        robot.sleep(0.2);

        // shusta persshuta mers i nfata
        robot.drive.followTrajectory(trajectories.get(4));

        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(0.75);

        // a prins conul primului cycle

        robot.glider.sliderState = Glider.SliderState.LOW;
        robot.sleep(0.8);

        robot.drive.followTrajectory(trajectories.get(5));
        robot.drive.followTrajectory(trajectories.get(6));

        robot.glider.triggerOn = true;
        robot.glider.slideLeft.setPower(robot.glider.RETRACT_POWER_SLOW);
        robot.glider.slideRight.setPower(robot.glider.RETRACT_POWER_SLOW);
        robot.sleep(0.35);
        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(1);
        robot.glider.triggerOn = false;
        robot.sleep(0.2);

        robot.glider.sliderState = Glider.SliderState.IDLE;
        robot.sleep(0.2);

        // park
        robot.drive.followTrajectory(trajectories.get(7));
        robot.drive.followTrajectory(trajectories.get(8));
        robot.sleep(10);


        robot.stop();
    }
}