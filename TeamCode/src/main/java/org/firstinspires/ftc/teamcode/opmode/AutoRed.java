package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AprilTagAutonomousInitDetectionExample;

import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.opmode.trajectories.TrajectoryRed;
import org.firstinspires.ftc.teamcode.opmode.trajectories.TrajectoyBlue;
import org.firstinspires.ftc.teamcode.subsytems.Glider;
import org.firstinspires.ftc.teamcode.subsytems.Outtake;
import org.firstinspires.ftc.teamcode.subsytems.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoRed extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);

        robot.start();
        robot.drive.setPoseEstimate(TrajectoryRed.START_POSE);

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


        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(0.4);

        // initializat zero
        List<Trajectory> trajectories = TrajectoryRed.getTrajectoriesRight();

        telemetry.addData("caemera vede", readFromCamera);

        // UND ERAU ASTEA COMMUITE

        if (readFromCamera == 1) {
            trajectories = TrajectoryRed.getTrajectoriesMiddle();
        }
        if (readFromCamera == 2) {
            trajectories = TrajectoryRed.getTrajectoriesLeft();
        }
        if (readFromCamera == 3) {
            trajectories = TrajectoryRed.getTrajectoriesRight();
        }



        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(1);

        //robot.glider.sliderState = Glider.SliderState.EXTEND;
        robot.sleep(0.1);
        robot.glider.sliderState = Glider.SliderState.IDLE;



        // se duce in colt + rotit
        robot.drive.followTrajectory(trajectories.get(0));

        // dropat con
        // TODO: ?????mers in spate?????

        //robot.glider.sliderState = Glider.SliderState.EXTEND;
        robot.sleep(0.8);
        robot.glider.sliderState = Glider.SliderState.IDLE;
        // mere in fata la lasat de preload + rotit
        robot.drive.followTrajectory(trajectories.get(1));

        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(1);

        robot.drive.followTrajectory(trajectories.get(2));
        robot.drive.followTrajectory(trajectories.get(3));

        //robot.glider.sliderState = Glider.SliderState.RETRACT;
        robot.sleep(0.75);
        robot.glider.sliderState = Glider.SliderState.IDLE;

        // shusta persshuta mers i nfata
        robot.drive.followTrajectory(trajectories.get(4));

        robot.outtake.clawState = Outtake.ClawState.CLOSED;
        robot.sleep(1);


        // a prins conul primului cycle


        //robot.glider.sliderState = Glider.SliderState.EXTEND;
        robot.sleep(0.8);
        robot.glider.sliderState = Glider.SliderState.IDLE;

        robot.drive.followTrajectory(trajectories.get(5));

        robot.outtake.clawState = Outtake.ClawState.OPEN;
        robot.sleep(1);


        //robot.glider.sliderState = Glider.SliderState.RETRACT;
        robot.sleep(0.75);
        robot.glider.sliderState = Glider.SliderState.IDLE;


        // park
        robot.drive.followTrajectory(trajectories.get(6));

        robot.drive.followTrajectory(trajectories.get(7));
        robot.sleep(10);


        robot.stop();
    }
}