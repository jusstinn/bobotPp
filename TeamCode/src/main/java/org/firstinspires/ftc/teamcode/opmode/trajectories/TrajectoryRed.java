package org.firstinspires.ftc.teamcode.opmode.trajectories;

import static org.firstinspires.ftc.teamcode.subsytems.DriveConstants.BASE_ACCEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.subsytems.DriveConstants.BASE_VEL_CONSTRAINT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryRed{
    public static final Pose2d START_POSE = new Pose2d(36, -64, Math.toRadians(90));

    private static Pose2d getTrajectorySequenceEndPos(List<Trajectory> trajectories) {
        if (trajectories.size() == 0) {
            return START_POSE;
        } else {
            return trajectories.get(trajectories.size() - 1).end();
        }
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startHeading, TrajectoryVelocityConstraint velocityConstraint, TrajectoryAccelerationConstraint accelerationConstraint) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPos(trajectories), startHeading, velocityConstraint, accelerationConstraint);
    }

    private static TrajectoryBuilder makeTrajectoryBuilder(List<Trajectory> trajectories, double startHeading) {
        return new TrajectoryBuilder(getTrajectorySequenceEndPos(trajectories), startHeading, BASE_VEL_CONSTRAINT, BASE_ACCEL_CONSTRAINT);
    }

    public static List<Trajectory> getTrajectoriesMiddle() {
        List<Trajectory> trajectories = new ArrayList<>();

        // 0
        // se duce in colt + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(78, -64, Math.toRadians(90)))
                .build()
        );

        // 1
        // mere in fata la lasat de preload + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(64, -18.5, Math.toRadians(180)))
                .build()
        );

        // 2
        // um pic spate sa nu dea de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(65, -17, Math.toRadians(180)))
                .build()
        );

        // 3
        // rotti 130 luat con
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(63, -15, Math.toRadians(35)))
                .build()
        );

        // 4
        // mers fata
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(35))
                .lineToLinearHeading(new Pose2d(73, -12, Math.toRadians(35)))
                .build()
        );

        // 5
        // reintors la pus de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(35))
                .lineToLinearHeading(new Pose2d(63, -24, Math.toRadians(180)))
                .build()
        );


        // 6?????????
        // GO TO PARKING PARTK
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(65, -20, Math.toRadians(180)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(64, -20, Math.toRadians(180)))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesLeft() {
        List<Trajectory> trajectories = new ArrayList<>();

        // 0
        // se duce in colt + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(78, -64, Math.toRadians(90)))
                .build()
        );

        // 1
        // mere in fata la lasat de preload + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(64, -18.5, Math.toRadians(180)))
                .build()
        );

        // 2
        // um pic spate sa nu dea de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(65, -17, Math.toRadians(180)))
                .build()
        );

        // 3
        // rotti 130 luat con
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(63, -15, Math.toRadians(35)))
                .build()
        );

        // 4
        // mers fata
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(35))
                .lineToLinearHeading(new Pose2d(67, -12, Math.toRadians(35)))
                .build()
        );

        // 5
        // reintors la pus de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(35))
                .lineToLinearHeading(new Pose2d(63, -24, Math.toRadians(180)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(67, -3, Math.toRadians(180)))
                .build()
        );

        // 6?????????
        // GO TO PARKING PARTK
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(42, -3, Math.toRadians(180)))
                .build()
        );

        return trajectories;
    }

    public static List<Trajectory> getTrajectoriesRight() {
        List<Trajectory> trajectories = new ArrayList<>();

        // 0
        // se duce in colt + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(78, -64, Math.toRadians(90)))
                .build()
        );

        // 1
        // mere in fata la lasat de preload + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(64, -18.5, Math.toRadians(180)))
                .build()
        );

        // 2
        // um pic spate sa nu dea de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(65, -17, Math.toRadians(180)))
                .build()
        );

        // 3
        // rotti 130 luat con
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(63, -15, Math.toRadians(35)))
                .build()
        );

        // 4
        // mers fata
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(35))
                .lineToLinearHeading(new Pose2d(73, -12, Math.toRadians(35)))
                .build()
        );

        // 5
        // reintors la pus de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(35))
                .lineToLinearHeading(new Pose2d(63, -24, Math.toRadians(180)))
                .build()
        );

        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(67, -3, Math.toRadians(180)))
                .build()
        );


        // 6?????????
        // GO TO PARKING PARTK
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(25, -3, Math.toRadians(180)))
                .build()
        );

        return trajectories;
    }
}
