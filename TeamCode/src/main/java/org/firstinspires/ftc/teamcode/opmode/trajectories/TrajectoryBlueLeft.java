package org.firstinspires.ftc.teamcode.opmode.trajectories;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import static org.firstinspires.ftc.teamcode.subsytems.DriveConstants.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class TrajectoryBlueLeft {
    public static final Pose2d START_POSE = new Pose2d(-36, -64, Math.toRadians(90));

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

    public static List<Trajectory> getTrajectories(int value) {
        List<Trajectory> trajectories = new ArrayList<>();

        // 0
        // se duce in colt + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-72, -64, Math.toRadians(90)))
                .build()
        );

        // 1
        // mere in fata la lasat de preload + rotit
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-62, -14.5, Math.toRadians(0)))
                .build()
        );

        // 2
        // um pic spate sa nu dea de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-70, -17, Math.toRadians(0)))
                .build()
        );

        // 3
        // rotti 130 luat con
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-65, -13, Math.toRadians(145)))
                .build()
        );

        // 4
        // mers fata
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(145))
                .lineToLinearHeading(new Pose2d(-73, -9, Math.toRadians(145)))
                .build()
        );

        // 5
        // reintors la pus de bat
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(145))
                .lineToLinearHeading(new Pose2d(-64, -24, Math.toRadians(0)))
                .build()
        );

        // 6
        // MERS FATA SA NU DEA PESTE CONU DE JOS
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-61, -16.5, Math.toRadians(0)))
                .build()
        );


        // 7?????????
        // GO TO PARKING PARTK
        trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-67, -3, Math.toRadians(0)))
                .build()
        );

        //8
        if (value == 1 || value == -1) {
            // Middle
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-64, -3, Math.toRadians(0)))
                    .build()
            );
        } else if (value == 2) {
            //Left
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-46, -3, Math.toRadians(0)))
                    .build()
            );
        } else if (value == 3) {
            // Right
            trajectories.add(makeTrajectoryBuilder(trajectories, Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-15, -3, Math.toRadians(0)))
                    .build()
            );
        }
        return trajectories;
    }
}