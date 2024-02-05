package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.variables;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class Auto_Blue extends LinearOpMode {
    SimpleServo clawAngleServo;
    SimpleServo clawRightServo;
    SimpleServo clawLeftServo;
    //double gripDegrees = 43;

    //double gripDegrees1 = 82;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        clawRightServo = new SimpleServo(
                hardwareMap, "clawRightServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawLeftServo = new SimpleServo(
                hardwareMap, "clawLeftServo", 10, 120,
                AngleUnit.DEGREES
        );
        clawAngleServo = new SimpleServo(
                hardwareMap, "clawAngleServo", 0, 180,
                AngleUnit.DEGREES
        );

        Pose2d startPose = new Pose2d(0, 0, 0);
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);
/*
        //Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(10, 10), 0)
                .build();

        //Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(25, -15), 0)
                .build();
*/
        Trajectory traj3 = drive.trajectoryBuilder(startPose)
                .forward(65)
                .build();

        // strafeRight(10) cannot be included in traj3 as it throws a PathContinuityException
        Trajectory traj4 = drive.trajectoryBuilder(startPose)
                .forward(110)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)), 0)
                .build();

        waitForStart();

        if (isStopRequested())
            return;

        /*drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj2);

        timer.reset();
        while(timer.seconds() < 3) drive.update();
*/
        //      drive.turn(Math.toRadians(45));
        clawRightServo.turnToAngle(variables.gripDegrees1);
        clawLeftServo.turnToAngle(variables.gripDegrees);
        drive.followTrajectory(traj3);
        drive.turn(Math.toRadians(150));
        drive.followTrajectory(traj4);
        clawAngleServo.turnToAngle(variables.AutoCLawDown);
        timer.reset();
        while(timer.seconds() < 1) drive.update();
        clawRightServo.turnToAngle(variables.gripDegrees);
        clawLeftServo.turnToAngle(variables.gripDegrees1);
        timer.reset();
        while(timer.seconds() < 2) drive.update();
        clawRightServo.turnToAngle(variables.gripDegrees1);
        clawLeftServo.turnToAngle(variables.gripDegrees);
        timer.reset();
        while(timer.seconds() < 1) drive.update();
        clawAngleServo.turnToAngle(variables.AutoCLawPark);

        while(!isStopRequested()) {

        }

        /*        drive.followTrajectory(traj4);

        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj5);

        timer.reset();
        while(timer.seconds() < 1) drive.update();

        drive.followTrajectory(traj6);*/
    }
}

