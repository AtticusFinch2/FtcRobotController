package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@Autonomous(name = "CPSEventAuto")
public class CPSEventAuto extends LinearOpMode {
    MainRobot robot;
    public Pose2d startPose = new Pose2d(24, 70, Math.toRadians(0));
    public int onetile = 14;
    public int leftright = 25;
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        robot.setPoseEstimate(startPose);

        Trajectory forward_1 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 3)
                .build();
        startPose = forward_1.end();
        Trajectory left_1 = robot.trajectoryBuilder(startPose)
                .strafeLeft(leftright)
                .build();
        startPose = left_1.end();
        Trajectory forward_2 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 2)
                .build();
        startPose = forward_2.end();
        Trajectory right_1 = robot.trajectoryBuilder(startPose)
                .strafeRight(leftright)
                .build();
        startPose = right_1.end();
        Trajectory forward_3 = robot.trajectoryBuilder(startPose)
                .forward((onetile * 4)+6)
                .build();
        startPose = forward_3.end();
        Trajectory left_2 = robot.trajectoryBuilder(startPose)
                .strafeLeft(leftright+2)
                .build();
        startPose = left_2.end();
        Trajectory forward_4 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 1.7)
                .build();
        startPose = forward_4.end();
        Trajectory right_2 = robot.trajectoryBuilder(startPose)
                .strafeRight(leftright-2)
                .build();
        startPose = right_2.end();
        Trajectory forward_5 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 5.6)
                .build();
        startPose = forward_5.end();
        Trajectory corner = robot.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX(), startPose.getY()+(onetile*3), Math.toRadians(180)))
                .build();
        startPose = corner.end();
        Trajectory forward_6 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 3)
                .build();
        startPose = forward_6.end();
        Trajectory left_3 = robot.trajectoryBuilder(startPose)
                .strafeLeft(leftright)
                .build();
        startPose = left_3.end();
        Trajectory forward_7 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 2)
                .build();
        startPose = forward_7.end();
        Trajectory right_3 = robot.trajectoryBuilder(startPose)
                .strafeRight(leftright-4)
                .build();
        startPose = right_3.end();
        Trajectory forward_8 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 5)
                .build();
        startPose = forward_8.end();
        Trajectory left_4 = robot.trajectoryBuilder(startPose)
                .strafeLeft(leftright)
                .build();
        startPose = left_4.end();
        Trajectory forward_9 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 1.8)
                .build();
        startPose = forward_9.end();
        Trajectory right_4 = robot.trajectoryBuilder(startPose)
                .strafeRight(leftright-4)
                .build();
        startPose = right_4.end();
        Trajectory forward_10 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 4)
                .build();
        startPose = forward_10.end();
        Trajectory left_5 = robot.trajectoryBuilder(startPose)
                .strafeLeft(leftright)
                .build();
        startPose = left_5.end();
        Trajectory forward_11 = robot.trajectoryBuilder(startPose)
                .forward(onetile * 2)
                .build();
        startPose = forward_11.end();
        Trajectory right_5 = robot.trajectoryBuilder(startPose)
                .strafeRight(leftright)
                .build();
        startPose = right_5.end();

        robot.followTrajectory(forward_1);
        robot.followTrajectory(left_1);
        robot.followTrajectory(forward_2);
        robot.followTrajectory(right_1);
        robot.followTrajectory(forward_3);
        robot.followTrajectory(left_2);
        robot.followTrajectory(forward_4);
        robot.followTrajectory(right_2);
        robot.followTrajectory(forward_5);
        robot.followTrajectory(corner);
        robot.followTrajectory(forward_6);
        robot.followTrajectory(left_3);
        robot.followTrajectory(forward_7);
        robot.followTrajectory(right_3);
        robot.followTrajectory(forward_8);
        robot.followTrajectory(left_4);
        robot.followTrajectory(forward_9);
        robot.followTrajectory(right_4);
        robot.followTrajectory(forward_10);
        robot.followTrajectory(left_5);
        robot.followTrajectory(forward_11);
        robot.followTrajectory(right_5);
        robot.pause(500);

    }



}