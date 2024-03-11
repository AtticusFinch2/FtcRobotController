package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@Autonomous(name = "BlueFlickFar")
public class BlueFlickFar extends LinearOpMode {
    MainRobot robot;
    int spike = 2;
    public Pose2d startPose = new Pose2d(24, 71, Math.toRadians(270));
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        robot.servos.Flick.setPosition(0.4);
        doTheCvThing();
        robot.pause(1200);
        robot.setPoseEstimate(startPose);
        switch (spike){
            case 1:
                robot.lighting.blinkMagenta();
                doSpike1();
                break;
            case 2:
                robot.lighting.blinkCyan();
                doSpike2();
                break;
            case 3:
                robot.lighting.blinkGreen();
                doSpike3();
                break;
        }
        //parkr();

    }
    public void doSpike1(){
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(28)
                .build();
        startPose = forward_1.end();
        Trajectory left_1 = robot.trajectoryBuilder(startPose)
                .strafeLeft(6)
                .build();
        startPose = left_1.end();
        Trajectory right_1 = robot.trajectoryBuilder(startPose)
                .strafeRight(6)
                .build();
        startPose = right_1.end();
        Trajectory forward_2 = robot.trajectoryBuilder(startPose)
                .forward(20)
                .build();
        startPose = forward_2.end();
        Trajectory left_2 = robot.trajectoryBuilder(startPose)
                .strafeLeft(95)
                .build();
        startPose = forward_2.end();
        robot.followTrajectory(forward_1);
        robot.followTrajectory(left_1);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_1);
        robot.followTrajectory(forward_2);
        robot.followTrajectory(left_2);
        robot.pause(500);
    }
    public void doSpike2(){
        Trajectory right_1  = robot.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();
        startPose = right_1.end();
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(6)
                .build();
        startPose = forward_1.end();
        Trajectory spline_1  = robot.trajectoryBuilder(forward_1.end())
                .splineTo(new Vector2d(startPose.getX()+5, startPose.getY()-21), startPose.getHeading()-Math.toRadians(90))
                .build();
        startPose = spline_1.end();
        Trajectory right_2 = robot.trajectoryBuilder(startPose)
                .strafeRight(4)
                .build();
        startPose = right_2.end();
        Trajectory forward  = robot.trajectoryBuilder(startPose)
                .forward(12)
                .build();
        startPose = forward.end();
        Trajectory left_1  = robot.trajectoryBuilder(startPose)
                .strafeLeft(30)
                .build();
        startPose = left_1.end();
        Trajectory back_1 = robot.trajectoryBuilder(startPose)
                .back(100)
                .build();
        startPose = back_1.end();
        Trajectory left_micro  = robot.trajectoryBuilder(startPose)
                .strafeRight(7)
                .build();
        startPose = left_micro.end();

        robot.followTrajectory(right_1);
        robot.followTrajectory(forward_1);
        robot.followTrajectory(spline_1);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_2);
        robot.pause(500);
        robot.followTrajectory(forward);
        robot.followTrajectory(left_1);
        robot.followTrajectory(back_1);
        robot.followTrajectory(left_micro);
    }
    public void doSpike3(){
        Trajectory spline_1  = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(startPose.getX()-5, startPose.getY()-29), startPose.getHeading()-Math.toRadians(180))
                .build();
        startPose = spline_1.end();
        Trajectory right_1 = robot.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();
        startPose = right_1.end();
        Trajectory right_2 = robot.trajectoryBuilder(startPose)
                .strafeRight(3)
                .build();
        startPose = right_2.end();
        Trajectory back_2 = robot.trajectoryBuilder(startPose)
                .back(18)
                .build();
        startPose = back_2.end();
        Trajectory right_3 = robot.trajectoryBuilder(startPose)
                .strafeRight(90)
                .build();
        startPose = right_3.end();

        robot.followTrajectory(spline_1);
        robot.followTrajectory(right_1);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_2);
        robot.followTrajectory(back_2);
        robot.followTrajectory(right_3);
        robot.pause(500);
    }

    /*public void parkr(){
        Trajectory rightpark = robot.trajectoryBuilder(startPose)
                .strafeRight(32)
                .build();
        startPose = rightpark.end();

        Trajectory creepbackward2 = robot.trajectoryBuilder(startPose)
                .back(6)
                .build();
        startPose = creepbackward2.end();
        //robot.followTrajectory(rightpark);
        robot.pause(300);
        //robot.followTrajectory(creepbackward2);
    }*/
    private ElapsedTime runtime = new ElapsedTime();
    public void doTheCvThing(){
        robot.vision.open();
        //robot.pause(2000);// hoping this is enough to get the camera booted up
        runtime.reset();
        while (runtime.seconds() < 4){
            spike = robot.vision.getSpike();
        }
        robot.pause(50);
        robot.vision.close();
    }




}
