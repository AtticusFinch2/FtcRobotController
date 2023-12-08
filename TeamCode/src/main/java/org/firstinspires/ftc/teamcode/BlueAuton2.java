package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Vector;

@Autonomous(name = "BlueAuton2")
public class BlueAuton2 extends LinearOpMode {
    MainRobot robot;
    int spike = 2;
    public Pose2d startPose = new Pose2d(24, 70, Math.toRadians(90));
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        robot.servos.Rotator.setPosition(0.24);
        doTheCvThing();
        robot.pause(1200);

        spike = 1; // TODO: REMOVE WHEN DONE MAKING TRAJECTORIES FOR EACH SPIKE
        switch (spike){
            case 1:
                doSpike1();
            case 2:
                doSpike2();
            case 3:
                //doSpike3(); /**DO NOT RUN*/
        }
        pixel1();
        parkr();

    }
    public void doSpike1(){
        robot.setPoseEstimate(startPose);
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(5)
                .build();
        startPose = forward_1.end();
        Trajectory left_1 = robot.trajectoryBuilder(startPose)
                .strafeLeft(5)
                .build();
        startPose = left_1.end();
        Trajectory spline_1 = robot.trajectoryBuilder(left_1.end())
                .splineTo(new Vector2d(startPose.getX()+0,startPose.getY()+20), Math.toRadians(0))
                .build();
        startPose = spline_1.end();
        Trajectory right_1 = robot.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();
        startPose = right_1.end();
        Trajectory back = robot.trajectoryBuilder(startPose)
                .back(20)
                .build();
        startPose = back.end();

        robot.followTrajectory(forward_1);
        robot.followTrajectory(left_1);
        robot.followTrajectory(spline_1);
        robot.servos.Purps.setPosition(0);
        robot.followTrajectory(right_1);
        robot.followTrajectory(back);
        robot.pause(500);
    }

    public void doSpike2(){
        robot.setPoseEstimate(startPose);
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(5)
                .build();
        startPose = forward_1.end();
        Trajectory spline_1  = robot.trajectoryBuilder(forward_1.end())
                .splineTo(new Vector2d(startPose.getX()+5, startPose.getY()+21), Math.toRadians(0))
                .build();
        startPose = spline_1.end();
        Trajectory backup  = robot.trajectoryBuilder(startPose)
                .back(38)
                .build();
        startPose = backup.end();

        robot.followTrajectory(forward_1);
        robot.followTrajectory(spline_1);
        robot.servos.Purps.setPosition(0);
        robot.followTrajectory(backup);
        robot.pause(500);
    }/*
    public void doSpike3(){// TODO
        robot.setPoseEstimate(startPose);
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(5)
                .build();
        startPose = forward_1.end();
        Trajectory spline_1  = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(startPose.getX()+5, startPose.getY()+15), startPose.getHeading()+Math.toRadians(180))
                .build();
        startPose = spline_1.end();
        Trajectory backup  = robot.trajectoryBuilder(startPose)
                .back(33)
                .build();
        startPose = backup.end();
        robot.followTrajectory(forward_1);
        robot.followTrajectory(spline_1);
        robot.servos.Purps.setPosition(1);
        robot.followTrajectory(backup);
        robot.pause(500);
    }*/
    public void pixel1() {
        Trajectory creepbackward = robot.trajectoryBuilder(startPose)
                .back(10)
                .build();
        startPose = creepbackward.end();
        Trajectory creepforward = robot.trajectoryBuilder(startPose)
                .forward(8)
                .build();
        startPose = creepforward.end();

        robot.slides.setSlidesPower(1.0);
        robot.pause(1000);
        robot.servos.Backhand.setPosition(0.5); //open
        robot.pause(1000);
        robot.servos.Rotator.setPosition(0.48);
        robot.pause(1000);
        robot.followTrajectory(creepbackward);
        robot.pause(2000);
        robot.servos.Dropper.setPosition(0.3);
        robot.pause(500);
        //robot.followTrajectory(creepforward);
        robot.pause(1000);
        robot.servos.Dropper.setPosition(0.7);
        robot.pause(200);
        robot.servos.Backhand.setPosition(0.0); //close
        robot.pause(300);
        robot.servos.Rotator.setPosition(0.24);
        robot.pause(300);
        robot.slides.setSlidesPower(-1.0);
        robot.pause(1200);
        robot.slides.setSlidesPower(0);
    }
    public void parkr(){
        Trajectory rightpark = robot.trajectoryBuilder(startPose)
                .strafeRight(32)
                .build();
        startPose = rightpark.end();

        Trajectory creepbackward2 = robot.trajectoryBuilder(startPose)
                .back(6)
                .build();
        startPose = creepbackward2.end();
        robot.followTrajectory(rightpark);
        robot.pause(300);
        robot.followTrajectory(creepbackward2);
    }
    public void doTheCvThing(){
        robot.vision.open();
        robot.pause(100);// hoping this is enough to get the camera booted up
        spike = robot.vision.getSpike();
        robot.pause(50);
        robot.vision.close();
    }




}
