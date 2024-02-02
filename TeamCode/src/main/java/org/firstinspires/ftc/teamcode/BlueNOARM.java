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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous(name = "BlueNOARM")
public class BlueNOARM extends LinearOpMode {
    MainRobot robot;
    int spike = 2;
    public Pose2d startPose = new Pose2d(24, 70, Math.toRadians(90));
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        //robot.servos.Rotator.setPosition(0.17);
        doTheCvThing();
        robot.pause(1200);
        robot.setPoseEstimate(startPose);
        //spike = 3;
        switch (spike){
            case 1:
                doSpike1();
                break;
            case 2:
                doSpike2();
                break;
            case 3:
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
                .strafeLeft(3)
                .build();
        startPose = left_1.end();
        Trajectory right_1 = robot.trajectoryBuilder(startPose)
                .strafeRight(3)
                .build();
        startPose = right_1.end();
        Trajectory forward_2 = robot.trajectoryBuilder(startPose)
                .forward(18)
                .build();
        startPose = forward_2.end();
        Trajectory left_2 = robot.trajectoryBuilder(startPose)
                .strafeLeft(38)
                .build();
        startPose = left_2.end();

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
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(7)
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
        robot.pause(500);
        robot.followTrajectory(backup);
        robot.pause(500);
    }
    public void doSpike3(){
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(5)
                .build();
        startPose = forward_1.end();
        Trajectory left_1 = robot.trajectoryBuilder(startPose)
                .strafeLeft(4)
                .build();
        startPose = left_1.end();
        Trajectory spline_1  = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(startPose.getX()+4, startPose.getY()+20), Math.toRadians(-90))
                .build();
        startPose = spline_1.end();
        Trajectory left_2 = robot.trajectoryBuilder(startPose)
                .strafeLeft(5)
                .build();
        startPose = left_2.end();
        Trajectory right_1  = robot.trajectoryBuilder(startPose)
                .strafeRight(40)
                .build();
        startPose = right_1.end();
        robot.followTrajectory(forward_1);
        robot.followTrajectory(left_1);
        robot.followTrajectory(spline_1);
        robot.followTrajectory(left_2);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_1);
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

        robot.followTrajectory(rightpark);
        robot.pause(300);
        robot.followTrajectory(creepbackward2);
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