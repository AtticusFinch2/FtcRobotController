package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

@Autonomous(name = "BlueDoubleClose 2 PIXELS")
public class BlueDoubleClose extends LinearOpMode {
    MainRobot robot;
    int spike = 1;
    public Pose2d startPose = new Pose2d(16, 65, Math.toRadians(270));
    @Override
    public void runOpMode() {
        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        robot.servos.Flick.setPosition(0.35);
        doTheCvThing();
        robot.pause(1200);
        robot.setPoseEstimate(startPose);
        switch (spike){
            case 1:
                robot.lighting.blinkMagenta();
                doSpike1();
                pixel1(spike);
                break;
            case 2:
                robot.lighting.blinkCyan();
                doSpike2();
                pixel1(spike);
                break;
            case 3:
                robot.lighting.blinkGreen();
                doSpike3();
                pixel1(spike);
                break;
        }
        //pixel1(); //FIXME uncomment when confirm pixel1 works

    }
    public void doSpike1(){
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(31)
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
        Trajectory back_1 = robot.trajectoryBuilder(startPose)
                .back(16)
                .build();
        startPose = back_1.end();
        Trajectory revert = robot.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(38, 45, Math.toRadians(180)))
                .build();
        startPose = revert.end();

        robot.followTrajectory(forward_1);
        robot.followTrajectory(left_1);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_1);
        robot.followTrajectory(back_1);
        robot.followTrajectory(revert);
        robot.pause(500);
    }
//temp change
    public void doSpike2(){
        Trajectory spline_1  = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(14, 38), Math.toRadians(180))
                .build();
        startPose = spline_1.end();
        Trajectory left_1  = robot.trajectoryBuilder(startPose)
                .strafeLeft(2)
                .build();
        startPose = left_1.end();
        Trajectory right_1  = robot.trajectoryBuilder(startPose)
                .strafeRight(5)
                .build();
        startPose = right_1.end();
        Trajectory backup  = robot.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)))
                .build();
        startPose = backup.end();

        robot.followTrajectory(spline_1);
        robot.followTrajectory(left_1);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_1);
        robot.followTrajectory(backup);
        robot.pause(500);
    }
    public void doSpike3(){
        Trajectory forward_1  = robot.trajectoryBuilder(startPose)
                .forward(6)
                .build();
        startPose = forward_1.end();
        Trajectory left_1 = robot.trajectoryBuilder(startPose)
                .strafeLeft(14)
                .build();
        startPose = left_1.end();
        Trajectory spline_1  = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 40), Math.toRadians(90))
                .build();
        startPose = spline_1.end();
        Trajectory left_2 = robot.trajectoryBuilder(startPose)
                .strafeLeft(10)
                .build();
        startPose = left_2.end();
        Trajectory right_1  = robot.trajectoryBuilder(startPose)
                .strafeRight(10)
                .build();
        startPose = right_1.end();
        Trajectory backup  = robot.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(38, 33, Math.toRadians(180)))
                .build();
        startPose = backup.end();
        robot.followTrajectory(forward_1);
        robot.followTrajectory(left_1);
        robot.followTrajectory(spline_1);
        robot.followTrajectory(left_2);
        robot.servos.Purps.setPosition(0);
        robot.pause(500);
        robot.followTrajectory(right_1);
        robot.followTrajectory(backup);
        robot.pause(500);
    }

    public void pixel1(int param_spike){
        /*Trajectory creepbackward = robot.trajectoryBuilder(startPose)
                .back(10)
                .build();
        startPose = creepbackward.end();*/
        Trajectory creepbackward2 = robot.trajectoryBuilder(startPose)
                .back(5)
                .build();
        startPose = creepbackward2.end();
        Trajectory jigle1 = robot.trajectoryBuilder(startPose)
                .forward(3)
                .build();
        startPose = jigle1.end();
        Trajectory jigle2 = robot.trajectoryBuilder(startPose)
                .back(2)
                .build();
        startPose = jigle2.end();
        Trajectory left_park = robot.trajectoryBuilder(startPose)
                .strafeLeft(24 + ((2 - param_spike) * 6)) // 24 for middle, +- 6 for 1 and 3 respectively
                .build();
        startPose = left_park.end();
        Trajectory back_1 = robot.trajectoryBuilder(startPose)
                .back(14)
                .build();
        startPose = back_1.end();

        //robot.followTrajectory(creepbackward);
        robot.pause(200);
        robot.servos.Flick.setPosition(0.5); // flick pos to bring up
        robot.pause(200);
        robot.slides.setSlidesPower(0.8);
        robot.pause(1200);
        robot.slides.setSlidesPower(0);
        robot.servos.Flick.setPosition(0); // flick pos to score
        robot.pause(500);
        robot.followTrajectory(creepbackward2);
        robot.servos.openClaw();
        robot.pause(200);
        robot.followTrajectory(jigle1);
        robot.followTrajectory(jigle2);
        robot.pause(500);
        robot.servos.closeClaw();
        robot.pause(200);
        robot.servos.Flick.setPosition(0.5); //flick pos to bring down
        robot.pause(1000);
        robot.slides.setSlidesPower(-0.8);
        robot.pause(1100);
        robot.slides.setSlidesPower(0);
        robot.followTrajectory(left_park);
        robot.followTrajectory(back_1);

    }


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
