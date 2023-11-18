package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous(name = "BluePark")
public class BluePark extends LinearOpMode {
    MainRobot robot;
    int spike =2;
    public Pose2d startPose = new Pose2d(24,-70,Math.toRadians(-90));
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap);
        waitForStart();
        robot.pause(1200);
        doTheCvThing();
        robot.setPoseEstimate(startPose);
        Trajectory strafeleft = robot.trajectoryBuilder(startPose)
                .strafeLeft(28)
                .build();
        startPose = strafeleft.end();

        robot.followTrajectory(strafeleft);


    }

    public void doTheCvThing() {
        robot.visionblue.open();
        robot.pause(100);// hoping this is enough to get the camera booted up
        spike = robot.visionblue.getSpike();
        robot.visionblue.close();
    }


}