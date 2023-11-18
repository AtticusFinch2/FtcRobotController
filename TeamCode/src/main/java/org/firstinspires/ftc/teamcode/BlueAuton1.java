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
@Autonomous(name = "BlueAuton1", group = "BlueWorking")
public class BlueAuton1 extends LinearOpMode {
    MainRobot robot;
    int spike=2;
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap);
        waitForStart();
        doTheCvThing();
        while (opModeIsActive()) {
            robot.pause(1200);
            if (spike == 1) {
                Path1();
                break;
            } else if (spike == 2) {
                Path2();
                break;
            } else if (spike == 3) {
                Path3();
                break;
            }

        }

    }
    public void doTheCvThing(){
        robot.visionblue.open();
        robot.pause(100);// hoping this is enough to get the camera booted up
        spike = robot.visionblue.getSpike();
        robot.visionblue.close();
    }
    public Pose2d startPose = new Pose2d(24, 70, Math.toRadians(-180));
    public void Path1() {
        robot.setPoseEstimate(startPose);
        Trajectory turnstrafe1 = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(startPose.getX() + 5, startPose.getY()-18), Math.toRadians(-270))
                .build();
        startPose = turnstrafe1.end();

        Trajectory firstforward1 = robot.trajectoryBuilder(turnstrafe1.end())
                .forward(14)
                .build();
        startPose = firstforward1.end();

        Trajectory firstleft1 = robot.trajectoryBuilder(firstforward1.end())
                .strafeLeft(5)
                .build();
        startPose = firstleft1.end();

        robot.followTrajectory(turnstrafe1);
        robot.pause(300);
        robot.followTrajectory(firstforward1);
        robot.pause(300);
        robot.followTrajectory(firstleft1);
        robot.pause(300);
    }

    public void Path2() {
        robot.setPoseEstimate(startPose);
        Trajectory turnstrafe2 = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(startPose.getX() + 5, startPose.getY() - 18), Math.toRadians(-270))
                .build();
        startPose = turnstrafe2.end();

        Trajectory firstforward2 = robot.trajectoryBuilder(turnstrafe2.end())
                .forward(14)
                .build();
        startPose = firstforward2.end();

        Trajectory firstmiddle2 = robot.trajectoryBuilder(firstforward2.end())
                .strafeLeft(3)
                .build();
        startPose = firstmiddle2.end();

        robot.followTrajectory(turnstrafe2);
        robot.pause(300);
        robot.followTrajectory(firstforward2);
        robot.pause(300);
        robot.followTrajectory(firstmiddle2);
        robot.pause(300);

    }

    public void Path3() {
        robot.setPoseEstimate(startPose);
        Trajectory strafe3 = robot.trajectoryBuilder(startPose)
                .strafeRight(14)
                .build();
        startPose = strafe3.end();

        TrajectorySequence turn3 = robot.trajectorySequenceBuilder(strafe3.end())
                .turn(Math.toRadians(-90))
                .build();
        startPose = turn3.end();

        Trajectory firstright3 = robot.trajectoryBuilder(turn3.end())
                .strafeLeft(14)
                .build();
        startPose = firstright3.end();

        robot.followTrajectory(strafe3);
        robot.pause(300);
        robot.followTrajectorySequence(turn3);
        robot.pause(300);
        robot.followTrajectory(firstright3);
        robot.pause(300);

    }

    public void pixel1(){
        robot.setPoseEstimate(startPose);
        Trajectory creepforward = robot.trajectoryBuilder(startPose)
                .forward(3)
                .build();
        startPose = creepforward.end();

        Trajectory creepbackward = robot.trajectoryBuilder(startPose)
                .back(3)
                .build();
        startPose = creepbackward.end();

    /** brady: everything above is initial movement
     * MAKE SURE TO CALL PIXEL1 AT END OF EACH PATH
     * what needs to be done next is just universal for all paths (not much trajectory required):
     * slides go up all the way
     * back metal piece goes down
     * grabber goes to placing position
     * creep forward till it aligns with the backboard
     * servo latch turns
     * pause for some time
     * creep forward
     * back metal piece goes back up
     * grabber returns to neutral state
     * slides go all the way back down
     * creep forward if necessary to park
     * done
     */

    }

}
