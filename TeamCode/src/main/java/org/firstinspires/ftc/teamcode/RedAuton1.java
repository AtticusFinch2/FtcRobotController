package org.firstinspires.ftc.teamcode;

import android.util.Size;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous(name = "RedAuton1", group = "RedWorking")
public class RedAuton1 extends LinearOpMode {
    MainRobot robot;
    int spike;
    @Override
    public void runOpMode(){
        robot = new MainRobot(hardwareMap);
        waitForStart();
        doTheCvThing();
        while(opModeIsActive()) {
            /** HI KATELYN
             *        same thing as BlueAuton1.java
             */
        }
    }


    public void doTheCvThing(){
        robot.visionred.open();
        robot.pause(500);// hoping this is enough to get the camera booted up
        robot.visionred.findFirstBox();
        spike = robot.visionred.getSpike();
        robot.visionred.close();
    }

}
