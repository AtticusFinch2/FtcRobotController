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
@Autonomous(name = "BlueAuton1", group = "BlueWorking")
public class BlueAuton1 extends LinearOpMode {
    MainRobot robot;
    int spike;
    @Override
    public void runOpMode(){
        robot = new MainRobot(hardwareMap);
        waitForStart();
        doTheCvThing();
        while(opModeIsActive()) {
            /** HI KATELYN
             *        ok heres the auton file
             *        the spike that the object is on is stored into the spike variable right after init
             *        1 is left side, 3 is right side, 2 is middle.
             *        i cooked up the vision class, but you shouldn't have to call it
             *        all you do is read off the spike variable, dw ab anything else
             *
             */
        }
    }


    public void doTheCvThing(){
        robot.visionblue.open();
        robot.pause(100);// hoping this is enough to get the camera booted up
        robot.visionblue.findFirstBox();
        spike = robot.visionblue.getSpike();
        robot.visionblue.close();
    }

}
