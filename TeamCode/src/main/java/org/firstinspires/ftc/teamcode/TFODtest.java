package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.MainRobot;

import java.util.List;

@Autonomous(name = "TFOD test")
public class TFODtest extends LinearOpMode {
    MainRobot robot;
    int spike = 2;

    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        doTheCvThing();
        while (opModeIsActive()) {
            telemetryTfod();
            telemetry.update();
        }
    }
    public void doTheCvThing(){
        robot.vision.open();
        robot.pause(100);// hoping this is enough to get the camera booted up
        spike = robot.vision.getSpike();
        robot.pause(50);
        //robot.vision.close();
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = robot.vision.tfod.getRecognitions();
        spike = robot.vision.getSpike();
        telemetry.addData("spike", spike);
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- TopLeft", "%.0f / %.0f", recognition.getLeft(), recognition.getTop());
            telemetry.addData("- BotRight", "%.0f / %.0f", recognition.getRight(), recognition.getBottom());
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}

