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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name = "ValuesTest") // Preliminary servo value testing
public class AutonTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    MainRobot robot;
    float x, y;
    double servoVal = 0.96;
    double lastCreepChange = runtime.seconds();
    @Override
    public void runOpMode() {

        robot = new MainRobot(hardwareMap, true);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            telemetry.addData("dir", "xin (%.2f), yin (%.2f)", x, y);
            if (gamepad1.dpad_up && runtime.seconds() - lastCreepChange > 0.15) {
                servoVal += 0.01;
                lastCreepChange = runtime.seconds();
            } else if (gamepad1.dpad_down && runtime.seconds() - lastCreepChange > 0.15) {
                servoVal -= 0.01;
                lastCreepChange = runtime.seconds();
            }
            telemetry.addData("servo", "val (%.2f)", servoVal);
            telemetry.update();
        }

    }
}