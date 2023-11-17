package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ThreadPool;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.ExecutorService;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Components.Vision;
import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.Components.Servos;


@Config
public class MainRobot extends SampleMecanumDrive{

    public Vision vision;
    public Slides slides;
    public Servos sweeper;
    public MainRobot (HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        vision = new Vision(hardwareMap);
        slides = new Slides(hardwareMap);
        sweeper =new Servos(hardwareMap);
    }

    public void pause(long time) {
        long initTime = System.currentTimeMillis();
        while(initTime + time > System.currentTimeMillis() && !Thread.currentThread().isInterrupted()) { }
    }

}
