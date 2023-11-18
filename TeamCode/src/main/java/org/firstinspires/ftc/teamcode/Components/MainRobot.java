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

    public Vision visionblue;
    public Vision visionred;
    public Slides slides;
    public Servos servos;
    public MainRobot (HardwareMap hardwareMap) {
        //calls SampleMecanumDrive
        super(hardwareMap);

        //cv
        visionblue = new Vision(hardwareMap, true);
        visionblue.close(); // dont put strain on the cpu
        visionred = new Vision(hardwareMap, false);
        visionred.close();



        //everything else
        slides = new Slides(hardwareMap);
        servos =new Servos(hardwareMap);
    }

    public void pause(long time) { //time in ms
        long initTime = System.currentTimeMillis();
        while(initTime + time > System.currentTimeMillis() && !Thread.currentThread().isInterrupted()) { }
    }

}
