package org.firstinspires.ftc.teamcode.Components;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Servos {

    public CRServo Sweep = null;
    public Servo Plane;
    public Servo Rotator;
    public Servo Dropper;
    public Servo Backhand;
    public Servo Purps;

    public Servos (HardwareMap hardwareMap) {
        Sweep = hardwareMap.get(CRServo.class, "sweeper");
        Plane = hardwareMap.get(Servo.class, "plane");
        Dropper = hardwareMap.get(Servo.class, "dropper");
        Rotator = hardwareMap.get(Servo.class, "rotator");
        Backhand = hardwareMap.get(Servo.class, "backhand");
        Purps = hardwareMap.get(Servo.class, "purps");
        Sweep.setPower(0.0);
        Plane.setPosition(0.0);
        Purps.setPosition(0.35);
    }



}
