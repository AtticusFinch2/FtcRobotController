package org.firstinspires.ftc.teamcode.Components;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Servos {

    public CRServo Sweep;
    public Servo Airplane;
    public Servo Flick;
    public Servo ClawL, ClawR;
    public Servo Purps;

    public Servos (HardwareMap hardwareMap) {
        Sweep = hardwareMap.get(CRServo.class, "sweeper");
        Flick = hardwareMap.get(Servo.class, "flick");
        ClawL = hardwareMap.get(Servo.class, "clawl");
        ClawR = hardwareMap.get(Servo.class, "clawr");
        Purps = hardwareMap.get(Servo.class, "purps");
        Airplane = hardwareMap.get(Servo.class, "airplane");
        closeClaw();
        Flick.setPosition(0.4);
        Sweep.setPower(0.0);
        Airplane.setPosition(0.5);
        Purps.setPosition(0.35);
    }

    public void openClaw() {
        ClawL.setPosition(0.5);
        ClawR.setPosition(0.7);
    }

    public void closeClaw() {
        ClawL.setPosition(0.1);
        ClawR.setPosition(1.0);
    }


}
