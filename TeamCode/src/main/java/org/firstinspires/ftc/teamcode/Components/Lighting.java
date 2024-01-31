package org.firstinspires.ftc.teamcode.Components;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Lighting {

    RevBlinkinLedDriver ledDriver;

    public Lighting(HardwareMap hardwareMap) {

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");

    }

    public void blinkOrange() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE); }
    public void blinkBlue() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA); }
    public void blinkYellow() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW); }
    public void blinkMagenta() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET); }
    public void blinkCyan() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN); }
    public void blinkBlack() { ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK); }


}