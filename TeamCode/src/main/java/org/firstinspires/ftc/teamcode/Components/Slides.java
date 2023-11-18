package org.firstinspires.ftc.teamcode.Components;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Slides {

    public DcMotor LS, RS;

    public Slides (HardwareMap hardwareMap) {
        LS = hardwareMap.get (DcMotor.class, "portMotor");
        RS = hardwareMap.get (DcMotor.class, "starboardMotor");
        LS.setDirection(DcMotor.Direction.REVERSE);
        RS.setDirection(DcMotor.Direction.FORWARD);
        LS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlidesPower (double power) {
        LS.setPower(power);
        RS.setPower(power*0.6);
    }

}
