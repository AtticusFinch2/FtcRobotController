package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


class ClawMachineRobot {
    public DcMotor LMY = null;
    public DcMotor RMY = null;
    public DcMotor Sliderx = null;
    public DcMotor Vertical = null;
    public Servo LServo = null;
    public Servo RServo = null;
    public DigitalChannel front;
    public DigitalChannel back;

    public static double Claw_HomeL = 0.30;
    public static double Claw_HomeR = 0.30;

    HardwareMap map = null;

    public void init(HardwareMap maps){
        map = maps;
        LMY = maps.get(DcMotor.class, "lmy1");
        RMY = maps.dcMotor.get("rmy1");
        Sliderx = maps.dcMotor.get("slider");
        Vertical = maps.dcMotor.get("vertical");
        LServo = maps.servo.get("lservo");
        RServo = maps.servo.get("rservo");
        front = maps.digitalChannel.get("front");
        back = maps.digitalChannel.get("back");

        LMY.setDirection(DcMotorSimple.Direction.FORWARD);
        RMY.setDirection(DcMotorSimple.Direction.REVERSE);
        Sliderx.setDirection(DcMotorSimple.Direction.FORWARD);
        Vertical.setDirection(DcMotorSimple.Direction.REVERSE);


        LMY.setPower(0.0);
        RMY.setPower(0.0);
        Sliderx.setPower(0.0);
        Vertical.setPower(0.0);
        LServo.setPosition(Claw_HomeL);
        RServo.setPosition(Claw_HomeR);

        LMY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Sliderx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front.setMode(DigitalChannel.Mode.INPUT);
        back.setMode(DigitalChannel.Mode.INPUT);

    }
}

@TeleOp(name = "ClawMachineTeleOP", group = "Claw_Machine")
public class ClawMachine extends LinearOpMode{
    ClawMachineRobot robot = new ClawMachineRobot();
    double y;
    double x;
    double ver_pos = 0.0;

    final double vmos = 1;
    final double claw_speed = 0.45;

    boolean hasGoneDown = false;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()){
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;


            if (gamepad1.a && !(hasGoneDown)) {
                openClaw();
                clawDown();
                sleep(1000);
                closeClaw();
                clawUp();
                hasGoneDown = true;
            }
            else if (hasGoneDown && gamepad1.a){
                openClaw();
            }
            else{
                robot.LMY.setPower(y/3);
                robot.RMY.setPower(y/3);
                robot.Sliderx.setPower(x/2);
            }

            robot.Vertical.setPower(ver_pos);


            telemetry.addData("y", "%.2f",y);
            telemetry.addData("x","%.2f",x);

            telemetry.update();

            sleep(50);
        }
    }
    public void closeClaw(){
        robot.LServo.setPosition(0.00);
        robot.RServo.setPosition(0.70);
        sleep(1500);
    }
    public void openClaw(){
        robot.LServo.setPosition(0.30);
        robot.RServo.setPosition(0.30);
        sleep(1500);
    }
    public void clawDown(){
        robot.Vertical.setPower(-1);
        sleep(1300);
        robot.Vertical.setPower(0);
    }
    public void clawUp(){
        robot.Vertical.setPower(1);
        sleep(1400);
        robot.Vertical.setPower(0);
    }
}
