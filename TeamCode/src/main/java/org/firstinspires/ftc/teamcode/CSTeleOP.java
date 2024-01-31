/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Lighting;

class CSbot{ // WE R NOT USING THIS IN TELEOP; This is so that autonomous modes can reference
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor LS = null;
    private DcMotor RS = null;
    private CRServo Sweep = null;
    private Servo Plane;
    HardwareMap map = null;
    public void init(HardwareMap maps){
        map = maps;
        BL  = maps.get(DcMotor.class, "bl");
        BR = maps.get(DcMotor.class, "br");
        FL  = maps.get(DcMotor.class, "fl");
        FR = maps.get(DcMotor.class, "fr");
        LS  = maps.get(DcMotor.class, "portMotor");
        RS = maps.get(DcMotor.class, "starboardMotor");
        Sweep = maps.get(CRServo.class, "sweeper");
        Plane = maps.get(Servo.class, "plane");

        BL.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);
        RS.setDirection(DcMotor.Direction.FORWARD);


        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        LS.setPower(0);
        RS.setPower(0);
        Sweep.setPower(0.0);
        Plane.setPosition(0.0);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

@TeleOp(name="CSTeleOP1", group="Linear OpMode")
//@Disabled THIS DISABLES THE OP MODE, LEAVE IT COMMENTED
public class CSTeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor LS = null;
    private DcMotor RS = null;
    private CRServo Sweep = null;
    private Servo Flick;
    private Servo ClawL;
    private Servo ClawR;
    private Servo Airplane;
    private DistanceSensor LeftColor, RightColor;
    private RevBlinkinLedDriver ledDriver;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        BL  = hardwareMap.get(DcMotor.class, "bl");
        BR = hardwareMap.get(DcMotor.class, "br");
        FL  = hardwareMap.get(DcMotor.class, "fl");
        FR = hardwareMap.get(DcMotor.class, "fr");
        LS  = hardwareMap.get(DcMotor.class, "portMotor");
        RS = hardwareMap.get(DcMotor.class, "starboardMotor");
        Sweep = hardwareMap.get(CRServo.class, "sweeper");
        Flick = hardwareMap.get(Servo.class, "flick");
        ClawL = hardwareMap.get(Servo.class, "clawl");
        ClawR = hardwareMap.get(Servo.class, "clawr");
        Airplane = hardwareMap.get(Servo.class, "airplane");
        LeftColor = hardwareMap.get(DistanceSensor.class, "LeftColor");
        RightColor = hardwareMap.get(DistanceSensor.class, "RightColor");
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");


        BL.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);
        RS.setDirection(DcMotor.Direction.FORWARD);


        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        LS.setPower(0);
        RS.setPower(0);
        Sweep.setPower(0.0);
        Flick.setPosition(0.0);
        Airplane.setPosition(0.5);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double x;
        double y;
        double vert;
        double flCurrentPower = 0;
        double frCurrentPower = 0;
        double blCurrentPower = 0;
        double brCurrentPower = 0;
        double lsCurrentPower = 0;
        double rsCurrentPower = 0;
        double tractionModifier = 0.02;
        double slideTractionModifier = 0.02;
        double lastCreepChange = runtime.seconds();
        double lastDropChange = runtime.seconds();
        double lastFlickChange = runtime.seconds();
        float stillModifier = 9f;
        double flPWR, frPWR, blPWR, brPWR, lsPWR, rsPWR;
        double lastTrimChange = runtime.seconds();
        int flick_position = 1; //0: down, 1: raised, 2: middle, 3: up
        boolean sweep_on = false;
        Airplane.setPosition(0.5);
        boolean creeping = false;
        boolean dropping = false;
        boolean open_arm = false;
        boolean open_finger = true;
        double speed = 1;//this affects how touchy the stick is to input;
        //             still goes to full power regardless
        float rx;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            x=0;
            y=0;
            flPWR=0;
            frPWR=0;
            blPWR=0;
            brPWR = 0;
            lsPWR = 0;
            rsPWR = 0;

            //This uses basic math to combine motions and is easier to drive straight.
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            vert = gamepad1.left_trigger - gamepad1.right_trigger;
            if (gamepad1.left_trigger > 0.95 && gamepad1.right_trigger > 0.95) {
                vert = -0.15;
            }
            telemetry.addData("dir", "xin (%.2f), yin (%.2f)", x, y);
            x = Range.clip(x, -1.0, 1.0) ; // gives values between -1 and 1, useful later
            y = Range.clip(y, -1.0, 1.0) ;
            vert = Range.clip(vert,-1.0,1.0);


            //convert circular plane to rectangular plane,
            // such that (x,y) : (1/sqrt(2),1/sqrt(2)) => (x,y) : (0.5,0.5)
            //           (x,y) : (1/sqrt(2),-1/sqrt(2)) => (x,y) : (0.5,-0.5)
            //           (x,y) : (-1/sqrt(2),-1/sqrt(2)) => (x,y) : (-0.5,-0.5)
            //           (x,y) : (-1/sqrt(2),1/sqrt(2)) => (x,y) : (-0.5,0.5)

            x = Math.asin(x)/(Math.PI/2);
            y = Math.asin(y)/(Math.PI/2);//convert to angle then divide by 90degrees(pi/2)
            telemetry.addData("dir", "x1 (%.2f), y1 (%.2f)", x, y);

            /*
            //creep (i'm a weirdoooooo what the hell am i doin' here)
            if (gamepad1.dpad_up) {
                y=0.3;
            } else if (gamepad1.dpad_down) {
                y=-0.3;
            } else if (gamepad1.dpad_left) {
                x=-0.3;
            } else if (gamepad1.dpad_right) {
                x=0.3;
            }
            */

            /*
            if (gamepad1.dpad_up) {
            } else if (gamepad1.dpad_down) {
            } else if (gamepad1.a) {
            }
            */
            if (gamepad1.dpad_left && runtime.seconds() - lastTrimChange > 0.15) {
                lastTrimChange = runtime.seconds();
            }
            if (gamepad1.dpad_right && runtime.seconds() - lastTrimChange > 0.15) {
                lastTrimChange = runtime.seconds();
            }
            if (gamepad1.b && runtime.seconds() - lastDropChange > 0.2) {
                open_finger = !open_finger;
                lastDropChange = runtime.seconds();
            }
            if (gamepad1.dpad_left && runtime.seconds() - lastDropChange > 0.15) {
                flick_position = 0;
            }
            if (gamepad1.dpad_down && runtime.seconds() - lastDropChange > 0.15) {
                flick_position = 1;
            }
            if (gamepad1.dpad_right && runtime.seconds() - lastDropChange > 0.15) {
                flick_position = 2;
            }
            if (!open_finger) { //open
                ClawL.setPosition(0.5);
                ClawR.setPosition(0.6);
            } else{ //closed
                ClawL.setPosition(0.1);
                ClawR.setPosition(1.0);
            }
            if (flick_position == 0) {
                Flick.setPosition(1.00);
            } else if (flick_position == 1) {
                Flick.setPosition(0.97);
            } else if (flick_position == 2) {
                Flick.setPosition(0.6);
            }
            //do smth to drive with x and y
            telemetry.addData("dir", "x2 (%.2f), y2 (%.2f)", x, y);
            if (Math.abs(x)> 0.03 || Math.abs(y)>0.03) { // if input is not due to stick drift
                if (Math.abs(x)<0.03) {x=0;} //if x is too low don't use it
                if (Math.abs(y)<0.03) {y=0;} //if y is too low don't use it

                // front left and back right give same power
               /*
               what to do with x and y: (with our drivebase)
                    Y
                a   b   c          a: no movement  b:forward      c:fast forward
                    |
              X_d___e___f_         d: backward     e:no movement  f:forward
                    |
                g   h   i          g:fast backward h:backward     i:no movement

                  use y+x to approximate this (from y+x=0)
                */
                flPWR = Range.clip(speed * (y+x), -1,1);
                brPWR = Range.clip(speed * (y+x), -1,1);


                // front right and back left give same power,
               /*
               what to do with x and y: (with our drivebase)
                a   b   c          a: fast forward b:forward       c:no movement
                    |
               _d___e___f_         d: forward      e:no movement   f:backward
                    |
                g   h   i          g:no movement   h:backward      i:fast backward
                  use y-x to approximate this (from y-x=0)
                */
                frPWR = Range.clip(speed * (y-x), -1,1);
                blPWR = Range.clip(speed * (y-x), -1,1);

            }


            rx = gamepad1.right_stick_x;//this is very touchy so it is divided by 4
            if (Math.abs(rx) > 0.05) { // we do a little spinning
                if (Math.abs(x) > 0.03 || Math.abs(y) > 0.03 ) {
                    flPWR += rx;
                    blPWR += rx;
                    frPWR -= rx;
                    brPWR -= rx;
                } else {
                    flPWR += rx * stillModifier;
                    blPWR += rx * stillModifier;
                    frPWR -= rx * stillModifier;
                    brPWR -= rx * stillModifier;
                }
            }

            lsPWR = vert;
            rsPWR = vert;

            flPWR = Range.clip(flPWR, -1.0, 1.0) ;
            frPWR = Range.clip(frPWR, -1.0, 1.0) ;
            blPWR = Range.clip(blPWR, -1.0, 1.0) ;
            brPWR = Range.clip(brPWR, -1.0, 1.0) ;

            lsPWR = Range.clip(lsPWR, -1.0, 1.0) ;
            rsPWR = Range.clip(rsPWR, -1.0, 1.0) ;
            // Send calculated power to wheels

            if (gamepad1.right_bumper && gamepad1.left_bumper || !gamepad1.right_bumper && !gamepad1.left_bumper) {
                Sweep.setPower(0.0);
                sweep_on = false;
            } else if (gamepad1.right_bumper) {
                Sweep.setPower(1.0);
                sweep_on = true;
            } else if (gamepad1.left_bumper) {
                Sweep.setPower(-1.0);
                sweep_on = true;
            }

            if (gamepad2.right_bumper && gamepad2.left_bumper || !gamepad2.right_bumper && !gamepad2.left_bumper) {
                Sweep.setPower(0.0);
                sweep_on = false;
            } else if (gamepad2.right_bumper) {
                Sweep.setPower(1.0);
                sweep_on = true;
            } else if (gamepad2.left_bumper) {
                Sweep.setPower(-1.0);
                sweep_on = true;
            }

            if (gamepad1.dpad_up && runtime.seconds() - lastCreepChange > 0.1) {
                creeping = !creeping;
                lastCreepChange = runtime.seconds();
            }

            if (creeping) {
                flPWR /= 3;
                frPWR /= 3;
                blPWR /= 3;
                brPWR /= 3;
            }

            if (gamepad1.y){
                Airplane.setPosition(0.0);
            }


            flCurrentPower -= Range.clip(flCurrentPower - (0.95*flPWR),-tractionModifier,tractionModifier);
            frCurrentPower -= Range.clip(frCurrentPower - frPWR,-tractionModifier,tractionModifier);
            blCurrentPower -= Range.clip(blCurrentPower - blPWR,-tractionModifier,tractionModifier);
            brCurrentPower -= Range.clip(brCurrentPower - brPWR,-tractionModifier,tractionModifier);
            flCurrentPower = Range.clip(flCurrentPower,-1,1);
            frCurrentPower = Range.clip(frCurrentPower,-1,1);
            blCurrentPower = Range.clip(blCurrentPower,-1,1);
            brCurrentPower = Range.clip(brCurrentPower,-1,1);


            lsCurrentPower -= Range.clip(lsCurrentPower - lsPWR,-slideTractionModifier,slideTractionModifier);
            rsCurrentPower -= Range.clip(rsCurrentPower - rsPWR,-slideTractionModifier,slideTractionModifier);
            lsCurrentPower = Range.clip(lsCurrentPower,-1,1);
            rsCurrentPower = Range.clip(rsCurrentPower,-1,1);

            FL.setPower(flCurrentPower);
            FR.setPower(frCurrentPower);
            BL.setPower(blCurrentPower);
            BR.setPower(brCurrentPower * 0.80);
            LS.setPower(lsCurrentPower);
            RS.setPower(rsCurrentPower * 0.6);

            blinkFromPixels(numOfPixelsInGuide());


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("fl", Double.toString(flPWR));
            telemetry.addData("fr", Double.toString(frPWR));
            telemetry.addData("bl", Double.toString(blPWR));
            telemetry.addData("br", Double.toString(brPWR));
            telemetry.addData("ls", Double.toString(lsPWR));
            telemetry.addData("rs", Double.toString(rsPWR));
            telemetry.addData("sweeper", Boolean.toString(sweep_on));
            telemetry.addData("Airplane",Double.toString(Airplane.getPosition()));
            telemetry.addData("ClawL",Double.toString(ClawL.getPosition()));
            telemetry.addData("ClawR",Double.toString(ClawR.getPosition()));
            telemetry.addData("Flick",Double.toString(Flick.getPosition()));
            telemetry.addData("DistL",LeftColor.getDistance(DistanceUnit.CM));
            telemetry.addData("DistR",RightColor.getDistance(DistanceUnit.CM));
            telemetry.addData("Luancher_pos",Double.toString(Airplane.getPosition()));
            telemetry.update();
        }
    }
    public int numOfPixelsInGuide(){
        int a = 0;
        if (LeftColor.getDistance(DistanceUnit.CM) < 2) {
            a++;
        }
        if (RightColor.getDistance(DistanceUnit.CM) < 2) {
            a++;
        }
        return a;
    }
    public void blinkFromPixels(int n){
        switch (n){
            case 0:
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
            case 1:
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            case 2:
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
    }
}

