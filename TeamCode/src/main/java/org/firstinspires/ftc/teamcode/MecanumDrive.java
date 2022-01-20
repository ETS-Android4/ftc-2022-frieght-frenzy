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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MecanumDrive", group="Linear Opmode")
@Disabled
public class MecanumDrive extends OpMode {

    MyHardwareV2 robot = new MyHardwareV2();

    double RFpower = 0;
    double RBpower = 0;
    double LFpower = 0;
    double LBpower = 0;
    double IntakePower = 0;
    double PulleyPower = 0;

    @Override
    public void init() {
        robot.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        //RFpower = 0;
        //RBpower = 0;
        //LFpower = 0;
        //LBpower = 0;
        IntakePower = 0;
        PulleyPower = 0;
        dPadDrive(0.5);
        tankDrive();
        intake(0.3);
        pulley(0.5, 0.4);
        bucket();
    }

    //Controls the bucket servo using the X gamepad button
    public void bucket(){
        if(gamepad1.x){
            robot.BucketServo.setPosition(0.275);
        } else{
            robot.BucketServo.setPosition(1);
        }
    }

    //Controls the pulley motor using the Y and A gamepad buttons
    public void pulley(double powerGoingUp, double powerGoingDown){
        if(gamepad1.y){
            PulleyPower = powerGoingUp;
        }
        if(gamepad1.a){
            PulleyPower = -powerGoingDown;
        }

        robot.PulleyMotor.setPower(PulleyPower);
    }

    //Controls the intake motor using the B gamepad button
    public void intake(double turningPower){
        if(gamepad1.b){
            IntakePower = turningPower;
        }

        robot.IntakeMotor.setPower(IntakePower);
    }

    //Moves the robot using tank drive using the left and right sticks
    public void tankDrive(){
        //double rightXAxis = gamepad1.right_stick_x;
        double rightYAxis = gamepad1.right_stick_y;
        //double leftXAxis = gamepad1.left_stick_x;
        double leftYAxis = gamepad1.left_stick_y;

        RFpower = -rightYAxis;
        RBpower = -rightYAxis;
        LFpower = -leftYAxis;
        LBpower = -leftYAxis;

        robot.RFMotor.setPower(RFpower);
        robot.RBMotor.setPower(RBpower);
        robot.LFMotor.setPower(LFpower);
        robot.LBMotor.setPower(LBpower);
    }

    //Moves the robot using tank drive + strafing using the dpad
    public void dPadDrive(double drivePow){
        if(gamepad1.dpad_up){
            RFpower = drivePow;
            RBpower = drivePow;
            LFpower = drivePow;
            LBpower = drivePow;
        }
        else if(gamepad1.dpad_down){
            RFpower = -drivePow;
            RBpower = -drivePow;
            LFpower = -drivePow;
            LBpower = -drivePow;
        }
        else if(gamepad1.dpad_right){
            RFpower = -drivePow;
            RBpower = drivePow;
            LFpower = drivePow;
            LBpower = -drivePow;
        }
        else if(gamepad1.dpad_left){
            RFpower = drivePow;
            RBpower = -drivePow;
            LFpower = -drivePow;
            LBpower = drivePow;
        }
        else {
            RFpower = 0;
            RBpower = 0;
            LFpower = 0;
            LBpower = 0;
        }

        robot.RFMotor.setPower(RFpower);
        robot.RBMotor.setPower(RBpower);
        robot.LFMotor.setPower(LFpower);
        robot.LBMotor.setPower(LBpower);
    }

}
