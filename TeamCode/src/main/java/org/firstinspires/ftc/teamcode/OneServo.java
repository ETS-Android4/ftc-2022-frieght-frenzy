package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="OneServo", group="Linear Opmode")
//@Disabled
public class OneServo extends OpMode{
    MyHardwareV2 robot = new MyHardwareV2();

    @Override
    public void init(){
        robot.initialize(hardwareMap);
    }

    @Override
    public void loop(){
        bucket();
    }

    public void bucket(){
        robot.BucketServo.setPosition(1);
    }
}
