package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;

public class Slider {
    //includes both bucket servo and motor
    //to do list:
    //- getter for slider motor encoder
    //- add hardware map to constructor
    //- setter for bucket and slider
    private HardwareMap hardwareMap;
    private DcMotor sliderMotor;
    private Servo bucketServo;

    public Slider(HardwareMap hwp) {
        //instantiates hardware map and devices
        this.hardwareMap = hwp;
        this.sliderMotor = hardwareMap.get(DcMotor.class, "PulleyMotor");
        this.bucketServo = hardwareMap.get(Servo.class, "BucketServo");

        //configures devices
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setPower(0);

        bucketServo.setPosition(Constants.BUCKET_RESET_POS);
    }

    public int getSliderLength() {
        return sliderMotor.getCurrentPosition();
    }

    public void extendSliderTo(){ //full extension is default
        sliderMotor.setTargetPosition(Constants.SLIDER_FULL_EXTEND_TICKS);
    }

    public void extendSliderTo(int ticks){ //custom extension
        sliderMotor.setTargetPosition(ticks);
    }

    public void tiltBucket(){
        bucketServo.setPosition(Constants.BUCKET_FULL_TILT_POS);
    }
}
