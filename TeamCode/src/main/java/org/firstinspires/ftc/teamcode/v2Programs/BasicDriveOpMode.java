package org.firstinspires.ftc.teamcode.v2Programs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BasicDriveOpMode extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private RevIMU imu;

    @Override
    public void init() {
        /* instantiate motors */
        fL = new Motor(hardwareMap, "LFMotor");
        fR = new Motor(hardwareMap, "RFMotor");
        bL = new Motor(hardwareMap, "LBMotor");
        bR = new Motor(hardwareMap, "RBMotor");

        fL.setInverted(true);
        fR.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);

        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);

        imu = new RevIMU(hardwareMap, "imu");
        imu.init();
    }
    
    @Override
    public void loop() {
        drive.driveFieldCentric(
            driverOp.getLeftX() * 2 / 3,
            driverOp.getLeftY()/2,
            driverOp.getRightX()/2,
                imu.getHeading()
        );
    }

}