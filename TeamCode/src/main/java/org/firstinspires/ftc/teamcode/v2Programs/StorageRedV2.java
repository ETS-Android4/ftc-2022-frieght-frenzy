package org.firstinspires.ftc.teamcode.v2Programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MyHardwareV2;

//@Disabled
@Autonomous(name = "StorageRedV2", group = "Storage")
public class StorageRedV2 extends LinearOpMode {

    //Creates new instance of HardwareMap
    MyHardwareV2 robot = new MyHardwareV2();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap); //Initializes all of the hardware for use

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

//        strafeRight(0.5, 100);
//        stopDriving(300);
//        turnCarousel(-0.6, 10);
//        driveStraight(0.1, 4500);
//        turnCarousel(0, 10);
//        strafeRight(0.5, 1050);
//        stopDriving(500);
//        driveStraight(0.5, 400);

        strafeRight(0.5, 100);
        stopDriving(300);
        turnCarousel(-0.6, 10);
        driveForward(0.1, 4500);
        turnCarousel(0, 10);
        strafeRight(0.5, 1200);
        stopDriving(500);
        driveForward(0.5, 400);

        stopDriving();
    }

    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void weirdMovement(double RF, double RB, double LF, double LB, int milliseconds) {
        robot.RFMotor.setPower(RF);
        robot.RBMotor.setPower(RB);
        robot.LFMotor.setPower(LF);
        robot.LBMotor.setPower(LB);

        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {

        }
    }

    public void driveForward(double power, int milliseconds) {
        robot.RFMotor.setPower(power);
        robot.RBMotor.setPower(power);
        robot.LFMotor.setPower(power);
        robot.LBMotor.setPower(power);

        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {

        }
    }

    public void driveStraight(double power, int duration) {
        resetAngle();

        robot.setAllPower(power);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && runtime.milliseconds() < duration) {
            while (opModeIsActive() && Math.abs(getAngle()) > 2) {
                double correction;
                if (getAngle() < 0) {
                    correction = 0.1 * Math.signum(power);
                    robot.setMotorPower(power - correction, power + correction, power - correction, power + correction);
                } else {
                    correction = 0.1;
                    robot.setMotorPower(power + correction, power - correction, power + correction, power - correction);
                }
            }
        }
        robot.setAllPower(0);
    }

//    public void strafeRight(double power, int milliseconds) {
//        robot.RFMotor.setPower(-power);
//        robot.RBMotor.setPower(power);
//        robot.LFMotor.setPower(power);
//        robot.LBMotor.setPower(-power);
//
//        ElapsedTime runtime = new ElapsedTime();
//        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
//
//        }
//    }

    public void strafeRight(double power, int duration) {
        resetAngle();

        robot.setMotorPower(power, -power, -power, power);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && runtime.milliseconds() < duration) {
            while (opModeIsActive() && Math.abs(getAngle()) > 2) {
                double correction;
                if (getAngle() < 0) {
                    correction = 0.1 * Math.signum(power);
                    robot.setMotorPower(power - correction, power + correction, power - correction, power + correction);
                } else {
                    correction = 0.1;
                    robot.setMotorPower(power + correction, power - correction, power + correction, power - correction);
                }
            }
        }
        robot.setAllPower(0);
    }

//    public void strafeLeft(double power, int milliseconds) {
//        robot.RFMotor.setPower(power);
//        robot.RBMotor.setPower(-power);
//        robot.LFMotor.setPower(-power);
//        robot.LBMotor.setPower(power);
//
//        ElapsedTime runtime = new ElapsedTime();
//        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {
//
//        }
//    }

    public void strafeLeft(double power, int duration) {
        resetAngle();

        robot.setMotorPower(-power, power, power, -power);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && runtime.milliseconds() < duration) {
            while (opModeIsActive() && Math.abs(getAngle()) > 2) {
                double correction;
                if (getAngle() < 0) {
                    correction = 0.1 * Math.signum(power);
                    robot.setMotorPower(power - correction, power + correction, power - correction, power + correction);
                } else {
                    correction = 0.1;
                    robot.setMotorPower(power + correction, power - correction, power + correction, power - correction);
                }
            }
        }
        robot.setAllPower(0);
    }

    public void stopDriving() {
        robot.RFMotor.setPower(0);
        robot.RBMotor.setPower(0);
        robot.LFMotor.setPower(0);
        robot.LBMotor.setPower(0);
    }

    public void stopDriving(double milliseconds) {
        robot.RFMotor.setPower(0);
        robot.RBMotor.setPower(0);
        robot.LFMotor.setPower(0);
        robot.LBMotor.setPower(0);

        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {

        }
    }

    public void turnCarousel(double turningPower, int milliseconds) {
        robot.DuckMotor.setPower(turningPower);

        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.milliseconds() < milliseconds)) {

        }
    }

}
