package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Gyro Test", group="Pushbot")
public class Gyro extends LinearOpMode {

    MyHardwareV2 robot = new MyHardwareV2();
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);

        waitForStart();

        strafeRight(0.5, 5000);
    }

    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if(error > 100){
            error -= 360;
        } else if(error < -100){
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle(){
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1){
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
        }
        robot.setAllPower(0);
    }

    public void driveStraight(double power, int duration){
        resetAngle();

        robot.setAllPower(power);
        ElapsedTime runtime = new ElapsedTime();
        while(opModeIsActive() && runtime.milliseconds() < duration){
            while(opModeIsActive() && Math.abs(getAngle()) > 2){
                double correction;
                if(getAngle() < 0){
                    correction = 0.1 * Math.signum(power);
                    robot.setMotorPower(power - correction, power + correction, power - correction, power + correction);
                }
                else{
                    correction = 0.1;
                    robot.setMotorPower(power + correction, power - correction, power + correction, power - correction);
                }
            }
        }
        robot.setAllPower(0);
    }

    public void strafe(double power, int duration){
        resetAngle();

        robot.setMotorPower(power, -power, -power, power);
        ElapsedTime runtime = new ElapsedTime();
        double correction;
        while(opModeIsActive() && runtime.milliseconds() < duration){
            while(opModeIsActive() && Math.abs(getAngle()) > 2){
                correction = 0;
                if(getAngle() < 0){
                    correction += 0.1;
                    robot.setMotorPower(power - correction, power + correction, power - correction, power + correction);
                }
                else{
                    correction += 0.1;
                    robot.setMotorPower(power + correction, power - correction, power + correction, power - correction);
                }
            }
        }
        robot.setAllPower(0);
    }

    public void strafeRight(double power, int duration){
        strafe(power, duration);
    }

    public void strafeLeft(double power, int duration){
        strafe(-power, duration);
    }

    public void turnPID(double degrees){
        turnToPID(degrees + getAbsoluteAngle());
    }
}
