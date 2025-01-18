package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Standard Autonmous", group="Robot")

public class StandardAutoMode extends LinearOpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor lowerArmMotor1;
    private DcMotor lowerArmMotor2;
    private DcMotor upperArmMotor1;
    private DcMotor intakeMoter;
    IMU gyro;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.94 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;  // ???
    static final double     TURN_SPEED              = 0.5;  // ???
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
    static final double LOWER_TICKS_PER_DEGREE = 1394 / 90;
    static final double UPPER_TICKS_PER_DEGREE = -722 / 90;

    @Override
    public void runOpMode() {
        lowerArmMotor1 = hardwareMap.get(DcMotor.class, "lam1");
        lowerArmMotor2 = hardwareMap.get(DcMotor.class, "lam2");
        upperArmMotor1 = hardwareMap.get(DcMotor.class, "uam1");
        intakeMoter = hardwareMap.get(DcMotor.class, "iM");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gyro = hardwareMap.get(IMU.class, "gyro");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        gyro.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        resetArm();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveForward(12);
        while(motorRunning()){}
        turnRightContinuous(-90);
        turnRight(-90);
        while(motorRunning()){}
        driveForward(24);
        scoreInLowerBasket();
        telemetry.update();

        while(motorRunning()){}
        while(armRunning()){}
    }

    public void driveForward(double inches) {
        while(motorRunning()){}
        int ticks = (int) Math.round(inches * COUNTS_PER_INCH);
        telemetry.addData("Driving strate", ticks);
        telemetry.update();
        reset();
        motor1.setTargetPosition(ticks);
        motor2.setTargetPosition(-ticks);
        motor3.setTargetPosition(ticks);
        motor4.setTargetPosition(-ticks);
        moveToPosition();
        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
        }
    public void turnRight(int degrees) {
       /* while(motorRunning()){}
         //int ticks = (int) Math.round(degrees * (7.113544266 * 2) );
        int ticks = (int) Math.round(degrees * (117) );
        telemetry.addData("turning", degrees);
        telemetry.update();
        reset();
        motor1.setTargetPosition(ticks);
        motor2.setTargetPosition(ticks);
        motor3.setTargetPosition(ticks);
        motor4.setTargetPosition(ticks);
        moveToPosition();
        motor1.setPower(.5);
        motor2.setPower(.5);
        motor3.setPower(.5);
        motor4.setPower(.5);
    */
        while(motorRunning()){}
        gyro.resetYaw();
        degrees = degrees-2;
        while (Math.abs(getYaw()) < degrees  ) {
            reset();
            int ticks = 140;
            telemetry.addData("turning -> D: ", getYaw());
            telemetry.update();
            motor1.setTargetPosition(ticks);
            motor2.setTargetPosition(ticks);
            motor3.setTargetPosition(ticks);
            motor4.setTargetPosition(ticks);
            moveToPosition();
            motor1.setPower(.5);
            motor2.setPower(.5);
            motor3.setPower(.5);
            motor4.setPower(.5);
            while(motorRunning()){}
        }
    }

    public void turnRightContinuous(int degrees) {
        while(motorRunning()){}
        gyro.resetYaw();
        reset();
        int ticks = 100000;
        telemetry.addData("turning -> D: ", getYaw());
        telemetry.update();
        motor1.setTargetPosition(ticks);
        motor2.setTargetPosition(ticks);
        motor3.setTargetPosition(ticks);
            motor4.setTargetPosition(ticks);
            moveToPosition();
            motor1.setPower(.05);
            motor2.setPower(.05);
            motor3.setPower(.05);
            motor4.setPower(.05);
        while(motorRunning()){
            if(Math.abs(getYaw()) >= degrees  ) {
                telemetry.addData("turning COMPLETE :D", getYaw());
                telemetry.update();
                reset();
                motor1.setTargetPosition(0);
                motor2.setTargetPosition(0);
                motor3.setTargetPosition(0);
                motor4.setTargetPosition(0);
                moveToPosition();
                motor1.setPower(.1);
                motor2.setPower(.1);
                motor3.setPower(.1);
                motor4.setPower(.1);
                reset();
                break;
            }
        }
    }

    // Extend arm, pick up sample, return arm to neutral position
    private void pickupSample() {
        setLowerArm(-90);
        setUpperArm(-5);
        intake(-1);
    }

    // Raise arm, release sample, return arm to neutral position
    private void scoreInUpperBasket() {
        setLowerArm(-3);
        setUpperArm(93);
        intake(-1);
    }

    // Raise arm, release sample, return arm to neutral position
    private void scoreInLowerBasket() {
        setLowerArm(-78);
        setUpperArm(62);
        while(armRunning()){}
        intake(1);
        try {
            TimeUnit.SECONDS.sleep(2);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        intake(0);
        stowArm();
    }

    // Put arm away at the end of auto
    private void stowArm() {
        setLowerArm(-124);
        setUpperArm(75);
    }

    // Put arm in neutral position between moves
    private void holdArm() {
        setLowerArm(-90);
        setUpperArm(145);
    }
    // set lower arm to degree position provided
    private void setLowerArm(int degrees){

        degrees = degrees + 124;
        // int lowerArmDegrees = -124 + (int) Math.round(lowerArmMotor1.getCurrentPosition() / LOWER_TICKS_PER_DEGREE);

        telemetry.addData("setting lower arm degrees to ", degrees);
        telemetry.addData("setting lower arm ticks to ", (int)(degrees * LOWER_TICKS_PER_DEGREE));
        telemetry.addData("lowerArm at ", lowerArmMotor2.getCurrentPosition());

        lowerArmMotor1.setTargetPosition((int)(degrees * LOWER_TICKS_PER_DEGREE));
        lowerArmMotor2.setTargetPosition(-(int)(degrees * LOWER_TICKS_PER_DEGREE));
        lowerArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowerArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowerArmMotor1.setPower(.5);
        lowerArmMotor2.setPower(.5);

        telemetry.addData("lowerArm new position ", lowerArmMotor2.getCurrentPosition());
    }
    // set upper arm to degree position provided
    private void setUpperArm(double degrees){
        degrees = degrees - 75;
        //int upperArmDegrees = 75 + (int) Math.round(upperArmMotor1.getCurrentPosition() / UPPER_TICKS_PER_DEGREE) + degrees;
        telemetry.addData("setting upper arm degrees to ", degrees);
        telemetry.addData("setting upper arm ticks to ", (int)(degrees * UPPER_TICKS_PER_DEGREE));
        telemetry.addData("upperArm at ", upperArmMotor1.getCurrentPosition());
        upperArmMotor1.setTargetPosition((int)(degrees * UPPER_TICKS_PER_DEGREE));
        upperArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperArmMotor1.setPower(.5);
        telemetry.addData("upperArm new position ", upperArmMotor1.getCurrentPosition());
    }
    private void intake(int Speed){
     intakeMoter.setPower(Speed);
    }
    private void reset() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void moveToPosition() {
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean motorRunning() {
        if(motor1.isBusy()) {
            return true;
        }
        if(motor2.isBusy()) {
            return true;
        }
        if(motor3.isBusy()) {
            return true;
        }
        if(motor4.isBusy()) {
            return true;
        }
        return false;
    }


    private boolean armRunning() {
        if(lowerArmMotor1.isBusy()) {
            return true;
        }
        if(lowerArmMotor2.isBusy()) {
            return true;
        }
        if(upperArmMotor1.isBusy()) {
            return true;
        }
        return false;
    }

    private double getYaw() {
        return gyro.getRobotYawPitchRollAngles().getYaw();
    }

    private void resetArm() {
        lowerArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
