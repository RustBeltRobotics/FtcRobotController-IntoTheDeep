package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class DriveTrainOpMode extends LinearOpMode {
    // private Gyroscope imu;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor lowerArmMotor1;
    private DcMotor lowerArmMotor2;
    private DcMotor upperArmMotor1;
    private DcMotor intakeMoter;
    // private DigitalChannel digitalTouch;
    // private DistanceSensor sensorColorRange;
    // private Servo servoTest;

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    @Override
    public void runOpMode() {
//        gyro = hardwareMap.get(Gyroscope.class, "gyro");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        lowerArmMotor1 = hardwareMap.get(DcMotor.class, "lam1");
        lowerArmMotor2 = hardwareMap.get(DcMotor.class, "lam2");
        upperArmMotor1 = hardwareMap.get(DcMotor.class, "uam1");
        intakeMoter = hardwareMap.get(DcMotor.class, "iM");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IMU gyro = hardwareMap.get(IMU.class, "gyro");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        gyro.initialize(parameters);
        reset();
//
//
//
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        double denom = 0;
        double tgtPowerY = 0;
        double tgtPowerX = 0;
        double tgtPowerRX = 0;
        double lowerArmTgtPower = 0;
        double upperArmTgtPower = 0;
        double lowerArmDegrees;
        double upperArmDegrees;
        double botHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (opModeIsActive()) {
            tgtPowerX = this.gamepad1.left_stick_x;
            tgtPowerY = -this.gamepad1.left_stick_y;
            tgtPowerRX = this.gamepad1.right_stick_x;
            lowerArmTgtPower = this.gamepad2.left_stick_y;
            upperArmTgtPower = this.gamepad2.right_stick_y;
            lowerArmDegrees = -124 + (lowerArmMotor1.getCurrentPosition() / COUNTS_PER_DEGREE);
            upperArmDegrees = -34 + (upperArmMotor1.getCurrentPosition() / COUNTS_PER_DEGREE);

            // check to make sure we're not moving the upper arm illegally
            if (lowerArmDegrees > -90) {
                if (lowerArmDegrees + upperArmDegrees < 90.0) {
                    moveArm(lowerArmTgtPower, upperArmTgtPower);
                } else {
                    if (upperArmTgtPower < 0 || lowerArmTgtPower < 0) {
                        telemetry.addData("WARNING UPPER ARM AT LIMIT", upperArmDegrees);
                    } else {
                        moveArm(lowerArmTgtPower, upperArmTgtPower);
                    }
                }
            } else {
                moveArm(lowerArmTgtPower, upperArmTgtPower);
            }

            intake();
            // enable field-centric adjustments when left bumper pushed
            if (this.gamepad1.left_bumper == true) {
                tgtPowerX = tgtPowerX * Math.cos(-botHeading) - tgtPowerY * Math.sin(-botHeading);
                tgtPowerY = tgtPowerY * Math.sin(-botHeading) + tgtPowerY * Math.cos(-botHeading);
                tgtPowerX = tgtPowerX * 1.1;
                telemetry.addData("Field centric", "ON");
            }
            denom = Math.max(Math.abs(tgtPowerY) + Math.abs(tgtPowerX) + Math.abs(tgtPowerRX), 1);

            motor1.setPower((tgtPowerY + tgtPowerX + tgtPowerRX) / -denom );
            telemetry.addData("Target Power",(tgtPowerY + tgtPowerX + tgtPowerRX) / -denom );
            telemetry.addData("Motor1 Power", motor1.getPower());
            telemetry.addData("Motor1 RunMode", motor1.getMode());

            motor2.setPower((tgtPowerY + -tgtPowerX + -tgtPowerRX) / denom );
            telemetry.addData("Target Power", (tgtPowerY + -tgtPowerX + -tgtPowerRX) / denom);
            telemetry.addData("Motor2 Power", motor2.getPower());
            telemetry.addData("Motor2 RunMode", motor2.getMode());

            motor3.setPower((tgtPowerY + -tgtPowerX + tgtPowerRX) / -denom );
            telemetry.addData("Target Power", (tgtPowerY + -tgtPowerX + tgtPowerRX) / -denom);
            telemetry.addData("Motor3 Power", motor3.getPower());
            telemetry.addData("Motor3 RunMode", motor3.getMode());

            motor4.setPower((tgtPowerY + tgtPowerX + -tgtPowerRX) / denom );
            telemetry.addData("Target Power", (tgtPowerY + tgtPowerX + -tgtPowerRX) / denom);
            telemetry.addData("Motor4 Power", motor4.getPower());
            telemetry.addData("Motor4 RunMode", motor4.getMode());


            telemetry.addData("Status", "Running v5");
            telemetry.update();
        }
        // shutdown code

    }

    private void reset() {
        lowerArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArm(double lowerArmTgtPower, double upperArmTgtPower) {
        lowerArmMotor1.setPower(lowerArmTgtPower);
        lowerArmMotor2.setPower(lowerArmTgtPower);
        telemetry.addData("lower arm tgt Power", lowerArmTgtPower);
        telemetry.addData("lower arm Power", lowerArmMotor1.getPower());
        upperArmMotor1.setPower(upperArmTgtPower);
        telemetry.addData("upper tgt arm Power", upperArmTgtPower);
        telemetry.addData("upper arm Power", upperArmMotor1.getPower());
    }
    private void intake(){
        if(this.gamepad2.right_trigger == 0){
            intakeMoter.setPower(-this.gamepad2.left_trigger);
        } else {
            intakeMoter.setPower(this.gamepad2.right_trigger);
        }





    }
}
