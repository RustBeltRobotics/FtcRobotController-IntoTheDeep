package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp

public class DriveTrainOpMode extends LinearOpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor lowerArmMotor1;
    private DcMotor lowerArmMotor2;
    private DcMotor upperArmMotor1;
    private DcMotorEx intakeMoter;

    // lower arm 1482
    // upper arm -160
    // lower arm 90 degrees off: 88
    // upper arm 90 degrees off: 562
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
    static final double LOWER_TICKS_PER_DEGREE = 1394 / 90;
    static final double UPPER_TICKS_PER_DEGREE = -722 / 90;

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        lowerArmMotor1 = hardwareMap.get(DcMotor.class, "lam1");
        lowerArmMotor2 = hardwareMap.get(DcMotor.class, "lam2");
        upperArmMotor1 = hardwareMap.get(DcMotor.class, "uam1");
        intakeMoter = (DcMotorEx) hardwareMap.get(DcMotor.class, "iM");
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
            botHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("botHeading", botHeading);
            tgtPowerX = this.gamepad1.left_stick_x;
            tgtPowerY = -this.gamepad1.left_stick_y;
            tgtPowerRX = jeySteckRespencse(-this.gamepad1.right_stick_x);
            lowerArmTgtPower = jeySteckRespencse(this.gamepad2.left_stick_y);
            upperArmTgtPower = jeySteckRespencse(this.gamepad2.right_stick_y);
//            lowerArmDegrees = -124 + (lowerArmMotor1.getCurrentPosition() / COUNTS_PER_DEGREE);
//            upperArmDegrees = -34 + (upperArmMotor1.getCurrentPosition() / COUNTS_PER_DEGREE);
            lowerArmDegrees = (lowerArmMotor1.getCurrentPosition() / LOWER_TICKS_PER_DEGREE) - 124;
            upperArmDegrees = (upperArmMotor1.getCurrentPosition() / UPPER_TICKS_PER_DEGREE) + 75;

// CAREFUL TESTING THIS!! this is to give us good control at slow speeds, while still being able to move fast.
            tgtPowerX = jeySteckRespencse(tgtPowerX);
            tgtPowerY = jeySteckRespencse(tgtPowerY);
//            tgtPowerRX =

            telemetry.addData("lowerArmMotor1 degrees", lowerArmDegrees);
            telemetry.addData("upperArm degrees", upperArmDegrees);

            // lower arm 1482
            // upper arm -160

            // lower arm 90 degrees off: 88
            // upper arm 90 degrees off: 562
            // check to make sure we're not moving the upper arm illegally
//            if (lowerArmDegrees > -90) {
//                if (lowerArmDegrees + upperArmDegrees < 90.0) {
//                    moveArm(lowerArmTgtPower, upperArmTgtPower);
//                } else {
//                    if (upperArmTgtPower < 0 || lowerArmTgtPower < 0) {
//                        telemetry.addData("WARNING UPPER ARM AT LIMIT", upperArmDegrees);
//                    } else {
//                        moveArm(lowerArmTgtPower, upperArmTgtPower);
//                    }
//                }
//            } else {
//                moveArm(lowerArmTgtPower, upperArmTgtPower);
//            }

//            if(isUpperArmLegal(lowerArmDegrees, upperArmDegrees) || this.gamepad2.left_bumper || upperArmTgtPower >= 0) {
//                moveUpperArm(upperArmTgtPower);
//            }
//
//            if(isLowerArmLegal(lowerArmDegrees, upperArmDegrees) || this.gamepad2.left_bumper || lowerArmTgtPower <= 0) {
//                moveLowerArm(lowerArmTgtPower);
//            }


            // moveArm(lowerArmTgtPower, upperArmTgtPower);
            if((isUpperArmLegal(lowerArmDegrees, upperArmDegrees) && isLowerArmLegal(lowerArmDegrees, upperArmDegrees)) || this.gamepad2.left_bumper) {
                moveArm(lowerArmTgtPower, upperArmTgtPower);
            } else {
                    if (upperArmTgtPower <= 0) {
                        telemetry.addData("WARNING UPPER ARM AT LIMIT", upperArmDegrees);
                        upperArmMotor1.setPower(0);
                        gamepad2.setLedColor(255,0, 0, 1000);

                    } else {
                        telemetry.addData(" UPPER ARM AT ", upperArmDegrees);
                        moveUpperArm(upperArmTgtPower);
                    }
                    if (lowerArmTgtPower <= 0) {
                        telemetry.addData("WARNING LOWER ARM AT LIMIT", lowerArmDegrees);
                        lowerArmMotor1.setPower(0);
                        lowerArmMotor2.setPower(0);
                        gamepad2.setLedColor(255,0, 0, 1000);
                    } else {
                        telemetry.addData(" LOWER ARM AT  ", lowerArmDegrees);
                        moveLowerArm(lowerArmTgtPower);
                    }
            }
            intake();
            // enable field-centric adjustments when left bumper pushed
            if (this.gamepad1.left_bumper == true) {
                tgtPowerX = tgtPowerX * Math.cos(-botHeading) - tgtPowerY * Math.sin(-botHeading);
                tgtPowerY = tgtPowerY * Math.sin(-botHeading) + tgtPowerY * Math.cos(-botHeading);
                tgtPowerX = tgtPowerX * 1.1;
                telemetry.addData("Field centric", "ON");
                telemetry.addData("botheading", botHeading);
            }
            if (this.gamepad2.right_bumper) {
                reset();
                telemetry.addData("Encoders reset. Lower arm degrees:", lowerArmDegrees);
                telemetry.addData("Encoders reset. Upper arm degrees:", upperArmDegrees);
                telemetry.update();
            }

            denom = Math.max(Math.abs(tgtPowerY) + Math.abs(tgtPowerX) + Math.abs(tgtPowerRX), 1);

            motor1.setPower(-(tgtPowerY + tgtPowerX + tgtPowerRX) / -denom );
//            telemetry.addData("Target Power",(tgtPowerY + tgtPowerX + tgtPowerRX) / -denom );
//            telemetry.addData("Motor1 Power", motor1.getPower());
////            telemetry.addData("Motor1 RunMode", motor1.getMode());
//            telemetry.addData("motor 1 position", motor1.getCurrentPosition());
//
            motor2.setPower(-(tgtPowerY + -tgtPowerX + -tgtPowerRX) / denom );
//            telemetry.addData("Target Power", (tgtPowerY + -tgtPowerX + -tgtPowerRX) / denom);
//            telemetry.addData("Motor2 Power", motor2.getPower());
////            telemetry.addData("Motor2 RunMode", motor2.getMode());
//            telemetry.addData("motor 2 position", motor2.getCurrentPosition());
//
            motor3.setPower(-(tgtPowerY + -tgtPowerX + tgtPowerRX) / -denom );
//            telemetry.addData("Target Power", (tgtPowerY + -tgtPowerX + tgtPowerRX) / -denom);
//            telemetry.addData("Motor3 Power", motor3.getPower());
////            telemetry.addData("Motor3 RunMode", motor3.getMode());
//            telemetry.addData("motor 3 position", motor3.getCurrentPosition());
//
            motor4.setPower(-(tgtPowerY + tgtPowerX + -tgtPowerRX) / denom );
//            telemetry.addData("Target Power", (tgtPowerY + tgtPowerX + -tgtPowerRX) / denom);
//            telemetry.addData("Motor4 Power", motor4.getPower());
//            telemetry.addData("motor 4 position", motor4.getCurrentPosition());
////            telemetry.addData("Motor4 RunMode", motor4.getMode());

            telemetry.addData("Status", "Running v6");
            telemetry.update();
        }
        // shutdown code

    }
    double jeySteckRespencse(double power) {
        double a = 3.7;
        double b = 0.43;

        return (Math.signum(power)*Math.pow(Math.abs(power), a) + (power * b)) / (1 + b);
    }
    private void reset() {
        lowerArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveArm(double lowerArmTgtPower, double upperArmTgtPower) {
        lowerArmMotor1.setPower(-lowerArmTgtPower);
        lowerArmMotor2.setPower((lowerArmTgtPower));
        telemetry.addData("lower arm tgt Power", lowerArmTgtPower);
        telemetry.addData("lower arm Power", lowerArmMotor1.getPower());
        upperArmMotor1.setPower(upperArmTgtPower);
        telemetry.addData("upper tgt arm Power", upperArmTgtPower);
        telemetry.addData("upper arm Power", upperArmMotor1.getPower());
    }

    private void moveUpperArm(double upperArmTgtPower) {
        upperArmMotor1.setPower(upperArmTgtPower);
        telemetry.addData("upper tgt arm Power", upperArmTgtPower);
        telemetry.addData("upper arm Power", upperArmMotor1.getPower());
    }

    private void moveLowerArm(double lowerArmTgtPower) {
        lowerArmMotor1.setPower(-lowerArmTgtPower);
        lowerArmMotor2.setPower((lowerArmTgtPower));
        telemetry.addData("lower arm tgt Power", lowerArmTgtPower);
        telemetry.addData("lower arm Power", lowerArmMotor1.getPower());
    }

    private void intake(){
        if(this.gamepad2.left_trigger == 0){
            intakeMoter.setPower(-this.gamepad2.right_trigger);
        } else {
            intakeMoter.setPower(this.gamepad2.left_trigger);

        }
        telemetry.addData("intake current: ", intakeMoter.getCurrent(CurrentUnit.MILLIAMPS));
    }

    private boolean isUpperArmLegal(double lowerArmDegrees, double upperArmDegrees) {
        double lowerArmAngle = lowerArmDegrees * (Math.PI / 180.0);
        double upperArmAngle = upperArmDegrees * (Math.PI / 180.0);
        double pivot_lowerArm_x = 0.0;
        double pivot_lowerArm_y = 0.0;

        double lowerArm_length = 15.5; // inches
        double upperArm_length = 18.0;
        double angleOffset = Math.PI/2;

        double pivot_upperArm_x = pivot_lowerArm_x + lowerArm_length * Math.cos(-lowerArmAngle + angleOffset);
        double pivot_upperArm_y = pivot_lowerArm_y + lowerArm_length * Math.sin(-lowerArmAngle + angleOffset);

        double gripper_x = pivot_upperArm_x + upperArm_length * Math.cos(-upperArmAngle + angleOffset);
        double gripper_y = pivot_upperArm_y + upperArm_length * Math.sin(-upperArmAngle + angleOffset);

        telemetry.addData("gripper_x", gripper_x);
        telemetry.addData("pivot_upperArm_x", pivot_upperArm_x);

        if (gripper_x >= 8) {
            telemetry.addData("WARNING UPPER ARM AT LIMIT", upperArmDegrees);
            return false;
        }

        return true;
    }

    private boolean isLowerArmLegal(double lowerArmDegrees, double upperArmDegrees) {
        double lowerArmAngle = lowerArmDegrees * (Math.PI / 180.0);
        double upperArmAngle = upperArmDegrees * (Math.PI / 180.0);
        double pivot_lowerArm_x = 0.0;
        double pivot_lowerArm_y = 0.0;

        double lowerArm_length = 15.5; // inches
        double upperArm_length = 18.0;
        double angleOffset = Math.PI/2;

        double pivot_upperArm_x = pivot_lowerArm_x + lowerArm_length * Math.cos(-lowerArmAngle + angleOffset);
        double pivot_upperArm_y = pivot_lowerArm_y + lowerArm_length * Math.sin(-lowerArmAngle + angleOffset);

        double gripper_x = pivot_upperArm_x + upperArm_length * Math.cos(-upperArmAngle + angleOffset);
        double gripper_y = pivot_upperArm_y + upperArm_length * Math.sin(-upperArmAngle + angleOffset);

        telemetry.addData("gripper_x", gripper_x);
        telemetry.addData("pivot_upperArm_x", pivot_upperArm_x);

        if (pivot_upperArm_x >= 7) {
            telemetry.addData("WARNING LOWER ARM AT LIMIT", lowerArmDegrees);
            return false;
        }

        return true;
    }

}
