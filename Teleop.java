package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop", group = "StarterBot")

public class Teleop extends OpMode {
    final double LAUNCHER_TARGET_VELOCITY = 1000;
    final double FULL_SPEED_SERVO = 1.0;
    final double FEED_TIME_SECONDS = 0.8;
    ElapsedTime feederTimer = new ElapsedTime();

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intakeMotor = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private enum LaunchState {
        IDLE,
        REQUESTED,
        READY,
    }
    LaunchState launchState;

    public void init() {
        
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        //launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        //launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);

        //launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.rightBumperWasPressed() && launchState == launchState.IDLE) {
            launchState = launchState.REQUESTED;
        }
        if (gamepad1.x) {
            intakeMotor.setPower(1.0);
        }
        else {
            intakeMotor.setPower(0.0);
        }

        //launch_logic(); 

    }

    /*public void launch_logic() {
        if (launchState == launchState.REQUESTED) {
            launcher.setVelocity(-1010);
            if (launcher.getVelocity() < -1000) {
                launchState = LaunchState.READY;
                leftFeeder.setPower(FULL_SPEED_SERVO);
                rightFeeder.setPower(FULL_SPEED_SERVO);
                feederTimer.reset();
                }
            }
            
        else if (launchState == launchState.READY) {
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                launchState = LaunchState.IDLE;
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
        }
    } 
    
        
    }
    */
    double leftFrontPower;        
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    void mecanumDrive(double forward, double strafe, double rotate) {

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;
        
        telemetry.addData("Speed", leftFrontPower);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }
}
