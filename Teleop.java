package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
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
    private boolean outTakeOn = false;
    private boolean yJustPressed = false;
    private AprilTagProcessor tag;
    private VisionPortal visionPortal;

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx rightLauncher = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx intakeMotor = null;
    private DcMotorEx elevator = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    
    private CRServo tempFeeder = null;


    private enum LaunchState {
        IDLE,
        REQUESTED,
        READY,
    }
    LaunchState launchState;

    public void init() {
        
        launchState = launchState.IDLE;
        
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        elevator = hardwareMap.get(DcMotorEx.class, "elevator_motor");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);

        //launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        //launcher.setZeroPowerBehavior(BRAKE);



        //launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        //rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //initAprilTag();

    }

    public void loop() {
        
        mecanumDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.x) {
            intakeMotor.setPower(1.0);
        }
        else {
            intakeMotor.setPower(0.0);
        }

        
        if (gamepad1.y && !yJustPressed) {
            yJustPressed = true;
        } else {
            yJustPressed = false;
        }

        if (yJustPressed) {
            if (!elevatorOn) {
                elevatorOn = true;
                elevator.setPower(1.0);
            }
            if (elevatorOn) {
                elevatorOn = false;
                elevator.setPower(0.0);
            }
        }

        if (gamepad1.a) {
            elevator.setPower(1.0);
        }
        else {
            elevator.setPower(0.0);
        }
         

        
        //launch_logic(); 
 
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.update();

    }
    boolean lstate = false;
    public void launch_logic() {
        lstate = (launchState == launchState.IDLE);
        telemetry.addData("data", lstate);
  
        telemetry.update();
        if (launchState == launchState.IDLE && gamepad1.y) {
            launchState = launchState.REQUESTED;
            rightLauncher.setVelocity(1010);
            leftLauncher.setVelocity(1010);
        }
        if (launchState == launchState.REQUESTED) {
            if (rightLauncher.getVelocity() > 1000 && 
            leftLauncher.getVelocity() > 1000) {
                launchState = LaunchState.READY;
                leftFeeder.setPower(-1);
                rightFeeder.setPower(1);
                }
            }
            
        if (launchState == launchState.READY) {
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                launchState = launchState.IDLE;
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                feederTimer.reset();

        } 

    } 
    
        
    }
    
    double leftFrontPower;        
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    
    void mecanumDrive(double forward, double strafe, double rotate) {

          //the denominator is the largest motor power (absolute value) or 1
          //This ensures all the powers maintain the same ratio,
          //but only if at least one is out of the range [-1, 1]
         
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
    /*void initAprilTag() {

        // Create the AprilTag processor.
        tag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // IDK IF THIS IS NECESSARY BUT IT DOESN'T BUILD OTHERWISE
        //builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(tag);

        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(tag, true);

        }   // end method initAprilTag() */
}
