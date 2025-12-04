package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import org.firstinspires.ftc.teamcode.aprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

@Autonomous

public class Auto extends LinearOpMode {

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

    private AprilTagProcessor tag;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detections = tag.getDetections();

    private ElapsedTime feedTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
                
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

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // sets the coefficients to run
        //setPositionCoefficient();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        forward(1000);
        right(1000);
        shoot(3);

    }
    
    public void runWheels(int leftFront, int rightFront, int leftBack, int rightBack) {

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(leftFront);
        rightFrontDrive.setTargetPosition(rightFront);
        leftBackDrive.setTargetPosition(leftBack);
        rightBackDrive.setTargetPosition(rightBack);
        
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        leftFrontDrive.setPower(0.8);
        rightFrontDrive.setPower(0.8);
        leftBackDrive.setPower(0.8);
        rightBackDrive.setPower(0.8);

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || 
        leftBackDrive.isBusy() || rightBackDrive.isBusy()) {
            telemetry.addData("Left F: ", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Right F: ", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Left B: ", leftBackDrive.getCurrentPosition());
            telemetry.addData("Rigth B: ", rightBackDrive.getCurrentPosition());
            telemetry.update();
        } // waits until they are done

        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        //sleep(100); 
    }
    
    public void forward(double mm) {
        int ticks = (int)((mm * 537.6) / (96 * Math.PI));
        runWheels(ticks, ticks, ticks, ticks);
    }

    public void right(double mm) {
        int ticks = (int)((mm * 537.6) / (96 * Math.PI));
        runWheels(ticks, -ticks, -ticks, ticks);
    }

    public void rotate(double mm) {
        int ticks = (int)((mm * 537.6) / (96 * Math.PI));
        runWheels(ticks, -ticks, ticks, -ticks);
    }

    public void shoot(int shots) {
        rightLauncher.setVelocity(1010);
        leftLauncher.setVelocity(1010);

        while (rightLauncher.getVelocity() < 1000
            && leftLauncher.getVelocity() < 1000) {}

        intakeMotor.setPower(1.0);
        elevator.setPower(1.0);
        rightFeeder.setPower(1.0);
        leftFeeder.setPower(-1.0);

        feedTimer.reset();
        while (feedTimer.seconds() < 0.1 + (0.2 * shots)) {}

        rightLauncher.setVelocity(0);
        leftLauncher.setVelocity(0);
        intakeMotor.setPower(0.0);
        elevator.setPower(0.0);
        rightFeeder.setPower(0.0);
        leftFeeder.setPower(0.0);
    
    }
    public void aprilTagInit() {
        // Create the AprilTag processor.
        tag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // IDK IF THIS IS NECESSARY BUT IT DOESN'T BUILD WITH THIS
        //builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(tag);

        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(tag, true);
    }
    public void localize() {
        detections = tag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) {
                telemetry.addLine("No Metadata");
            } else {
                telemetry.addLine(String.format("ID: ", detection.id, "name: ", detection.metadata.name));

                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch?)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg?)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            }

        }
    }
}













