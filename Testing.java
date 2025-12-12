/* Copyright (c) 2017 FIRST. All rights reserved.
        // show motor velocities
        telemetry.addLine(String.format("LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f",
            leftFrontDrive.getVelocity(),
            rightFrontDrive.getVelocity(),
            leftBackDrive.getVelocity(),
            rightBackDrive.getVelocity()
        ));
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Testing", group="Linear OpMode")

public class testing extends OpMode {
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
    
    private int deviceIndex = 0;
    private boolean rightBumperPrev = false;
    private boolean leftBumperPrev = false;
    private List<DcMotorEx> devices = null;
    

    
    public void init() {
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

        devices = new ArrayList<>();
        devices.add(leftFrontDrive);
        devices.add(rightFrontDrive);
        devices.add(leftBackDrive);
        devices.add(rightBackDrive);
        

    }
    
    @Override
    public void loop() {
        if (gamepad1.right_bumper && !rightBumperPrev) {
            deviceIndex = (deviceIndex + 1) % devices.size();
        }
        if (gamepad1.left_bumper && !leftBumperPrev) {
            deviceIndex = (deviceIndex - 1 + devices.size()) % devices.size();
        }

        telemetry.addLine("Current Device");
        switch (deviceIndex) {
            case 0:
                telemetry.addLine("Left Front Drive");
                break;
            case 1:
                telemetry.addLine("Right Front Drive");
                break;
            case 2:
                telemetry.addLine("Left Back Drive");
                break;
            case 3:
                telemetry.addLine("Right Back Drive");
                break;
        }
        devices.get(deviceIndex).setPower(-gamepad1.left_stick_y);

        telemetry.addLine("Velocities:");
        telemetry.addLine(String.format("LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f",
            leftFrontDrive.getVelocity(),
            rightFrontDrive.getVelocity(),
            leftBackDrive.getVelocity(),
            rightBackDrive.getVelocity()
        ));
        telemetry.update();

        rightBumperPrev = gamepad1.right_bumper;
        leftBumperPrev = gamepad1.left_bumper;
        
    }
}

