/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
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
import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="HoloTeleOP", group="Linear Opmode")
//@Autonomous(name="Gyro holo drive", group="Linear opmodes")

//@Disabled
public class Holodrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    BNO055IMU imu    = null;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // * CONTROL_HUB_ORIENTATION_FACTOR MUST be set correctly depending on whether control hub is on the top or bottom of the robot!!! * //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private int  CONTROL_HUB_ORIENTATION_FACTOR = -1; // -1 for top, +1 for bottom

    /**
     * The following methods are hardware specific and should be changed for each robot
     * to match your robot's specific hardware names for motors, gyro etc...
     */

    /**
     * Initialize the motors
     */
    private void initializeMotors(){
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Motors on one side need to effectively run 'backwards' to move 'forward'
        // Reverse the motors that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
    }

    private void resetEncoders(){
        //Stop the motors and reset the encoders to zero
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Make sure we re-enable the use of encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int[] getMotorEncoders(){
        int[] result = new int[4];

        result[0] = frontLeftDrive.getCurrentPosition();
        result[1] = frontRightDrive.getCurrentPosition();
        result[2] = backLeftDrive.getCurrentPosition();
        result[3] = backRightDrive.getCurrentPosition();

        return result;
    }

    /**
     * Initialize the gyro
     */
    private void initializeGyro(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    private void resetGyro(){
        //  imu.resetZAxisIntegrator();
    }

    private double getGyroHeading(){
        //  return gyro.getIntegratedZValue();
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle * CONTROL_HUB_ORIENTATION_FACTOR;
    }

    /**
     * This is the main op mode and should call all the initialization, wait for start,
     * execute your desired auto/tele-op, then stop everything
     */
    @Override
    public void runOpMode() {

        //Must be called at the start to initialize all necessary hardware
        //Add other hardware (e.g. vision etc...) in this method
        initializeHardware();

        // Wait for the game to start (driver presses PLAY)
        //Note, we can use this time to be processing the vision to have the skystone location ready ASAP.
        //waitForStart();
        waitStart();
        runtime.reset();

        //Make sure the gyro is at zero
        resetGyro();

        doTeleop();

        //Done so turn everything off now
        disableHardware();
    }

    private void waitStart(){
        // Wait for the game to start (driver presses PLAY)
        //Can use this time to process the vision in order to get the skystone location ASAP
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
    }

    /**
     * Disable all hardware
     */
    private void disableHardware() {
        //Only the motors really need to be turned off at the moment
        setMotors(0, 0, 0, 0);
    }

    /**
     * Initialize all hardware
     */
    private void initializeHardware(){
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        initializeMotors();
        initializeGyro();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

      public double getCurrentHeading(){
        double current;

        current = getGyroHeading();
        //Convert to +- 180
        while (current > Math.PI)  current -= 2*Math.PI;
        while (current <= -Math.PI) current += 2*Math.PI;
        return current;
    }

    public double calculateHeadingError(double targetHeading, double currentHeading) {
        double robotError;

        // calculate error in -179 to +180 range : In radians
        robotError = targetHeading - currentHeading;
        while (robotError > Math.PI)  robotError -= 2*Math.PI;
        while (robotError <= -Math.PI) robotError += 2*Math.PI;
        return robotError;
    }

    private void setMotors(double FL, double FR, double BL, double BR){
        double max;
        //If any value is greater than 1.0 normalize all values accordingly
        max = Math.max(Math.max(FL,FR),Math.max(BL,BR));
        if (max > 1.0){
            FL = FL / max;
            FR = FR / max;
            BL = BL / max;
            BR = BR / max;
        }

        frontLeftDrive.setPower(FL);
        frontRightDrive.setPower(FR);
        backLeftDrive.setPower(BL);
        backRightDrive.setPower(BR);

    }

    /**
     * Calculate normalization factor angle (i.e. where in semi-quadrant for unit square to unit circle transposition)
     * @param angle angle to calculate normalization factor for in degrees
     */
    private double unitNormalizationAngle(double angle){
        double normalizationAngle;
        double angleDegrees;

        angleDegrees = Math.toDegrees(angle);
        if (angleDegrees >= 0)
            normalizationAngle = angleDegrees % 90;
        else
            normalizationAngle = (-angleDegrees) % 90;

        if (normalizationAngle >= 45) {
            normalizationAngle = 90 - normalizationAngle;
        }

        //
        return normalizationAngle;
    }

    /**
     * Tele-op example
     * Right stick will move the robot in field centric world view
     * Left stick will point the robot in the field centric direction
     * Left/right bumbers will rotate the robot
     */
    private void doTeleop(){
        double rightJoyX;
        double leftJoyX;
        double leftJoyY;
        boolean leftBumper;
        boolean rightBumper;
        double translateDirection;
        double botCentricDirection;
        double translateSpeed;
        double targetHeading;
        double headingError;
        double currentHeading;
        double translateDeadband = 0.1;
        double rotationSpeedFactor = 1;//Sets maximum rotation speed. Gets multiplied by error in radians
        double manualRotationSpeed = .02;//Heading change per loop period in radians. Will need to tune
        double minimumHeadingCorrectionSpeed = 0.1;//Minimum rotation correction speed
        double headingCorrectionDeadband = 0.05;//Deadband in radians
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;
        double normalizationAngle;
        double finalSpeed;
        double normalizationFactor;
        double headingCorrectionSpeed;

        //Take our initial zero heading from our starting pose
        targetHeading = getCurrentHeading();

        while(opModeIsActive()){
            //Get the joysticks
            leftJoyX = gamepad1.left_stick_x;
            leftJoyY = -gamepad1.left_stick_y;
            rightJoyX = gamepad1.right_stick_x;
            leftBumper = gamepad1.left_bumper;
            rightBumper = gamepad1.right_bumper;

            //Calculate desired translation 'speed' and  direction
            translateSpeed = Math.sqrt((leftJoyX * leftJoyX) + (leftJoyY * leftJoyY));

            if (translateSpeed > translateDeadband){//Create deadband and also ensure no divide by zero in atan2 and stop robot twitching
                //Calculate the desired robot base direction from the right joystick
                //Forward = 0 radians (0 degrees)
                translateDirection = (-Math.atan2(leftJoyY, leftJoyX) + (Math.PI/2));
            }
            else {
                translateDirection = 0;
                translateSpeed = 0;
            }

            //Now check if any rotation requested from the bumpers
            if (leftBumper) {
                targetHeading = targetHeading - manualRotationSpeed;
            }
            else if (rightBumper) {
                targetHeading = targetHeading + manualRotationSpeed;
            }
            //Otherwise check if any rotation requested from the bumpers
            else if (Math.abs(rightJoyX) > 0.1) {
                targetHeading = targetHeading + (rightJoyX * manualRotationSpeed);
            }

            //Calculate if any rotation is needed to point bot in 'targetHeading' direction
            currentHeading = getCurrentHeading();
            headingError = calculateHeadingError(targetHeading, currentHeading);
            //If the error is small (less than the deadband) then do not correct anything
            if (Math.abs(headingError) > headingCorrectionDeadband)
                headingCorrectionSpeed = headingError * rotationSpeedFactor;
            else
                headingCorrectionSpeed = 0.0;

            //If the correction power is really small then increase it so it actually does something, unless it really was zero
            if ((Math.abs(headingCorrectionSpeed) < minimumHeadingCorrectionSpeed)  && (headingCorrectionSpeed != 0.0))
            {
                if (headingCorrectionSpeed >= 0)
                    headingCorrectionSpeed = minimumHeadingCorrectionSpeed;
                else
                    headingCorrectionSpeed = -minimumHeadingCorrectionSpeed;
            }

            //Subtract the current heading to get robot centric direction
            botCentricDirection =  translateDirection - currentHeading;

/*          This version will normalize the joystick position to fully utilize the entire power range for 0, 90, 180 & 270 by mapping from unit square to unit circle
            //Adjust to use full range of speed to 'boost' the power for 0, 90, 180, 270 from .707 to 1.0
            normalizationAngle = unitNormalizationAngle(targetHeading);
            //The following can be optimized to eliminate some sqrt calls
            normalizationFactor = Math.sqrt(1+Math.tan(normalizationAngle))/Math.sqrt(2);
            finalSpeed = translateSpeed / (normalizationFactor * Math.sqrt(2));
*/
            //This version is a simplified version that does not maximize the entire power range of the motors
            finalSpeed = translateSpeed;
            normalizationFactor = 1.0;

            //Now calculate the actual motor speeds
            //Note, the diagonally opposite speeds are the same, so only need to calculate 2 values
            frontLeftSpeed = finalSpeed*Math.sin(botCentricDirection+(Math.PI/4));
            //Now scale to utilize full power
            frontLeftSpeed = (frontLeftSpeed/normalizationFactor);

            frontRightSpeed = finalSpeed*Math.cos(botCentricDirection+(Math.PI/4));
            //Now scale to utilize full power
            frontRightSpeed = (frontRightSpeed/normalizationFactor);

            //Duplicate diagonal speeds for translation and add in rotation
            backLeftSpeed   = frontRightSpeed + headingCorrectionSpeed;
            backRightSpeed  = frontLeftSpeed - headingCorrectionSpeed;
            frontRightSpeed = frontRightSpeed - headingCorrectionSpeed;
            frontLeftSpeed  = frontLeftSpeed + headingCorrectionSpeed;

            telemetry.addData("leftJoyX: ", leftJoyX);
            telemetry.addData("leftJoyY: ", leftJoyY);
            telemetry.addData("HE: ", Math.toDegrees((headingError)));
            //telemetry.addData("CH: ", Math.toDegrees(currentHeading));
            //telemetry.addData("TD: ", translateDirection);
            //telemetry.addData("BCD: ", botCentricDirection);
            telemetry.addData("FL: ", frontLeftSpeed);
            telemetry.addData("FR: ", frontRightSpeed);
            telemetry.addData("BL: ", backLeftSpeed);
            telemetry.addData("BR: ", backRightSpeed);
            //telemetry.addData("FLE: ", frontLeftDrive.getCurrentPosition());
            //telemetry.addData("FRE: ", frontRightDrive.getCurrentPosition());
            //telemetry.addData("BLE: ", backLeftDrive.getCurrentPosition());
            //telemetry.addData("BRE: ", backRightDrive.getCurrentPosition());
            //And actually set the motors accordingly
            //Note, this function will also clamp and scale the power to 1.0
            setMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
            telemetry.update();
        }
        setMotors(0, 0, 0, 0);
    }
}

