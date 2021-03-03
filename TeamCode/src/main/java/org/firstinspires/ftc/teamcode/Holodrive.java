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

    static final double     COUNTS_PER_INCH         =  55;//Need to update this with correct value

    double skystoneLocation;

    /**
     * The following methods are hardware specific and should be changed for each robot
     * to match your robot's specific hardware names for motors, gyro etc...
     */

    private void initializeVuforia(){

    }

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
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

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
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * The remaining methods are generic and can be copied as-is
     */

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

        //Turn off Vuforia now since we no longer need it?

        //Make sure the gyro is at zero
        resetGyro();

        //Auto example
        //doAuto_1();

        //or do teleop
        doTeleop();

        //Done so turn everything off now
        disableHardware();
    }

    private void waitStart(){
        // Wait for the game to start (driver presses PLAY)
        //Use this time to process the vision in order to get the skystone location ASAP
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            // Display the light level while we are waiting to start
            skystoneLocation = getSkystoneLocation();
            telemetry.addData("Stone location ", skystoneLocation);
            telemetry.update();
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
        initializeVuforia();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    private double getSkystoneLocation(){
        //Process Vuforia to determine the stone position with respect to our location
        //0 is directly ahead. Result is inches offset from directly ahead.
        return 0;
    }

    public double getCurrentHeading(){
        double current;

        current = getGyroHeading();
        //Convert to +- 180
        while (current > 180)  current -= 360;
        while (current <= -180) current += 360;
        return current;
    }

    public double calculateHeadingError(double targetHeading, double currentHeading) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetHeading - currentHeading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * Turn the robot to point in the selected heading (within 2 degrees)
     * Currently does NOT slow on approach to desired heading
     * @param heading       world centric heading to target
     * @param rotationSpeed speed at which to rotate the robot
     */
    public void doHoloTurn(double heading, double rotationSpeed) {
        double headingError;

        headingError = calculateHeadingError(heading, getCurrentHeading());
        if (headingError > 2){
            setMotors(rotationSpeed, -rotationSpeed, rotationSpeed, -rotationSpeed);
        }

        while(opModeIsActive() && (headingError > 2)) {
            headingError = calculateHeadingError(heading, getCurrentHeading());
        }
        //Done so turn off motors
        setMotors(0, 0, 0, 0);
    }

    /**
     * Move the robot base in a given direction with respect to the robot whilst trying to head/maintain
     * 'targetHeading'
     * If the target heading is not the same as the initial heading then the bot will rotate towards
     * 'targetHeading' but the translation will remain world fixed towards 'direction'
     * This is the basis for field centric drive
     * @param direction     FIELD centric direction of travel. does NOT cause rotation though
     * @param speed         speed at which to move robot in 'direction'
     * @param distance      distance to travel in 'direction'
     * @param targetHeading gyro heading of robot to target/maintain during move
     * @param rotationSpeed rate at which to change/maintain heading
     */
    public void doHoloDrive(int direction, double speed, int distance, double targetHeading, double rotationSpeed) {
        double currentHeading;
        int targetTicks;
        int currentTicks;
        int FLstartCount;
        int FRstartCount;
        int BLstartCount;
        int BRstartCount;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        int FLCount;
        int FRCount;
        int BLCount;
        int BRCount;
        double[] motorSettings;
        double headingError;
        double headingCorrectionPower;

        //Where did the encoders start?
        int[] startCount =getMotorEncoders();
        FLstartCount = startCount[0];
        FRstartCount = startCount[1];
        BLstartCount = startCount[2];
        BRstartCount = startCount[3];

        //Calculate encoder ticks for required distance
        targetTicks = (int) (distance * COUNTS_PER_INCH);

        currentTicks = 0;


        while(opModeIsActive() && (currentTicks < targetTicks)){
            //Calculate if any rotation is needed
            currentHeading = getCurrentHeading();
            headingError = calculateHeadingError(targetHeading, currentHeading);
            headingCorrectionPower = -headingError * rotationSpeed;

            //Subtract the current heading to get field centric direction
            motorSettings = calculateVectorPower(direction - currentHeading, speed);

            //Now merge translation and rotation powerss
            frontLeftPower = motorSettings[0] + headingCorrectionPower;
            frontRightPower = motorSettings[1] - headingCorrectionPower;
            backLeftPower = motorSettings[2] + headingCorrectionPower;
            backRightPower = motorSettings[3] - headingCorrectionPower;

            // Send calculated power to wheels, clamping (and normalizing) if necessary
            setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            FLCount = frontLeftDrive.getCurrentPosition();
            FRCount = frontRightDrive.getCurrentPosition();
            BLCount = backLeftDrive.getCurrentPosition();
            BRCount = backRightDrive.getCurrentPosition();
            //We have 4 wheels counting, but each wheel force is 1/2 in each vector direction so need additional /2 to get effective distance traveled
            //We need to take abs() since motors can be moving 'forward' or 'backwards'. We don't actually care though
            currentTicks = (Math.abs(FLstartCount - FLCount) + Math.abs(FRstartCount - FRCount) + Math.abs(BLstartCount - BLCount) + Math.abs(BRstartCount - BRCount)) / 2;
        }
        //Done so turn off motors
        setMotors(0, 0, 0, 0);
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
     * Calculate a normalization factor to scale to/from unit circle from/to unit square
     * @param angle angle to calculate normalization factor for in degrees
     */
    private double unitNormalization(double angle){
        double normalizationAngle;

        if (angle >= 0)
            normalizationAngle = angle % 90;
        else
            normalizationAngle = (-angle) % 90;

        if (normalizationAngle >= 45) {
            normalizationAngle = 90 - normalizationAngle;
        }

        //Calculate normalization factor (unit square to unit circle transposition)
        return Math.sqrt(1 + Math.tan(Math.toRadians(normalizationAngle))) / Math.sqrt(2);
    }

    /**
     * Calculate power required to translate the bot it 'direction' and 'speed's
     * @param direction     robot centric direction of travel. Does NOT cause rotation
     * @param speed         speed at which to move robot in 'direction'
     */
    private double[] calculateVectorPower(double direction, double speed) {
        double frontLeftVectorPower;
        double frontRightVectorPower;
        double backLeftVectorPower;
        double backRightVectorPower;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double directionInRads;
        double directionInRadsRotated;
        double normalization;
        double[] result = new double[4];

        directionInRads = Math.toRadians(direction);
        directionInRadsRotated = directionInRads + (Math.PI / 4);

        //Normalization factor needs to ramp up then down every 90 degrees : e.g. /\/\/\/\ over 360 degrees
        normalization = unitNormalization(direction);

        //Calculate each wheel vector power for translations
        frontLeftVectorPower = speed * Math.sin(directionInRadsRotated);
        frontRightVectorPower = speed * Math.cos(directionInRadsRotated);
        backLeftVectorPower = speed * Math.cos(directionInRadsRotated);
        backRightVectorPower = speed * Math.sin(directionInRadsRotated);


        //Now scale the vector power to actual power (speed really, not power)
        frontLeftPower = frontLeftVectorPower / normalization;
        frontRightPower = frontRightVectorPower / normalization;
        backLeftPower = backLeftVectorPower / normalization;
        backRightPower = backRightVectorPower / normalization;

        result[0] = frontLeftPower;
        result[1] = frontRightPower;
        result[2] = backLeftPower;
        result[3] = backRightPower;

        return result;
    }


    /**
     * Auto examples
     */
    private void doAuto_1(){
        doHoloDrive(0, .5, 18, 0, .1);//Forward towards stones
        //Grab Stone
        doHoloDrive(180, .5, 4, 0, .1);//Back a bit
        doHoloDrive(-90, .5, 48, 0, .1);//Strafe left under bridge
        doHoloDrive(0, .5, 5, 0, .1);//Forward up to the foundation
        //Drop stone
        //Lower foundation grabber
        doHoloDrive(180, .5, 20, 0, .1);//Back up to wall with foundation
        //Raise foundation grabber
        doHoloDrive(90, .5, 20, 0, .1);//Strafe right to move away from foundation
        doHoloDrive(0, .5, 15, 0, .1);//Forward
        doHoloDrive(90, .5, 36, 0, .1);//Strafe right to go back to pick up new stone
        doHoloDrive(0, .5, 3, 0, .1);//Forward
        //Grab stone
        doHoloDrive(180, .5, 4, 0, .1);//Back a bit
        doHoloDrive(-90, .5, 48 + 8, -90, .1);//Strafe left under bridge and turn to face foundation
        //Drop stone
        doHoloDrive(90, .5, 48 + 8 + 8, -90, .1);//Back up away from the foundation and under the bridge
        doHoloTurn(0, .4);//Turn around so facing back towards stones again
        //Grab next stone
        doHoloTurn(-90, .4);//Turn back towards foundation
        doHoloDrive(-90, .5, 30, -90, .1);//Drive forward under bridge
        //Set lift raising to second brick level
        doHoloDrive(-90, .5, 30, -90, .1);//Continue driving forward towards foundation
        //Deposit brick on level 2 (lower, open grip, raise)
        doHoloDrive(90, .5, 8, -90, .1);//Drive backwards away from foundation
        //Set lift lowering
        doHoloDrive(90, .5, 18, -90, .1);//Drive backwards away from foundation to park line
    }

    /**
     * Tele-op example
     * Right stick will move the robot in field centric world view
     * Left stick will point the robot in the field centric direction
     * Left/right bumbers will rotate the robot
     */
    private void doTeleop(){
        double rightJoyX;
        double rightJoyY;
        double leftJoyX;
        double leftJoyY;
        boolean leftBumper;
        boolean rightBumper;
        boolean freeRotate;
        double translateDirection;
        double translateSpeed;
        double targetHeading;
        double headingError;
        double headingCorrectionPower;
        double currentHeading;
        double[] motorSettings;
        double translateDeadband = 0.1;
        double rotationSpeed;
        double rotateDeadband = 0.3;//Sets rotation deadbands
        double rotationSpeedFactor = .04;//Sets maximum rotation speed
        double manualRotationSpeed = .02;//Heading change per loop period. Will need to tune
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //Take our zero heading from our starting pose
        targetHeading = getCurrentHeading();

        while(opModeIsActive()){
            //Get the joysticks
            rightJoyX = -gamepad1.right_stick_x;
            rightJoyY = gamepad1.right_stick_y;//Up returns negative, so flip it
            leftJoyX = -gamepad1.left_stick_x;
            leftJoyY = -gamepad1.left_stick_y;
            leftBumper = gamepad1.left_bumper;
            rightBumper = gamepad1.right_bumper;
            freeRotate = !(gamepad1.left_trigger > 0.5);

            /*
            if(gamepad1.dpad_up){
                doHoloDrive(0, .5, 20, 0, .01);//Forward towards stones

            }
            if(gamepad1.dpad_down){
                doHoloDrive(180, .5, 20, 0, .01);//Forward towards stones

            }
            if(gamepad1.dpad_right){
                doHoloDrive(90, .5, 20, 0, .01);//Forward towards stones

            }
            if(gamepad1.dpad_left){
                doHoloDrive(-90, .5, 20, 0, .01);//Forward towards stones

            }
*/
            //Calculate desired translation 'speed' and  direction
            translateSpeed = Math.sqrt((rightJoyX * rightJoyX) + (rightJoyY * rightJoyY));
            if(translateSpeed > 1)
                translateSpeed = 1;

            if (translateSpeed > translateDeadband){//Create deadband and also ensure no divide by zero in atan2
                //Calculate the desired robot base direction from the right joystick
                //Forward = 0 radians (0 degrees)
                translateDirection = Math.toDegrees((Math.atan2(rightJoyX, rightJoyY)));

                //translateDirection = Math.toDegrees((Math.PI / 2) + Math.atan2(rightJoyX, rightJoyY));
                //Normalize the speed so that only hit full speed when fully pushed (really for diagonal directions)
                //Ensures joystick results between -1 and +1 for all locations in the joystick square
                //    translateSpeed = translateSpeed/(Math.sqrt(2) * unitNormalization(translateDirection));
            }
            else {
                translateDirection = 0;
                translateSpeed = 0;
            }

            //Now check if any rotation requested
            if (leftBumper) {
                targetHeading = targetHeading - manualRotationSpeed;
                rotationSpeed = rotationSpeedFactor;
            }
            else if (rightBumper) {
                targetHeading = targetHeading + manualRotationSpeed;
                rotationSpeed = rotationSpeedFactor;
            }
            else {
                /*
                rotationSpeed = Math.max(Math.abs(leftJoyX), Math.abs(leftJoyY));
                if (rotationSpeed > rotateDeadband) {
                    //targetHeading = Math.toDegrees((Math.PI / 2) - Math.atan2(leftJoyX, leftJoyY));
                    targetHeading = Math.toDegrees(Math.atan2(leftJoyX, leftJoyY));
                    if(!freeRotate){//If not free rotate then 'snap' to 90 degree positions, centered on axes
                        targetHeading = 90 * (Math.round(targetHeading / 90));
                    }
                }
                else{
                    rotationSpeed = 0;
                }*/
                rotationSpeed = 0;
            }

            //Now set the motors accordingly
            //Calculate if any rotation is needed
            currentHeading = getCurrentHeading();
            headingError = calculateHeadingError(targetHeading, currentHeading);

            headingCorrectionPower = headingError * rotationSpeedFactor;

            telemetry.addData("FR ENC : ", getMotorEncoders()[0]);
            telemetry.addData("FL ENC: ", getMotorEncoders()[1]);
            telemetry.addData("RR ENC : ", getMotorEncoders()[2]);
            telemetry.addData("RL ENC: ", getMotorEncoders()[3]);
            telemetry.addData("Target heading : ", targetHeading);
            telemetry.addData("Rotation rate : ", rotationSpeed);
            telemetry.addData("Heading error : ", headingError);


            //Subtract the current heading to get field centric direction
            motorSettings = calculateVectorPower(currentHeading - translateDirection, translateSpeed);

            //Now merge translation and rotation powers
            frontLeftPower = motorSettings[0] + headingCorrectionPower;
            frontRightPower = motorSettings[1] + headingCorrectionPower;
            backLeftPower = motorSettings[2] - headingCorrectionPower;
            backRightPower = motorSettings[3] - headingCorrectionPower;

            telemetry.addData("FL: ", frontLeftPower);
            telemetry.addData("FR: ", frontRightPower);
            telemetry.addData("BL: ", backLeftPower);
            telemetry.addData("BR: ", backRightPower);
            //And actually set the motors accordingly
            setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
        setMotors(0, 0, 0, 0);
    }
}

