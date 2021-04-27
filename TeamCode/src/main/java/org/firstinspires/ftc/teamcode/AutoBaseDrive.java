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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



@Autonomous(name="Auto Drive: Iterative OpMode", group="Iterative Opmode")
public class AutoBaseDrive extends OpMode
{
    public enum AutoStates { wait, goToWhite, moveTo1, moveTo2, moveTo3, backUp, raiseWobble, swingFront, adjustDrop, goToGoal, backToStart }

    AutoStates curState = AutoStates.goToWhite;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor liftMotor = null;
    private Servo clawRotation = null;
    private Servo clawOpen = null;

    private RevColorSensorV3 colSense1 = null;
    private Rev2mDistanceSensor disSense1 = null, disSense2 = null;
    private RevTouchSensor touchSense1 = null;

    int square = 1;
    int leftDrop = 0;
    int backTime = 0;
    int goalTime = 0;
    double swingValue = 0;
    boolean turning = false, turnBack = false;

    BNO055IMU imu;

    Orientation angles, targetAngle = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone)

        // wheel initialization
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        // claw and lift initialization
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        clawOpen = hardwareMap.get(Servo.class, "clawOpen");

        liftMotor.setTargetPosition(3200);

        // sensor initialization
        colSense1 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor1");
        disSense1 = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor1");
        disSense2 = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor2");
        touchSense1 = hardwareMap.get(RevTouchSensor.class, "TouchSensor1");


        // set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        resetEncoders();
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");


        //frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (targetAngle == null)
            targetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        switch(curState) {
            case wait:
                moveForward(0f, 1.25f);
                break;
            case goToWhite:
                if (!(isOnWhite())){
                    moveForward(0.2f, 1.25f);
                }
                else{
                    if (square == 1) {
                        resetEncoders();
                        curState = AutoStates.moveTo1;
                    }
                    else if (square == 2)
                        curState = AutoStates.moveTo2;
                    else
                        curState = AutoStates.moveTo3;
                }
                break;
            case moveTo1:
                if (!(isOnRed()))
                    strafe(0.2f, 1.25f);
                else{
                    resetEncoders();
                    curState = AutoStates.backUp;
                }
                break;
            case moveTo2:
                break;
            case moveTo3:
                break;
            case backUp:
                if (){
                    moveForward(0.2f, 1.25f);
                    backTime--;
                }
                else{
                    curState = AutoStates.raiseWobble;
                }
                break;
            // following three states deal with motion of placing wobble and moving to left
            case raiseWobble:
                if (liftMotor.getCurrentPosition() > -75) {
                    liftMotor.setPower(0.5);
                }
                else {
                    liftMotor.setPower(0);
                    turning = true;
                    curState = AutoStates.swingFront;
                }
                break;
            case swingFront:
                if (turning && swingValue < 0.63)
                    swingValue += 0.02;
                else {
                    turning = false;
                    curState = AutoStates.adjustDrop;
                }
            case adjustDrop:
                if (leftDrop < 540){
                    strafe(-0.2f, 1.25f);
                    leftDrop++;
                }
                else{
                    curState = AutoStates.goToGoal;
                }
            case goToGoal:

                if (disSense2.getDistance(DistanceUnit.INCH) > 12)
                    moveForward(0.5f, 1.25f);
                else {
                    clawOpen.setPosition(0.75);
                    if (goalTime < 60)
                        goalTime++;
                    else
                        curState = AutoStates.backToStart;
                }
                break;
            case backToStart:
                if (disSense1.getDistance(DistanceUnit.INCH) > 1.5)
                    moveForward(-0.8f, 1.25f);
                else{
                    curState = AutoStates.wait;
                }
                break;
        }

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Color Sensor Red reading: ", colSense1.red());
        //telemetry.addData("Color Sensor Green reading: ", colSense1.green());
        //telemetry.addData("Color Sensor Blue reading: ", colSense1.blue());
        telemetry.addData("Current Front Distance: ", disSense2.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Current Back Distance: ", disSense1.getDistance(DistanceUnit.INCH));
        telemetry.addData("State: ", curState);
        telemetry.addData("Lift encoder: ", liftMotor.getCurrentPosition());
        telemetry.update();
    }


    void setMotors(double fl, double fr, double bl, double br){
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    void moveForward(float i, float correct){
        float pl = i, pr = i;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (Math.abs(targetAngle.firstAngle) - Math.abs(angles.firstAngle) > -2f)
            // more power to left
            pl *= correct;
        else if (Math.abs(targetAngle.firstAngle) - Math.abs(angles.firstAngle) < 2f)
            pr *= correct;

        frontLeftDrive.setPower(pl);
        frontRightDrive.setPower(pr);
        backLeftDrive.setPower(pl);
        backRightDrive.setPower(pr);

      //  telemetry.addData("Left Side: ", pl);
      //  telemetry.addData("Right Side: ", pr);

    }
    void turn(int i){
        frontLeftDrive.setPower(i);
        frontRightDrive.setPower(-i);
        backLeftDrive.setPower(i);
        backRightDrive.setPower(-i);
    }
    void strafe(float i, float correct){

        float pl = i, pr = i;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (Math.abs(targetAngle.firstAngle) - Math.abs(angles.firstAngle) > -2f)
            // more power to left
            pl *= correct;
        else if (Math.abs(targetAngle.firstAngle) - Math.abs(angles.firstAngle) < 2f)
            pr *= correct;

        frontLeftDrive.setPower(pl);
        frontRightDrive.setPower(-pl);
        backLeftDrive.setPower(-pl);
        backRightDrive.setPower(pl);
    }


    boolean isTouching(int rt, int gt, int bt){
        colSense1.getRawLightDetected();
        double r = colSense1.red();
        double g = colSense1.green();
        double b = colSense1.blue();

        return (r > rt && g > gt && b > bt);
    }

    boolean isTouching(int rt, int gt, int bt, int rtt, int gtt, int btt ){
        colSense1.getRawLightDetected();
        double r = colSense1.red();
        double g = colSense1.green();
        double b = colSense1.blue();

        return (r > rt && r < rtt &&
                g > gt && g < gtt &&
                b > bt && b < btt);

    }

    boolean isOnRed(){
        telemetry.addData("Red:",isTouching (93, 95, 63,150,150,150));
        return isTouching (93, 95, 63,150,150,150);
    }

    boolean isOnWhite(){
        telemetry.addData("White:" , isTouching(180, 180, 180));
        return isTouching(150, 150, 150);
    }

    boolean isOnBlue(){
        telemetry.addData("Blue:",  isTouching(70, 50, 115, 150,150,160));
        return isTouching(70, 50, 115, 150,150,160);
    }

    boolean isOnGrey(){
        return isTouching(40, 80, 60);
    }

    void resetEncoders(){
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
