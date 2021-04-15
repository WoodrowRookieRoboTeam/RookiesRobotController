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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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



@Autonomous(name="Test: testSensor", group="Iterative Opmode")
public class TestSensor extends OpMode
{
    public enum RobotStates { stop, drive60, driveToWhite, strafeRight20, dropWobble, strageLeft20, liftClaw, driveToWall, openClaw, backToLine }

    RobotStates curState = RobotStates.driveToWhite;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private RevColorSensorV3 colSense1 = null;
    private Rev2mDistanceSensor disSense1 = null;
    private RevTouchSensor touchSense1 = null;

    ArrayList<Double> disList = new ArrayList<Double>();

    int count = 0;
    int errCount = 0;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // sensor initialization
        colSense1 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor1");
        disSense1 = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor1");
        //touchSense1 = hardwareMap.get(RevTouchSensor.class, "TouchSensor1");


        // set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        switch (curState){
            case stop:
                break;
            case drive60:
                // read current heading & store
                if (targetAngle == null)
                    targetAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (!(goToWhite())) {
                    driveForward(0.5f, 1.25f);
                }
                else{
                    stopMotor();
                }
                break;
            case driveToWhite:
                telemetry.addData("red: ", colSense1.red());
                telemetry.addData("green: ", colSense1.green());
                telemetry.addData("blue: ", colSense1.blue());
                telemetry.update();

                colSense1.enableLed(true);


        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Color Sensor blue reading:", colSense1.getRawLightDetected());
        telemetry.addData("Current Distance:", disSense1.getDistance(DistanceUnit.INCH));
        telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        //telemetry.addData("Timer:", count++);

    }

    // scans 4 most recent distance values to counteract random discrepancies
    double getAvDis(double d){
        if(d < 310) {
            if (disList.size() > 4)
                disList.remove(0);
            disList.add(d);
        }
        else
            telemetry.addData("err count: ", errCount++);

        if(disList.size() == 0)
            return 0;

        double total = 0;
        int i;
        for (i = 0; i < disList.size(); i++){
            total += disList.get(i);

        }

        total /= i;
        telemetry.addData("Average: ", total);
        telemetry.update();
        return total;

    }

    void setMotors(double fl, double fr, double bl, double br){
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }


    // drives the robot forwards
    void driveForward(float i, float correct){

        float pl = i, pr = i;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // adjusts power to each side of robot to counteract drift
        if(Math.abs(targetAngle.firstAngle) - Math.abs(angles.firstAngle) > -2f)
            // more power to left
            pl *= correct;
        else if(Math.abs(targetAngle.firstAngle) - Math.abs(angles.firstAngle) < 2f)
            pr *= correct;

        // sets power to wheels
        frontLeftDrive.setPower(pl);
        frontRightDrive.setPower(pr);
        backLeftDrive.setPower(pl);
        backRightDrive.setPower(pr);

        // displays power for each wheel and target angle
        telemetry.addData("pl: ", pl);
        telemetry.addData("pr: ", pr);
        telemetry.addData("target: ", targetAngle.firstAngle);
        telemetry.update();

    }

    // stops the motor
    void stopMotor(){
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    // Color methods

    boolean isTouching(int rt, int gt, int bt){
        colSense1.getRawLightDetected();
        double r = colSense1.red();
        double g = colSense1.green();
        double b = colSense1.blue();

        return (r > rt && g > gt && b > bt);
    }

    boolean goToRed(){
        return isTouching (121, 95, 63);
    }

    boolean goToWhite(){
        return isTouching(235, 381, 314);
    }

    boolean goToBlue(){
        return isTouching(75, 102, 148);
    }

    boolean goToGrey(){
        return isTouching(65, 107, 85);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
