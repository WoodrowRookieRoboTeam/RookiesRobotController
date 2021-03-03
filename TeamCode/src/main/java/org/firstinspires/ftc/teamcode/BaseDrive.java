package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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



@TeleOp(name="Basic Drive: Iterative OpMode", group="Iterative Opmode")
public class BaseDrive extends OpMode
{
    //public enum botState { off, running}

    //botState curBotState = botState.off;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

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
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;
        double angle = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
        double mag = Math.sqrt(Math.pow(gamepad1.right_stick_y, 2) + (Math.pow(gamepad1.right_stick_x, 2)));

        //frontLeftPower = -Math.sin(angle + (0.25 * Math.PI)) * mag + gamepad1.left_stick_x;
        //frontRightPower = Math.sin(angle - (0.25 * Math.PI)) * mag + gamepad1.left_stick_x;

        frontLeftPower    = Range.clip(Math.sin(angle + (0.25 * Math.PI)) * mag - gamepad1.left_stick_x, -1.0, 1.0) ;
        frontRightPower   = Range.clip(Math.sin(angle - (0.25 * Math.PI)) * mag + gamepad1.left_stick_x, -1.0, 1.0) ;
        backLeftPower = frontRightPower;
        backRightPower = frontLeftPower;

        // Show the elapsed game time and wheel power.
        telemetry.addData("LX", gamepad1.left_stick_x);
        telemetry.addData("RX", gamepad1.right_stick_x);
        telemetry.addData("RY", gamepad1.right_stick_y);
        telemetry.addData("FrontLeftPower", frontLeftPower);
        telemetry.addData("FrontRightPower", frontRightPower);
        telemetry.addData("BackLeftPower", backLeftPower);
        telemetry.addData("BackRightPower", backRightPower);
/*
        if (gamepad1.left_stick_x != 0) {
            frontLeftPower = gamepad1.left_stick_y;
            frontRightPower = gamepad1.left_stick_y;
            backLeftPower = gamepad1.left_stick_y;
            backRightPower = gamepad1.right_stick_y;
        }
*/
        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);


        // Show the elapsed game time and wheel power.
    //    telemetry.addData("Status", "Run Time: " + runtime.toString());
    //    telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /*private double getMechanamMotorPower(float right_stick_y, String m) {

        if(m.equals(("FL")) {
            return the FL motor setting for the directyion right_stick_y
        }



    }*/

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
