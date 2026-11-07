package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous", group = "Autonomous")

public class HelperAuto extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft; //Comment this line out for tank drive
    private DcMotor backRight; //Comment this line out for tank drive

    @Override
    public void runOpMode() {
/*----------------------IMPORTANT--------------------------
        To the team that is using this, adjust the hardware mapping to the same as your drive motors are set to.
        If you are running a tank drive, comment out the backLeft and backRight motors and only map the first two.
        Go to the code underneath the: if (opModeIsActive()) { and comment out everything that has to do with the backLeft and backRight methods.
        You can comment using // at the beginning of a code line.
        Then run the code, it will be called Autonomous
        Talk to team 38473 if you need more help.
 */

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left"); //Adjust harware map for tank drive
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Uncomment this line if motor moves backwards
        frontRight = hardwareMap.get(DcMotor.class, "front_right"); //Adjust harware map for tank drive
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE); //Uncomment this line if motor moves backwards
        backLeft = hardwareMap.get(DcMotor.class, "back_left"); //Comment this line out for tank drive
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Uncomment this line if motor moves backwards
        backRight = hardwareMap.get(DcMotor.class, "back_right"); //Comment this line out for tank drive
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE); //Uncomment this line if motor moves backwards

        // Wait for start
        waitForStart();

        if (opModeIsActive()) {

            // Set all motors to 50% power
            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5); //Comment this line out for tank drive
            backRight.setPower(0.5); //Comment this line out for tank drive

            // Run for 0.5 seconds
            sleep(500);

            // Stop all motors
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0); //Comment this line out for tank drive
            backRight.setPower(0); //Comment this line out for tank drive
        }
    }
}