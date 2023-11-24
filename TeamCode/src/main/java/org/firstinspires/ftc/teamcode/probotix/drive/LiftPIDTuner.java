

package org.firstinspires.ftc.teamcode.probotix.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.probotix.main.hardware;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled
@TeleOp(name="LiftPIDTuner", group="Linear Opmode")
public class LiftPIDTuner extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private hardware Hardware;

    //32767/2100 = 15.6

    public static PIDFCoefficients liftPID = new PIDFCoefficients(1.56,0.156,0,15.6);



    public double currentVelocity;
    public double maxVelocity = 0.0;

    @Override
    public void runOpMode() {

        this.Hardware = new hardware(hardwareMap);




        Hardware.init();
        Hardware.reset();

        Hardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Hardware.liftMotor.setVelocityPIDFCoefficients(liftPID.p,liftPID.i,liftPID.d,liftPID.f);
            Hardware.liftMotor.setPositionPIDFCoefficients(5.0);

            if (gamepad1.y){
                Hardware.getLiftMotor().setPower(1);
            } else if (gamepad1.x){
                Hardware.getLiftMotor().setPower(0.8);
            }
            /*else {
                Hardware.getLiftMotor().setPower(0);
            }*/

            //high junction
            if (gamepad1.dpad_up) {
                Hardware.liftMotor.setTargetPosition(-4180);
                //Hardware.liftMotor.setPower(0.6);
            }
            //pick up
            else if (gamepad1.dpad_down) {
                Hardware.liftMotor.setTargetPosition(-10);
                //Hardware.liftMotor.setPower(0.6);
            }
            //low junction
            else if (gamepad1.dpad_left) {
                Hardware.liftMotor.setTargetPosition(-1800);//-2000
                //Hardware.liftMotor.setPower(0.6);
            }
            //medium junction
            else if (gamepad1.dpad_right) {
                Hardware.liftMotor.setTargetPosition(-3000);
                //Hardware.liftMotor.setPower(0.6);
            }
            //pick up hover
            else if (gamepad1.a) {
                Hardware.liftMotor.setTargetPosition(-500);
                //Hardware.liftMotor.setPower(0.6);
            }

            //ground junction
            else if (gamepad1.b) {
                Hardware.liftMotor.setTargetPosition(-150);
                //Hardware.liftMotor.setPower(0.6);
            }



            currentVelocity = Hardware.getLiftMotor().getVelocity();

            if (currentVelocity > maxVelocity)
            {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current vel: ", currentVelocity);
            telemetry.addData("max vel: ", maxVelocity);
            telemetry.addData("current pos: ", Hardware.getLiftMotor().getCurrentPosition());
            telemetry.update();


        }
    }
}
