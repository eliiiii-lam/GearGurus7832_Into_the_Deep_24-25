package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Ultimate Continuity")
public class ultimate_continuity extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Start position for RED
        Pose2d initialPose = new Pose2d(-10, 61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        //  resetRuntime();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "Arm");

        if (gamepad2.right_stick_button){

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        waitForStart();

        TrajectoryActionBuilder spec1 = drive.actionBuilder(initialPose)
                .lineToY(33)
                .setReversed(true)

                .splineToLinearHeading(new Pose2d(-35,40 ,Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-35,14, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-45,12, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(true)
                .setReversed(false)
                //delete the above to test true continuity, basically let the tangent be reversed the whole time
                .splineToLinearHeading(new Pose2d(-45,50, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-45,14, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-55,12, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(true)
                .setReversed(false)

                .splineToLinearHeading(new Pose2d(-55,50, Math.toRadians(270)), Math.toRadians(270))


                .splineToLinearHeading(new Pose2d(-55,14, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-61,12, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(true)

                .splineToLinearHeading(new Pose2d(-61,50, Math.toRadians(270)), Math.toRadians(270))





                ////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(-35,61.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-2,35), Math.toRadians(270))
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(-35,61.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-2,35), Math.toRadians(270))
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(-35,61.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-2,35), Math.toRadians(270))
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(-35,61.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-2,35), Math.toRadians(270))
////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(-35,61.5), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-2,35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-35,61.5), Math.toRadians(270))



                ;














        Action TrajectoryClose = spec1.endTrajectory().build();


        if (isStopRequested()) return;




        Actions.runBlocking(


                new ParallelAction(
                        arm.lockIntake(),
                        spec1.build(),
                        arm.UpdatePID()

                )


        );
    }
}