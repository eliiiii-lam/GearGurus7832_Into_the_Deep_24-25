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

@Autonomous(name = "The Holy Grail")
public class TheHolyGrail extends LinearOpMode {
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

//                .stopAndAdd(arm.closeClaw())
//                .waitSeconds(0.5)
//                .stopAndAdd(arm.closeClaw())
//                .stopAndAdd(arm.armUp())
//                .afterTime(0,arm.SetPosition(-475))
//                .stopAndAdd(arm.closeClaw())
                .lineToY(33)

//                .afterTime(0,arm.SetPosition(-800))

//                .waitSeconds(0.1)
//                .stopAndAdd(arm.openClaw())
//                .afterTime(0.2,arm.SetPosition(20))
//                .stopAndAdd(arm.armDown())
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(17,30), Math.toRadians(270))
//
                .splineToConstantHeading(new Vector2d(18,15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(33,10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(35 ,45), Math.toRadians(270))
               /// /////////////////////////////////////////////////////////////////////////
                .setTangent(-60)
                .splineToConstantHeading(new Vector2d(23,23), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(25,22), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(51,15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(52,45), Math.toRadians(270))
/// /////////////////////////////////////////////////////////////////////////////////////////
                .setTangent(-75)
                .splineToConstantHeading(new Vector2d(30,17), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(50,13), Math.toRadians(260))
                .splineToConstantHeading(new Vector2d(51 ,45), Math.toRadians(260))
                /////////////////////////////////////////////////////////////////////////////////////////////
                .strafeToLinearHeading(new Vector2d(38,50), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
                .lineToY(55)
//                /////////////////////////////////////////////////////////////////////////////////////////////
//                .splineToLinearHeading(new Pose2d(28, 15, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .splineToLinearHeading(new Pose2d(45, 10, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .lineToY(47)
//                ////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//                .strafeToLinearHeading(new Vector2d(26,62), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .stopAndAdd(arm.closeClaw())
//                .waitSeconds(0.15)
//                .afterTime(0,arm.SetPosition(-400))
//                .stopAndAdd(arm.armUp())
//                .strafeToLinearHeading(new Vector2d(-23,31), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .afterTime(0,arm.SetPosition(-800))
//                .waitSeconds(0.1)
//                .stopAndAdd(arm.openClaw())
//                .afterTime(0.3,arm.SetPosition(0))
//                .stopAndAdd(arm.armDown())
//////////////////////////////////////////////////////////////////////////////////////////
//                .strafeToLinearHeading(new Vector2d(26,61.5), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .stopAndAdd(arm.closeClaw())
//                .waitSeconds(0.15)
//                .afterTime(0,arm.SetPosition(-475))
//                .stopAndAdd(arm.armUp())
//                .strafeToLinearHeading(new Vector2d(-21,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .afterTime(0,arm.SetPosition(-800))
//                .waitSeconds(0.1)
//                .stopAndAdd(arm.openClaw())
//                .afterTime(0.3,arm.SetPosition(0))
//                .stopAndAdd(arm.armDown())
//////////////////////////////////////////////////////////////////////////////////////////
//                .strafeToLinearHeading(new Vector2d(26,61.5), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .stopAndAdd(arm.closeClaw())
//                .waitSeconds(0.15)
//                .afterTime(0,arm.SetPosition(-475))
//                .stopAndAdd(arm.armUp())
//                .strafeToLinearHeading(new Vector2d(-19,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .afterTime(0,arm.SetPosition(-800))
//                .waitSeconds(0.1)
//                .stopAndAdd(arm.openClaw())
//                .afterTime(0.3,arm.SetPosition(0))
//                .stopAndAdd(arm.armDown())
//////////////////////////////////////////////////////////////////////////////////////////
//                .strafeToLinearHeading(new Vector2d(26,61.5), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .stopAndAdd(arm.closeClaw())
//                .waitSeconds(0.15)
//                .afterTime(0,arm.SetPosition(-475))
//                .stopAndAdd(arm.armUp())
//                .strafeToLinearHeading(new Vector2d(-17,35), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//                .afterTime(0,arm.SetPosition(-800))
//                .waitSeconds(0.1)
//                .stopAndAdd(arm.openClaw())
//                .afterTime(1.2,arm.SetPosition(0))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(50, 60, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxProfileAccel * 3.9))
//

////////////////////////////////////////////////////////////////////////////////////////





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