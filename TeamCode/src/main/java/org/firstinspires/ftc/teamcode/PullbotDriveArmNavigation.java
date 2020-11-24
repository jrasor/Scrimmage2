/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification,
 * are permitted (subject to the limitations in the disclaimer below)
 * provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 *  endorse or
 * promote products derived from this software without specific prior written
 *  permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 *  THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

/**
 * This OpMode uses the common Pullbot hardware class to define the devices
 * on a robot.
 * All device access is managed through the HardwarePullbot class.
 * The code is structured as a LinearOpMode.
 * <p>
 * This particular OpMode executes a Tank Drive style Teleop for a Pullbot.
 * In this mode the left stick controls the left drive wheel; the right stick
 * the right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
 * Station OpMode list.
 */

@TeleOp(name = "Pullbot: Drive, Arm, Navigate", group = "Pullbot")
//@Disabled
public class PullbotDriveArmNavigation extends LinearOpMode {

  /*
   * IMPORTANT: You need to obtain your own license key to use Vuforia. The
   *  string below with which 'parameters.vuforiaLicenseKey' is 5197's. Get your
   *  Vuforia 'Development' license key, free of charge from the Vuforia
   * developer from https://developer.vuforia.com/license-manager.
   *
   * Vuforia license keys are always 380 characters long, and look as if they
   * contain mostly random data. Once you've obtained a license key, copy the
   * string from the Vuforia web site and paste it in to your code on the next
   * line, between the double quotes, like this:
   */
  private final String VUFORIA_KEY = GenericFTCRobot.VUFORIA_KEY;
  /* Declare OpMode members. */
  Pullbot robot = new Pullbot(this);   // Use a Pullbot's hardware

  // Class Members
  private OpenGLMatrix lastLocation = null;
  private VuforiaLocalizer vuforia = null;
  private boolean targetVisible;
  private float phoneXRotate = 0;
  private float phoneYRotate = 0;
  private float phoneZRotate = 0;

  @Override
  public void runOpMode() {
        /*
                Vision.
         *
         * Configure Vuforia by creating a Parameter object, and passing it
         to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on
         the RC phone);
         * If no camera monitor is desired, use the parameter-less
         constructor instead (commented
         * out below).
         */
    int cameraMonitorViewId =
        hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id",
            hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters =
        new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer
    // .Parameters();
    parameters.vuforiaLicenseKey = GenericFTCRobot.VUFORIA_KEY;
    parameters.cameraDirection = Pullbot.CAMERA_CHOICE;

    // Make sure extended tracking is disabled for this example.
    parameters.useExtendedTracking = false;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Load the data sets for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    VuforiaTrackables targetsUltimateGoal =
        this.vuforia.loadTrackablesFromAsset("UltimateGoal");
    VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
    blueTowerGoalTarget.setName("Blue Tower Goal Target");
    VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
    redTowerGoalTarget.setName("Red Tower Goal Target");
    VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
    redAllianceTarget.setName("Red Alliance Target");
    VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
    blueAllianceTarget.setName("Blue Alliance Target");
    VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
    frontWallTarget.setName("Front Wall Target");

    // For convenience, gather together all the trackable objects in one
    // easily-iterable
    // collection
    List<VuforiaTrackable> allTrackables =
        new ArrayList<>(targetsUltimateGoal);

    /*
     * In order for localization to work, we need to tell the system
     * where each target is on
     * the field, and where the phone resides on the robot.  These
     * specifications are in the
     * form of <em>transformation matrices.</em>
     *
     * Transformation matrices are a central, important concept in the
     * math here involved in
     * localization.
     * See <a href="https://en.wikipedia
     * .org/wiki/Transformation_matrix">Transformation Matrix</a>
     * for detailed information. Commonly, you'll encounter
     * transformation matrices as instances
     * of the {@link OpenGLMatrix} class.
     *
     * If you are standing in the Red Alliance Station looking towards
     * the center of the field,
     *     - The X axis runs from your left to the right. (positive from
     * the center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the
     * other side of the field
     *       where the Blue Alliance Station is. (Positive is from the
     * center, towards the
     *       BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.
     *   (Positive is above
     *       the floor)
     *
     * Before being transformed, each target image is conceptually
     * located at the origin of the
     * field's coordinate system (the center of the field), facing up.
     */

    //Set the position of the perimeter targets with relation to origin
    // (center of field)
    redAllianceTarget.setLocation(OpenGLMatrix
        .translation(0, -Pullbot.halfField, Pullbot.mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES
            , 90,
            0, 180)));

    blueAllianceTarget.setLocation(OpenGLMatrix
        .translation(0, Pullbot.halfField, Pullbot.mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES
            , 90,
            0, 0)));
    frontWallTarget.setLocation(OpenGLMatrix
        .translation(-Pullbot.halfField, 0, Pullbot.mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES
            , 90,
            0, 90)));

    // The tower goal targets are located a quarter field length from the
    // ends of the back
    // perimeter wall.
    blueTowerGoalTarget.setLocation(OpenGLMatrix
        .translation(Pullbot.halfField, Pullbot.quarterField,
            Pullbot.mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES
            , 90,
            0, -90)));
    redTowerGoalTarget.setLocation(OpenGLMatrix
        .translation(Pullbot.halfField, -Pullbot.quarterField,
            Pullbot.mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES
            , 90,
            0, -90)));
    //
    // Create a transformation matrix describing where the phone is on
    // the robot.
    //
    // NOTE !!!!  It's very important that you turn OFF your phone's
    // Auto-Screen-Rotation option.
    // Lock it into Portrait for these numbers to work.
    //
    // Info:  The coordinate frame for the robot looks the same as the
    // field.
    // The robot's "forward" direction is facing out along X axis, with
    // the LEFT side facing out
    // along the Y axis. Z is UP on the robot.  This equates to a bearing
    // angle of Zero degrees.
    //
    // The phone starts out lying flat, with the screen facing Up and
    // with the physical top of
    // the phone pointing to the LEFT side of the Robot.
    // The two examples below assume that the camera is facing forward
    // out the front of the robot.

    // We need to rotate the camera around its long axis to bring the
    // correct camera forward.
    if (Pullbot.CAMERA_CHOICE == BACK) {
      phoneYRotate = -90;
    } else {
      phoneYRotate = 90;
    }

    // Rotate the phone vertical about the X axis if it's in portrait mode
    if (Pullbot.PHONE_IS_PORTRAIT) {
      phoneXRotate = 90;
    }

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of
    // the middle of the robot,
    // and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * Pullbot.mmPerInch;   // 4
    // Inches in front of robot
    // center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * Pullbot.mmPerInch;   // 8
    // Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT = 0;     // ON the robot's
    // center line
    // Pushbots can have their cameras located at multiple positions.
    OpenGLMatrix robotFromCamera = OpenGLMatrix
        .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
            CAMERA_VERTICAL_DISPLACEMENT)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES
            , phoneYRotate,
            phoneZRotate, phoneXRotate));

    /*  Let all the trackable listeners know where the phone is.  */
    for (VuforiaTrackable trackable : allTrackables) {
      ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(
          robotFromCamera, parameters.cameraDirection);
    }

    // Note: To use the remote camera preview:
    // AFTER you hit Init on the Driver Station, use the "options menu"
    // to select "Camera Stream"
    // Tap the preview window to receive a fresh image.

    targetsUltimateGoal.activate();

        /*
                Movement.
         */
    double left;
    double right;
    double drive;
    double turn;
    double max;

    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
    robot.init(hardwareMap);

    ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    // Send telemetry message to signify robot waiting;
    telemetry.addData("Say", "Hello Driver");    //
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // Get vision results.

      // check all the trackable targets to see which one (if any) is
      // visible.
      targetVisible = false;
      for (VuforiaTrackable trackable : allTrackables) {
        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
          telemetry.addData("Visible Target", trackable.getName());
          targetVisible = true;

          // getUpdatedRobotLocation() will return null if no new
          // information is available since
          // the last time that call was made, or if the trackable
          // is not currently visible.
          OpenGLMatrix robotLocationTransform = ((
              VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
          if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
          }
          break;
        }
      }

      // Provide feedback as to where the robot is located (if we know).
      if (targetVisible) {
        // express position (translation) of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = " +
                "%4.1f, %4.1f, %4.1f",
            translation.get(0) / Pullbot.mmPerInch,
            translation.get(1) / Pullbot.mmPerInch,
            translation.get(2) / Pullbot.mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation =
            Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ,
                DEGREES);

        telemetry.addData(
            "Rot (deg)", "{Roll, Pitch, Heading} = " +
                "%5.0f\u00B0, %5.0f\u00B0, %5.0f\u00B0",
            rotation.firstAngle, rotation.secondAngle,
            rotation.thirdAngle);
      } else {
        telemetry.addData("Visible Target", "none");
      }
      //telemetry.update();

      // Run wheels in Tank mode: the Left stick moves the robot left
      // wheel fwd and back;
      // the Right stick moves the robot right wheel fwd and back.
/*
      left = gamepad1.left_stick_y;
      right = gamepad1.right_stick_y;

      // Output the safe vales to the motor drives.
      robot.leftDrive.setPower(left);
      robot.rightDrive.setPower(right);
*/
      //robot.tankDrive();
      robot.simpleDrive();

      // Use gamepad buttons to move arm up (Y) and down (A)
      if (gamepad1.y)
        robot.arm.setPosition(robot.DEPLOYED);
      else if (gamepad1.a)
        robot.arm.setPosition(robot.STOWED);

      //NormalizedRGBA colors = colorSensor.;
      telemetry.addData("Colors",
          "Red %4d   Green %4d   Blue %4d",
          colorSensor.red(), colorSensor.green(), colorSensor.blue());
      telemetry.update();
    }

    // Disable Tracking when we are done.
    targetsUltimateGoal.deactivate();
  }
}
