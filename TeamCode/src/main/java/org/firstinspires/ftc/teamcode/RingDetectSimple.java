/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *  all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;
import java.util.Locale;

/*
 * Test opmode. Trying to get it to call robot method CountRings.
 *   Looks for yellow rectangles, and reports their aspect ratio. It will be
 * used in
 * Ultimate Goal to count Rings in the Starter Stack.
 * o A case: no rectangles means no Rings. Put Wobble Goal in Zone A.
 * o B case: rectangle with short squat aspect ratio, 4 - 8. Go for Zone B.
 * o C case: taller rectangle, ratio 1-2. Go for Zone C.
 */

@TeleOp(name = "Ring Detect Simple", group = "Vision test")
//@Disabled
public class RingDetectSimple extends LinearOpMode {
  Pullbot robot = new Pullbot(this);
  @Override
  public void runOpMode() {
    /*
     * NOTE: Many comments have been omitted from EasyOpenCV's samples for
     * conciseness. If you're just starting out with EasyOpenCv,
     * you should take a look at {@link InternalCamera2Example} or its
     * webcam counterpart, {@link WebcamExample} first.
     */

    Pullbot.RingOrientationAnalysisPipeline pipeline;

    int ringsDetected = 0;
    // Create camera instance
   // int cameraMonitorViewId =
     //   hardwareMap.appContext.getResources().getIdentifier(
       //     "cameraMonitorViewId", "id",
         //   hardwareMap.appContext.getPackageName());
    //robot.phoneCam =
    //    OpenCvCameraFactory.getInstance().createInternalCamera2
    //    (OpenCvInternalCamera2.CameraDirection.BACK, robot.cameraMonitorViewId);
    // Open async and start streaming inside opened callback
    //robot.phoneCam.openCameraDeviceAsync(new OpenCvCamera
    // .AsyncCameraOpenListener() {
    //  @Override
    /*
      public void onOpened() {
        robot.phoneCam.startStreaming(320, 240,
       OpenCvCameraRotation.SIDEWAYS_LEFT);

        robot.pipeline = new Pullbot.RingOrientationAnalysisPipeline();
        robot.phoneCam.setPipeline(robot.pipeline);
      }
    //});

     */
    telemetry.addLine("Pipeline ready.");
    telemetry.update(); sleep (1000);
    // Tell telemetry to update faster than the default 250ms period :)
    telemetry.setMsTransmissionInterval(20);

    waitForStart();
    telemetry.addLine("Running opmode now.");
    telemetry.update(); sleep (1000);
    while (opModeIsActive()) {
      // Don't burn an insane amount of CPU cycles in this sample because
      // we're not doing anything else
      sleep(20);
      ringsDetected = robot.CountRings(); // was
      // using cameraMonitorViewId
      telemetry.addData("Rings detected",  "%2d.", ringsDetected);
      telemetry.update();

    }
  }
}