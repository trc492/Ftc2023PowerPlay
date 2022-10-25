/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvAprilTagPipeline;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements Vuforia/TensorFlow/Grip/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "EasyOpenCV";
    public static final String LABEL_BOLT = "1 Bolt";
    public static final String LABEL_BULB = "2 Bulb";
    public static final String LABEL_PANEL = "3 Panel";
    public static final String SAW_TARGET = "SawTarget";
    public static final String GOT_TARGET = "GotTarget";
    public static final String IMAGE1_NAME = "Red Audience Wall";
    public static final String IMAGE2_NAME = "Red Rear Wall";
    public static final String IMAGE3_NAME = "Blue Audience Wall";
    public static final String IMAGE4_NAME = "Blue Rear Wall";
    public static final String[] TARGET_LABELS = {LABEL_BOLT, LABEL_BULB, LABEL_PANEL};
    public static final String DRIVE_ORIENTATION_FIELD = "FieldMode";
    public static final String DRIVE_ORIENTATION_ROBOT = "RobotMode";
    public static final String DRIVE_ORIENTATION_INVERTED = "InvertedMode";

    private final TrcRevBlinkin.Pattern[] ledPatternPriorities =
        {
            new TrcRevBlinkin.Pattern(LABEL_BOLT, TrcRevBlinkin.RevLedPattern.SolidRed),
            new TrcRevBlinkin.Pattern(LABEL_BULB, TrcRevBlinkin.RevLedPattern.SolidGreen),
            new TrcRevBlinkin.Pattern(LABEL_PANEL, TrcRevBlinkin.RevLedPattern.SolidBlue),
            new TrcRevBlinkin.Pattern(SAW_TARGET, TrcRevBlinkin.RevLedPattern.SolidViolet),
            new TrcRevBlinkin.Pattern(GOT_TARGET, TrcRevBlinkin.RevLedPattern.SolidAqua),
            new TrcRevBlinkin.Pattern(IMAGE1_NAME, TrcRevBlinkin.RevLedPattern.FixedStrobeRed),
            new TrcRevBlinkin.Pattern(IMAGE2_NAME, TrcRevBlinkin.RevLedPattern.FixedStrobeBlue),
            new TrcRevBlinkin.Pattern(IMAGE3_NAME, TrcRevBlinkin.RevLedPattern.FixedLightChaseRed),
            new TrcRevBlinkin.Pattern(IMAGE4_NAME, TrcRevBlinkin.RevLedPattern.FixedLightChaseBlue),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_FIELD, TrcRevBlinkin.RevLedPattern.SolidYellow),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_ROBOT, TrcRevBlinkin.RevLedPattern.SolidWhite),
            new TrcRevBlinkin.Pattern(DRIVE_ORIENTATION_INVERTED, TrcRevBlinkin.RevLedPattern.SolidGray)
        };

    private final Robot robot;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public EocvVision frontEocvVision;
    public EocvVision elevatorEocvVision;

    private int lastSignal = 0;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        this.robot = robot;
        if (RobotParams.Preferences.useEasyOpenCV)
        {
            int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(
                cameraViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
            OpenCvCamera frontWebcam =
                OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_FRONT_WEBCAM),
                    viewportContainerIds[0]);
            OpenCvCamera elevatorWebcam =
                OpenCvCameraFactory.getInstance().createWebcam(
                    opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_ELEVATOR_WEBCAM),
                    viewportContainerIds[1]);

            frontEocvVision = new EocvVision(
                "frontEocvVision", RobotParams.FRONTCAM_IMAGE_WIDTH, RobotParams.FRONTCAM_IMAGE_HEIGHT,
                RobotParams.cameraRect, RobotParams.worldRect, frontWebcam, OpenCvCameraRotation.UPRIGHT,
                RobotParams.Preferences.showEasyOpenCvView, null);
            elevatorEocvVision = new EocvVision(
                "elevatorEocvVision", RobotParams.ELEVATORCAM_IMAGE_WIDTH, RobotParams.ELEVATORCAM_IMAGE_HEIGHT,
                null, null, elevatorWebcam, OpenCvCameraRotation.SIDEWAYS_LEFT,
                RobotParams.Preferences.showEasyOpenCvView, null);
        }
        else if (RobotParams.Preferences.useVuforia || RobotParams.Preferences.useTensorFlow)
        {
            final String VUFORIA_LICENSE_KEY =
                "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
                "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
                "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
                "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
            //
            // If no camera view ID, do not activate camera monitor view to save power.
            //
            VuforiaLocalizer.Parameters vuforiaParams =
                RobotParams.Preferences.showVuforiaView?
                    new VuforiaLocalizer.Parameters(cameraViewId): new VuforiaLocalizer.Parameters();

            vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_FRONT_WEBCAM);
            vuforiaParams.useExtendedTracking = false;
            vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

            vuforiaVision = RobotParams.Preferences.useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
            tensorFlowVision = RobotParams.Preferences.useTensorFlow? new TensorFlowVision(vuforia, null): null;
        }
    }   //Vision

    /**
     * This method sets up the Blinkin with a priority pattern list and a pattern name map.
     */
    public void setupBlinkin()
    {
        robot.blinkin.setPatternPriorities(ledPatternPriorities);
    }   //setupBlinkin

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        if (tensorFlowVision != null)
        {
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
    }   //tensorFlowShutdown

    /**
     * This method does not initiate detection. It simply returns the signal detected by the last call to
     * getDetectedSignal. This is typically used to get the last detected signal during the init period.
     *
     * @return last detected signal.
     */
    public int getLastSignal()
    {
        return lastSignal;
    }   //getLastSignal

    /**
     * This method determines the signal value from the detected object info.
     *
     * @return signal value of the detected object, 0 if no detected object.
     */
    public int determineDetectedSignal(TrcVisionTargetInfo<?> target)
    {
        int detectedSignal = 0;

        if (target != null)
        {
            if (tensorFlowVision != null)
            {
                FtcTensorFlow.DetectedObject detectedObj = (FtcTensorFlow.DetectedObject) target.detectedObj;

                switch (detectedObj.label)
                {
                    case LABEL_BOLT:
                        detectedSignal = 1;
                        break;

                    case LABEL_BULB:
                        detectedSignal = 2;
                        break;

                    case LABEL_PANEL:
                        detectedSignal = 3;
                        break;
                }
            }
            else if (frontEocvVision != null && frontEocvVision.getPipeline() instanceof FtcEocvAprilTagPipeline)
            {
                FtcEocvAprilTagPipeline.DetectedObject detectedObj =
                    (FtcEocvAprilTagPipeline.DetectedObject) target.detectedObj;

                detectedSignal = detectedObj.object.id;
            }
        }

        if (detectedSignal != 0)
        {
            lastSignal = detectedSignal;
            if (robot.blinkin != null)
            {
                // Turn off previous detection indication.
                robot.blinkin.setPatternState(Vision.LABEL_BOLT, false);
                robot.blinkin.setPatternState(Vision.LABEL_BULB, false);
                robot.blinkin.setPatternState(Vision.LABEL_PANEL, false);

                switch (detectedSignal)
                {
                    case 1:
                        robot.blinkin.setPatternState(Vision.LABEL_BOLT, true);
                        break;

                    case 2:
                        robot.blinkin.setPatternState(Vision.LABEL_BULB, true);
                        break;

                    case 3:
                        robot.blinkin.setPatternState(Vision.LABEL_PANEL, true);
                        break;
                }
            }
        }

        return detectedSignal;
    }   //determineDetectedSignal

    /**
     * This method calls the appropriate vision detection to detect the signal position.
     *
     * @return detected signal position, 0 if none detected.
     */
    public int getDetectedSignal()
    {
        return determineDetectedSignal(getBestDetectedTargetInfo(null));
    }   //getDetectedSignal

    /**
     * This method returns an array of the detected targets info.
     *
     * @param label specifies the target label, only valid for TensorFlowVision, null for others.
     * @return an array of detected targets info.
     */
    public TrcVisionTargetInfo<?>[] getDetectedTargetsInfo(String label)
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null && tensorFlowVision.isEnabled())
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                label, null, this::compareConfidence,
                RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.FRONTCAM_HEIGHT_OFFSET);
        }
        else if (frontEocvVision != null && frontEocvVision.isEnabled())
        {
            targets = frontEocvVision.getDetectedTargetsInfo(
                null, null, RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.FRONTCAM_HEIGHT_OFFSET);
        }
        else if (elevatorEocvVision != null && elevatorEocvVision.isEnabled())
        {
            targets = elevatorEocvVision.getDetectedTargetsInfo(null, null, 0.0, 0.0);
        }

        return targets;
    }   //getDetectedTargetsInfo

    /**
     * This method returns the best detected target info.
     *
     * @param label specifies the target label, only valid for TensorFlowVision, null for others.
     * @return best detected target info.
     */
    public TrcVisionTargetInfo<?> getBestDetectedTargetInfo(String label)
    {
        TrcVisionTargetInfo<?>[] targets = getDetectedTargetsInfo(label);

        return targets != null? targets[0]: null;
    }   //getDetectedTargetsInfo

    /**
     * This method returns info of the closest detected target to image center.
     *
     * @param label specifies the target label, only valid for TensorFlowVision, null for others.
     * @return closest detected target info.
     */
    public TrcVisionTargetInfo<?> getClosestTargetInfo(String label)
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null && tensorFlowVision.isEnabled())
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                label, null, this::compareDistanceFromCamera,
                RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.FRONTCAM_HEIGHT_OFFSET);
        }
        else if (frontEocvVision != null && frontEocvVision.isEnabled())
        {
            targets = frontEocvVision.getDetectedTargetsInfo(
                null, this::compareDistanceFromCamera,
                RobotParams.APRILTAG_HEIGHT_OFFSET, RobotParams.FRONTCAM_HEIGHT_OFFSET);
        }
        else if (elevatorEocvVision != null && elevatorEocvVision.isEnabled())
        {
            targets = elevatorEocvVision.getDetectedTargetsInfo(null, this::compareDistanceFromCamera, 0.0, 0.0);
        }

        return targets != null? targets[0]: null;
    }   //getClosestTargetInfo

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing object distance.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller distance to image center than b, 0 if a and b have equal distance to
     * image center, positive value if a has larger distance to image center than b.
     */
    private int compareDistanceFromCamera(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.distanceFromCamera.y) - Math.abs(b.distanceFromCamera.y))*1000);
    }   //compareDistanceFromCamera

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing camera angle.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller camera angle than b, 0 if a and b have equal camera angle, positive
     *         value if a has larger camera angle than b.
     */
    private int compareCameraAngle(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.horizontalAngle) - Math.abs(b.horizontalAngle))*1000);
    }   //compareCameraAngle

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing object size.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller area than b, 0 if a and b have equal area, positive value if a has
     *         larger area than b.
     */
    private int compareObjectSize(
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> a,
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> b)
    {
        return (int)((a.detectedObj.getArea() - b.detectedObj.getArea())*1000);
    }   //compareObjectSize

}   //class Vision
