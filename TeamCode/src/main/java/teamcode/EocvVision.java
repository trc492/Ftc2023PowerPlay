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

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcEocvDetector;

/**
 * This class implements EOCV Vision that provides the capability to detect color blobs and return their location
 * info.
 */
public class EocvVision extends FtcEocvDetector
{
    private static final Scalar ANNOTATE_RECT_COLOR = new Scalar(0, 255, 0);
    private final TrcDbgTrace tracer;
    private final GripPipeline gripPipeline;
    private TrcOpenCVDetector.DetectedObject[] detectedObjects = null;

    private double totalTime = 0.0;
    private long totalFrames = 0;
    private double taskStartTime = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the camera image width.
     * @param imageHeight specifies the camera image height.
     * @param cameraRect specifies the homography camera pixel rectangle, can be null if not provided.
     * @param worldRect specifies the homography world coordinate rectangle, can be null if not provided.
     * @param openCvCam specifies the OpenCV camera object.
     * @param cameraRotation specifies the camera orientation.
     * @param showEocvView specifies true to show the annotated image on robot controller screen, false to hide the
     *        image.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public EocvVision(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCam, OpenCvCameraRotation cameraRotation, boolean showEocvView, TrcDbgTrace tracer)
    {
        super(instanceName, imageWidth, imageHeight, cameraRect, worldRect, openCvCam, cameraRotation,
              showEocvView, tracer);

        this.tracer = tracer;
        gripPipeline = new GripPipeline();
    }   //EocvVision

    /**
     * This method pauses/resumes pipeline processing.
     *
     * @param enabled specifies true to start pipeline processing, false to stop.
     */
    @Override
    public void setEnabled(boolean enabled)
    {
        if (enabled && !isEnabled())
        {
            detectedObjects = null;
            totalTime = 0.0;
            totalFrames = 0;
            taskStartTime = TrcUtil.getCurrentTime();
            super.setEnabled(true);
        }
        else if (!enabled && isEnabled())
        {
            super.setEnabled(false);
            detectedObjects = null;
        }
    }   //setEnabled

    /**
     * This method returns the currently detect objects in a thread safe manner.
     *
     * @return array of detected objects.
     */
    public synchronized TrcOpenCVDetector.DetectedObject[] getDetectedObjects()
    {
        TrcOpenCVDetector.DetectedObject[] targets = detectedObjects;
        detectedObjects = null;
        return targets;
    }   //getDetectedObjects

    //
    // Implements FtcEocvDetector abstract methods.
    //

    /**
     * This method is called by EasyOpenCV.OpenCVPipeline to process an image frame. It calls the grip pipeline to
     * process the image and converts the detected object into an array of TrcOpenCvDetector.DetectedObject. It also
     * annotates the original image with rectangles around the detected objects.
     *
     * @param input specifies the image frame.
     * @return annotated image frame.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        final String funcName = "processFrame";
        TrcOpenCVDetector.DetectedObject[] targets = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }

        double startTime = TrcUtil.getCurrentTime();
        gripPipeline.process(input);
        MatOfKeyPoint detectedTargets = gripPipeline.findBlobsOutput();
        double elapsedTime = TrcUtil.getCurrentTime() - startTime;

        totalTime += elapsedTime;
        totalFrames++;
        if (tracer != null)
        {
            tracer.traceInfo(
                funcName, "AvgProcessTime=%.3f sec, FrameRate=%.1f",
                totalTime/totalFrames, totalFrames/(TrcUtil.getCurrentTime() - taskStartTime));
        }

        if (detectedTargets != null)
        {
            KeyPoint[] targetPoints = detectedTargets.toArray();
            targets = new TrcOpenCVDetector.DetectedObject[targetPoints.length];
            for (int i = 0; i < targets.length; i++)
            {
                double radius = targetPoints[i].size/2;
                targets[i] = new TrcOpenCVDetector.DetectedObject(
                    new Rect((int)(targetPoints[i].pt.x - radius), (int)(targetPoints[i].pt.y - radius),
                             (int)targetPoints[i].size, (int)targetPoints[i].size),
                    targetPoints[i].angle, targetPoints[i].response, targetPoints[i].octave, targetPoints[i].class_id);
            }
            detectedTargets.release();
            TrcOpenCVDetector.drawRectangles(input, targets, ANNOTATE_RECT_COLOR, 0);
            synchronized (this)
            {
                detectedObjects = targets;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", targets != null);
        }

        return input;
    }   //processFrame

}   //class EocvVision
