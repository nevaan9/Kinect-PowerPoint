using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Diagnostics;


using Microsoft.Kinect;
using Microsoft.Kinect.VisualGestureBuilder;
using System.Runtime.InteropServices;
//using KinectMouseController;
using Coding4Fun.Kinect.Wpf;
using Nui = Microsoft.Kinect;
using System.Windows.Forms;


namespace bodySkeleton
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public enum Mode
        {
            Color,
            Depth,
            Infrared
        }

        // Mouse control variables!
        [DllImport("user32.dll")]
        static extern bool SetCursorPos(int X, int Y);


        // SEND KEYS -- USE THIS API TO PRESS ANY KEY
        [DllImport("user32.dll", SetLastError = true)]
        static extern void keybd_event(byte bVk, byte bScan, uint dwFlags, UIntPtr dwExtraInfo);
        public static void PressKey(Keys key, bool up)
        {
            const int KEYEVENTF_EXTENDEDKEY = 0x1;
            const int KEYEVENTF_KEYUP = 0x2;
            if (up)
            {
                keybd_event((byte)key, 0x45, KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP, (UIntPtr)0);
            }
            else
            {
                keybd_event((byte)key, 0x45, KEYEVENTF_EXTENDEDKEY, (UIntPtr)0);
            }
        }


        // MOUSE Click Control 
        [System.Runtime.InteropServices.DllImport("user32.dll")]
        public static extern void mouse_event(int dwFlags, int dx, int dy, int cButtons, int dwExtraInfo);
        public const int MOUSEEVENTF_LEFTDOWN = 0x02;
        public const int MOUSEEVENTF_LEFTUP = 0x04;

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
        int count;

        bool _displayBody = false;
        bool isForwardGestureActive = false;
        bool isBackGestureActive = false;
        bool isB = false;
        bool isW = false;
        bool isCTLP = false;
        bool leftHandRaisedGestureActive = false;
        bool fullScreenOn = false;
        bool turnedCursorOn = false;
        bool isLeftGrabGestureActive = false;
        bool isRightGrabGestureActive = false;
        bool isLeftLassoGestureActive = false;
        bool isRightLassoGestureActive = false;
        bool currentOnArrow = true;
        bool currentOnPen = false;
        bool currentOnLazer = false;
        bool dimentionsSet = false;
        bool programStarted = false;
        bool setFirstDimention = false;
        bool setSecondDimention = false;

        Mode _mode = Mode.Color;


        // Gesture Builder
        private VisualGestureBuilderDatabase gestureDatabase;
        private VisualGestureBuilderFrameSource gestureFrameSource;
        private VisualGestureBuilderFrameReader gestureFrameReader;

        // Gestures
        private Gesture point_right;
        private Gesture point_left;
        private Gesture pen;
        private Gesture block_left;
        private Gesture block_right;


        //Joints
        private Joint rightHand;



        Queue<Point> pointQueue;
        Point[] point_window;
        Point relativePoint;


        public MainWindow()
        {
            InitializeComponent();

            // Obtain the sensor and start it up
            _sensor = KinectSensor.GetDefault(); // Different than article
            count = 0;

            pointQueue = new Queue<Point>();

            

            if (_sensor != null)
            {
                _sensor.Open();
            }

            // Gestures
            //gestureDatabase = new VisualGestureBuilderDatabase(@"../../Pointing/PointSolution.gbd");
            gestureFrameSource = new VisualGestureBuilderFrameSource(_sensor, 0);
            /*
            foreach (var gesture in gestureDatabase.AvailableGestures)
            {

                if (gesture.Name == "Point_Right")
                {
                    
                    point_right = gesture;

                
                }
                if (gesture.Name == "Point_Left")
                {
                    point_left = gesture;
                    
                }

                if (gesture.Name == "Greet") {

                    pen = gesture;
                }

                if (gesture.Name == "Block_Right") {

                    block_right = gesture;

                }


                if (gesture.Name == "Block_Left") {

                    block_left = gesture;

                }

                this.gestureFrameSource.AddGesture(gesture);
            }*/

            gestureFrameReader = gestureFrameSource.OpenReader();
            gestureFrameReader.IsPaused = true;
            gestureFrameReader.FrameArrived += gestureFrameReader_FrameArrived; 

            // Specify the requires streams
            _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);

            // Add an event handler
            _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Get a reference to the multi-frame
            var reference = e.FrameReference.AcquireFrame();


            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Color)
                    {
                        camera.Source = ToBitmap(frame);
                    }
                }
            }

            // Depth
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Depth)
                    {
                        camera.Source = ToBitmap(frame);
                    }
                }
            }

            // Infrared
            using (var frame = reference.InfraredFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Infrared)
                    {
                        camera.Source = ToBitmap(frame);
                    }
                }
            }



            // Open body frame
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    // Do something with the body frame...
                    canvas.Children.Clear();

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];

                    frame.GetAndRefreshBodyData(_bodies);

                    foreach (var body in _bodies)
                    {
                        if (body.IsTracked)
                        {
                            DrawSkeleton(body);

                            gestureFrameSource.TrackingId = body.TrackingId;
                            gestureFrameReader.IsPaused = false;


                            Joint head = body.Joints[JointType.Head];
                            Joint lefthand = body.Joints[JointType.HandLeft];
                            rightHand = body.Joints[JointType.HandRight];
                            Joint waist = body.Joints[JointType.SpineBase];
                            Joint leftThumb = body.Joints[JointType.ThumbLeft];
                            Joint rightThumb = body.Joints[JointType.ThumbRight];


                            CameraSpacePoint position = rightHand.Position;

                            DepthSpacePoint handPt = _sensor.CoordinateMapper.MapCameraPointToDepthSpace(position);


                            // MOUSE SMOOTHER!
                            double n = 1.5;
                            int size = 5;
                            double cx = Math.Pow(handPt.X, n) * (SystemParameters.PrimaryScreenWidth / Math.Pow(_sensor.DepthFrameSource.FrameDescription.Width, n));
                            double cy = Math.Pow(handPt.Y, n) * (SystemParameters.PrimaryScreenHeight / Math.Pow(_sensor.DepthFrameSource.FrameDescription.Height, n));


                            double avgx = cx;
                            double avgy = cy;

                            pointQueue.Enqueue(new Point(avgx, avgy));

                            if (pointQueue.Count == size)
                            {
                                for (int i = 0; i < pointQueue.Count; i++)
                                {

                                    var aPoint = pointQueue.Dequeue();

                                    avgx += aPoint.X;
                                    avgy += aPoint.Y;

                                    pointQueue.Enqueue(aPoint);
                                }

                                relativePoint.X = avgx / 5;
                                relativePoint.Y = avgy / 5;
                                pointQueue.Dequeue();

                            }
                            else
                            {
                                relativePoint.X = cx;
                                relativePoint.Y = cy;
                            }

                            //enter fullscreen 
                            if (leftThumb.Position.Y > head.Position.Y)
                            {
                                if (!leftHandRaisedGestureActive)
                                {
                                    if (!fullScreenOn)
                                    {
                                        pressF5();
                                        leftHandRaisedGestureActive = true;
                                        fullScreenOn = true;
                                    }
                                    else
                                    {
                                        System.Windows.Forms.SendKeys.SendWait("{ESC}");
                                        leftHandRaisedGestureActive = true;
                                        fullScreenOn = false;
                                    }

                                }

                            }
                            else
                            {

                                leftHandRaisedGestureActive = false;

                            }

                            if (!fullScreenOn)
                            {
                                switch (body.HandRightState)
                                {
                                    case HandState.Closed:
                                        if (!isRightLassoGestureActive)
                                        {

                                            Trace.WriteLine("Position: " + position.X + ", " + position.Y);
                                            Trace.WriteLine("Handpt: " + handPt.X + ", " + handPt.Y);
                                            Trace.WriteLine("Relative Point: " + relativePoint.X + " ," + relativePoint.Y);
                                            Trace.WriteLine("");
                                            isRightLassoGestureActive = true;
                                        }
                                        break;

                                    case HandState.Open:
                                        isRightLassoGestureActive = false;
                                        break;
                                }
                            }


                            // These gestures work only if we ar eon fullscreen of the powerpoint 
                            if (fullScreenOn)
                            {

                                //RIGHT HAND GESTURES
                                switch (body.HandRightState)
                                {


                                    // Switch through pen modes using RIGHT HAND
                                    case HandState.Lasso:
                                        if (!isRightLassoGestureActive)
                                        {
                                            if (currentOnArrow)
                                            {
                                                pressCTL_P();
                                                isRightLassoGestureActive = true;
                                                currentOnPen = true;
                                                currentOnArrow = false;
                                            }
                                            else if (currentOnPen)
                                            {
                                                pressCTL_L();
                                                isRightLassoGestureActive = true;
                                                currentOnPen = false;
                                                currentOnLazer = true;
                                            }
                                            else
                                            {
                                                pressCTL_A();
                                                isRightLassoGestureActive = true;
                                                currentOnLazer = false;
                                                currentOnArrow = true;
                                            }

                                        }
                                        break;


                                    case HandState.Closed:
                                        if (!isRightGrabGestureActive)
                                        {
                                            System.Windows.Forms.SendKeys.SendWait("{Right}");
                                            isRightGrabGestureActive = true;
                                        }
                                        break;


                                    case HandState.Open:
                                        isRightLassoGestureActive = false;
                                        isRightGrabGestureActive = false;
                                        break;
                                }




                                // LEFT HAND GESTURES
                                switch (body.HandLeftState)
                                {

                                    // USE THE LASSO TO DRAG ONLY IF THE PEN MODE IS ON
                                    case HandState.Lasso:
                                        LeftMouseDrag((int)relativePoint.X, (int)relativePoint.Y);
                                        break;


                                    case HandState.Closed:
                                        if (!isLeftGrabGestureActive)
                                        {
                                            System.Windows.Forms.SendKeys.SendWait("{Left}");
                                            isLeftGrabGestureActive = true;
                                        }
                                        break;

                                    case HandState.Open:
                                        if (currentOnPen)
                                        {
                                            mouse_event(MOUSEEVENTF_LEFTUP, (int)relativePoint.X, (int)relativePoint.Y, 0, 0);
                                        }
                                        isLeftGrabGestureActive = false;
                                        break;
                                }


                            }

                            //-------------------------------------------------------------------------------------------------------------------------------------------------------------

                            if (turnedCursorOn)
                            {
                                //if (handPt.X > firstXPoint && handPt.X < secondXPoint)
                                //{
                                SetCursorPos((int)relativePoint.X, (int)relativePoint.Y);
                                //}
                            }


                            //}
                            //else
                            //{
                            //    switch (body.HandLeftState)
                            //    {
                            //        // USE HAND TO SET DIMENTIONS
                            //        case HandState.Closed:
                            //            if (!isLeftGrabGestureActive)
                            //            {
                            //                firstXPoint = handPt.X;
                            //                isLeftGrabGestureActive = true;
                            //                Trace.WriteLine("Set first dimention to: " + firstXPoint);
                            //                setFirstDimention = true;
                            //            }
                            //            break;

                            //        case HandState.Open:
                            //            isLeftGrabGestureActive = false;
                            //            break;
                            //    }


                            //    switch (body.HandRightState)
                            //    {

                            //        case HandState.Closed:
                            //            if (!isRightGrabGestureActive)
                            //            {
                            //                secondXPoint = handPt.X;
                            //                isRightGrabGestureActive = true;
                            //                Trace.WriteLine("Set second dimention to: " + secondXPoint);
                            //                setSecondDimention = true;
                            //            }
                            //            break;

                            //        case HandState.Open:
                            //            isRightGrabGestureActive = false;
                            //            break;
                            //    }

                            //    if (setFirstDimention && setSecondDimention)
                            //    {
                            //        dimentionsSet = true;
                            //    }


                            //}
                        }

                    }
                }
            }
        }

        // PRESS KEYS

        //Press CTRL+P
        private void pressCTL_P()
        {
            PressKey(Keys.ControlKey, false);
            PressKey(Keys.P, false);
            PressKey(Keys.P, true);
            PressKey(Keys.ControlKey, true);
        }

        //Press CTRL+L
        private void pressCTL_L()
        {
            PressKey(Keys.ControlKey, false);
            PressKey(Keys.L, false);
            PressKey(Keys.L, true);
            PressKey(Keys.ControlKey, true);
        }

        //Press CTRL+A
        private void pressCTL_A()
        {
            PressKey(Keys.ControlKey, false);
            PressKey(Keys.A, false);
            PressKey(Keys.A, true);
            PressKey(Keys.ControlKey, true);
        }

        //Press F5
        private void pressF5()
        {
            PressKey(Keys.F5, false);
            PressKey(Keys.F5, true);

        }

        // DRAG THE MOUSE!
        public static void LeftMouseDrag(int xpos, int ypos)
        {
            SetCursorPos(xpos, ypos);
            mouse_event(MOUSEEVENTF_LEFTDOWN, xpos, ypos, 0, 0);

        }


        //CLICK THE MOUSE
        public static void LeftMouseClick(int xpos, int ypos)
        {
            SetCursorPos(xpos, ypos);
            mouse_event(MOUSEEVENTF_LEFTDOWN, xpos, ypos, 0, 0);
            mouse_event(MOUSEEVENTF_LEFTUP, xpos, ypos, 0, 0);
        }



        // VISUAL GESTURE BUILDER METHODS
        private void gestureFrameReader_FrameArrived(object sender, VisualGestureBuilderFrameArrivedEventArgs e)
        {
            return;
            using (var gestureFrame = e.FrameReference.AcquireFrame())
            {

                if (gestureFrame != null && gestureFrame.DiscreteGestureResults != null)
                {
                    // TURN CURSOR ON
                    var pen_gesture = gestureFrame.DiscreteGestureResults[pen];
                    if (pen_gesture.Detected && pen_gesture.Confidence > .95)
                    {

                        if (!isCTLP)
                        {
                            if (!turnedCursorOn)
                            {
                                isCTLP = true;
                                turnedCursorOn = true;
                            }
                            else
                            {
                                isCTLP = true;
                                turnedCursorOn = false;
                            }
                        }

                    }
                    else
                    {

                        isCTLP = false;
                    }


                    // USE THESE GESTURES ONLY IF IT'S IN FULLSCREEN 

                    if (fullScreenOn)
                    {


                        // WHITE SCREEN 
                        var whiteScreen = gestureFrame.DiscreteGestureResults[block_right];
                        if (whiteScreen.Detected && whiteScreen.Confidence > .95)
                        {

                            if (!isW)
                            {
                                System.Windows.Forms.SendKeys.SendWait("W");
                                isW = true;


                            }

                        }
                        else
                        {

                            isW = false;
                        }


                        // BLACK SCREEN
                        var blackScreen = gestureFrame.DiscreteGestureResults[block_left];
                        if (blackScreen.Detected && blackScreen.Confidence > .95)
                        {

                            if (!isB)
                            {
                                System.Windows.Forms.SendKeys.SendWait("B");
                                isB = true;


                            }

                        }
                        else
                        {

                            isB = false;
                        }
                    }
                }
            }
        }

        public void DrawSkeleton(Body body)
        {
            if (body == null) return;

            // Draw the joints
            foreach (Joint joint in body.Joints.Values)
            {
                DrawJoint(joint);
            }

            // Draw the bones
            DrawLine(body.Joints[JointType.Head], body.Joints[JointType.Neck]);
            DrawLine(body.Joints[JointType.Neck], body.Joints[JointType.SpineShoulder]);
            DrawLine(body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderLeft]);
            DrawLine(body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderRight]);
            DrawLine(body.Joints[JointType.SpineShoulder], body.Joints[JointType.SpineMid]);
            DrawLine(body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ElbowLeft]);
            DrawLine(body.Joints[JointType.ShoulderRight], body.Joints[JointType.ElbowRight]);
            DrawLine(body.Joints[JointType.ElbowLeft], body.Joints[JointType.WristLeft]);
            DrawLine(body.Joints[JointType.ElbowRight], body.Joints[JointType.WristRight]);
            DrawLine(body.Joints[JointType.WristLeft], body.Joints[JointType.HandLeft]);
            DrawLine(body.Joints[JointType.WristRight], body.Joints[JointType.HandRight]);
            DrawLine(body.Joints[JointType.HandLeft], body.Joints[JointType.HandTipLeft]);
            DrawLine(body.Joints[JointType.HandRight], body.Joints[JointType.HandTipRight]);
            DrawLine(body.Joints[JointType.HandTipLeft], body.Joints[JointType.ThumbLeft]);
            DrawLine(body.Joints[JointType.HandTipRight], body.Joints[JointType.ThumbRight]);
            //DrawLine(body.Joints[JointType.SpineMid], body.Joints[JointType.SpineBase]);
            //DrawLine(body.Joints[JointType.SpineBase], body.Joints[JointType.HipLeft]);
            //DrawLine(body.Joints[JointType.SpineBase], body.Joints[JointType.HipRight]);
            //DrawLine(body.Joints[JointType.HipLeft], body.Joints[JointType.KneeLeft]);
            //DrawLine(body.Joints[JointType.HipRight], body.Joints[JointType.KneeRight]);
            //DrawLine(body.Joints[JointType.KneeLeft], body.Joints[JointType.AnkleLeft]);
            //DrawLine(body.Joints[JointType.KneeRight], body.Joints[JointType.AnkleRight]);
            //DrawLine(body.Joints[JointType.AnkleLeft], body.Joints[JointType.FootLeft]);
            //DrawLine(body.Joints[JointType.AnkleRight], body.Joints[JointType.FootRight]);

        }

        public void DrawJoint(Joint joint)
        {
            if (joint.TrackingState == TrackingState.Tracked)
            {
                // 3D space point
                CameraSpacePoint jointPosition = joint.Position;

                // 2D space point
                Point point = new Point();

                ColorSpacePoint colorPoint = _sensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);

                // Handle inferred points
                point.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
                point.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;

                // Draw an ellipse for that joint
                Ellipse ellipse = new Ellipse { Fill = Brushes.Red, Width = 30, Height = 30 };

                Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
                Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

                canvas.Children.Add(ellipse);
            }
        }

        public void DrawLine(Joint first, Joint second)
        {
            if (first.TrackingState == TrackingState.NotTracked || second.TrackingState == TrackingState.NotTracked) return;

            // Joint data is in Camera XYZ coordinates
            // 3D space point
            CameraSpacePoint jointFirstPosition = first.Position;
            CameraSpacePoint jointSecondPosition = second.Position;

            // 2D space points in XY coordinates
            Point pointFirst = new Point();
            Point pointSecond = new Point();

            // Apply COORDINATE MAPPING - Here mapping to ColorSpace
            ColorSpacePoint colorPointFirst = _sensor.CoordinateMapper.MapCameraPointToColorSpace(jointFirstPosition);
            ColorSpacePoint colorPointSecond = _sensor.CoordinateMapper.MapCameraPointToColorSpace(jointSecondPosition);

            // Handle inferred points
            pointFirst.X = float.IsInfinity(colorPointFirst.X) ? 0 : colorPointFirst.X;
            pointFirst.Y = float.IsInfinity(colorPointFirst.Y) ? 0 : colorPointFirst.Y;

            pointSecond.X = float.IsInfinity(colorPointSecond.X) ? 0 : colorPointSecond.X;
            pointSecond.Y = float.IsInfinity(colorPointSecond.Y) ? 0 : colorPointSecond.Y;

            // Creat a Line using the ColorSpacePoints
            Line line = new Line
            {
                X1 = pointFirst.X,
                Y1 = pointFirst.Y,
                X2 = pointSecond.X,
                Y2 = pointSecond.Y,
                StrokeThickness = 8,
                Stroke = new SolidColorBrush(Colors.Red)
            };

            canvas.Children.Add(line);
        }


        // Convert a ColorFrame to an ImageSource
        private ImageSource ToBitmap(ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            byte[] pixels = new byte[width * height * ((format.BitsPerPixel + 7) / 8)];

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(pixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        // Convert a DepthFrame to an ImageSource
        private ImageSource ToBitmap(DepthFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            ushort minDepth = frame.DepthMinReliableDistance;
            ushort maxDepth = frame.DepthMaxReliableDistance;

            ushort[] depthData = new ushort[width * height];
            byte[] pixelData = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            frame.CopyFrameDataToArray(depthData);

            int colorIndex = 0;
            for (int depthIndex = 0; depthIndex < depthData.Length; ++depthIndex)
            {
                ushort depth = depthData[depthIndex];
                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                pixelData[colorIndex++] = intensity; // Blue
                pixelData[colorIndex++] = intensity; // Green
                pixelData[colorIndex++] = intensity; // Red

                ++colorIndex;
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixelData, stride);
        }

        // Convert an InfraredFrame to an ImageSource
        private ImageSource ToBitmap(InfraredFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            ushort[] infraredData = new ushort[width * height];
            byte[] pixelData = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            frame.CopyFrameDataToArray(infraredData);

            int colorIndex = 0;
            for (int infraredIndex = 0; infraredIndex < infraredData.Length; ++infraredIndex)
            {
                ushort ir = infraredData[infraredIndex];
                byte intensity = (byte)(ir >> 8);

                pixelData[colorIndex++] = intensity; // Blue
                pixelData[colorIndex++] = intensity; // Green   
                pixelData[colorIndex++] = intensity; // Red

                ++colorIndex;
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixelData, stride);
        }
    }
}
