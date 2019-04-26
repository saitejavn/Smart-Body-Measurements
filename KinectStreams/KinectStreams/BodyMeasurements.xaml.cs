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

using RosSharp.RosBridgeClient;
using bm_msgs = RosSharp.RosBridgeClient.Messages.bmmessage;
using bm_imsgs = RosSharp.RosBridgeClient.Messages.Sensor;


using Microsoft.Kinect;
using LightBuzz.Vitruvius;
using System.IO;
using System.Runtime.InteropServices;

namespace KinectStreams
{
    /// <summary>
    /// Interaction logic for BodyMeasurements.xaml
    /// </summary>
    /// 
    public partial class BodyMeasurements : Page
    {
        String rosBMMeasurements_Id;
        String rosBMImage_Id;
        String rosBMFits_Id;
        static readonly string uri = "ws://192.168.189.130:9090";

        RosSocket rosSocket;

        Mode _mode = Mode.Color;

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;

        bool _displayBody = true;

        // Custom

        Body frontbody, sidebody;
        DepthFrame sideDepthFrame, frontDepthFrame;
        BodyIndexFrame frontBodyIndexFrame, sideBodyIndexFrame;


        // Final Camera Points

        CameraSpacePoint waist = new CameraSpacePoint();
        CameraSpacePoint neck_custom = new CameraSpacePoint();

        CameraSpacePoint chest = new CameraSpacePoint();
        CameraSpacePoint bottomPoint = new CameraSpacePoint();

        DepthSpacePoint chestRightPoint = new DepthSpacePoint();
        DepthSpacePoint chestLeftPoint = new DepthSpacePoint();

        DepthSpacePoint waistRightPoint = new DepthSpacePoint();
        DepthSpacePoint waistLeftPoint = new DepthSpacePoint();

        DepthSpacePoint bottomRightPoint = new DepthSpacePoint();
        DepthSpacePoint bottomLeftPoint = new DepthSpacePoint();

        // Pixels Data

        byte[] _frontbodyData = null;
        byte[] _sidebodyData = null;

        byte[] _finaldisplayPixels = new byte[512 * 424 * 4];

        // Flags

        int frontFrameDetected_flag = 0;
        int sideFrameDetected_flag = 0;

        double[,] garment = new double[12, 4] {
            {100, 94 , -1, 87.3 },
            {106, 100, -1, 90   },
            {112, 106, -1, 92.7 },
            {118, 112, -1, 95.5 },
            {104, 104, 104, 87  },
            {110, 110, 110, 90 },
            {116, 116, 116, 93 },
            {122, 122, 122, 96 },
            {98, 89, 98, 87.3 },
            {104, 95, 104, 90 },
            {110, 101, 110, 92.7 },
            {116, 107, 116, 95.5 }
        };

        double[,] gbdifference = new double[12, 4] {
            {8, 10, -1, 0 },
            {10, 12, -1, 0 },
            {8 , 10, -1, 0 },
            {10, 12, -1, 0 },
            {12, 14, 8, 0 },
            {14, 16, 10, 0 },
            {12, 14, 8, 0  },
            {14, 16, 10, 0 },
            {6, 9, 2, 0 },
            {8, 11, 4, 0 },
            {6, 9, 2, 0 },
            {8, 11, 4, 0 }
        };

        // 
        int stride;

        // 
        double sleeve_front = 0;
        double chest_front = 0;
        double shoulder_front = 0;
        double neck_front = 0;
        double waist_front = 0;
        double chest_side = 0;
        double neck_side = 0;
        double waist_side = 0;
        double waist_size = 0;
        double chest_size = 0;
        double bottom_front = 0;
        double bottom_side = 0;
        double bottom_size = 0;

        ushort[,] rarray_final = new ushort[424, 512];
        ushort[,] garray_final = new ushort[424, 512];
        ushort[,] barray_final = new ushort[424, 512];

        // Fits

        int chest_fit = -1;
        int waist_fit = -1;
        int bottom_fit = -1;

        int size = -1;
        int fit = -1;

        int unityui_flag = 0;

        public BodyMeasurements()
        {
            rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri));
            rosBMMeasurements_Id = rosSocket.Advertise<bm_msgs.bm_message>("/bm_MeasurementsChatter");
            rosBMImage_Id = rosSocket.Advertise<bm_imsgs.Image>("/bm_ImagesChatter");
            rosBMFits_Id = rosSocket.Advertise<bm_msgs.bm_fits>("/bm_FitsChatter");

            string rosBMSizeFit_Id = rosSocket.Subscribe<bm_msgs.bm_sizefit>("/bm_SizeFitChatter", SizeFitSubscriptionHandler);
            //rosBMSizeFit_Id = rosSocket.Subscribe<bm_msgs.bm_sizefit>("/bm_SizeFitChatter", SizeFitSubscriptionHandler);

            InitializeComponent();

            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.BodyIndex | FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);

                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

            }
        }

        private void Back_Click(object sender, RoutedEventArgs e)
        {
            if (NavigationService.CanGoBack)
            {
                NavigationService.GoBack();
            }
        }

        private void Page_Unloaded(object sender, RoutedEventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
            rosSocket.Close();
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            
            var reference = e.FrameReference.AcquireFrame();

            using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
            using (var colorFrame = reference.ColorFrameReference.AcquireFrame())
            using (var bodyIndexFrame = reference.BodyIndexFrameReference.AcquireFrame())
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {

                if (frame != null)
                {

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];
                    frame.GetAndRefreshBodyData(_bodies);


                    foreach (var body in _bodies)
                    {
                        if (body != null)
                        {
                            if (body.IsTracked)
                            {

                                // Joints

                                Joint head = body.Joints[JointType.Head];
                                Joint shoulderLeft = body.Joints[JointType.ShoulderLeft];
                                Joint shoulderRight = body.Joints[JointType.ShoulderRight];
                                Joint hipRight = body.Joints[JointType.HipRight];
                                Joint ankleRight = body.Joints[JointType.AnkleRight];
                                Joint spineBase = body.Joints[JointType.SpineBase];
                                Joint spineMid = body.Joints[JointType.SpineMid];
                                Joint shoulderMid = body.Joints[JointType.SpineShoulder];
                                Joint handRight = body.Joints[JointType.HandRight];
                                Joint kneeRight = body.Joints[JointType.KneeRight];
                                Joint kneeLeft = body.Joints[JointType.KneeLeft];
                                Joint neck = body.Joints[JointType.Neck];
                                Joint rightHandTip = body.Joints[JointType.HandTipRight];

                                CameraSpacePoint headpoint = head.Position;
                                CameraSpacePoint shoulderLeftPoint = shoulderLeft.Position;
                                CameraSpacePoint shoulderRightPoint = shoulderRight.Position;
                                CameraSpacePoint hipRightPoint = hipRight.Position;
                                CameraSpacePoint ankleRightPoint = ankleRight.Position;
                                CameraSpacePoint spineBasePoint = spineBase.Position;
                                CameraSpacePoint spineMidPoint = spineMid.Position;
                                CameraSpacePoint shoulderMidPoint = shoulderMid.Position;
                                CameraSpacePoint handRightPoint = handRight.Position;
                                CameraSpacePoint kneeRightPoint = kneeRight.Position;
                                CameraSpacePoint kneeLeftPoint = kneeLeft.Position;
                                CameraSpacePoint neckPoint = neck.Position;
                                CameraSpacePoint handTipRightPoint = rightHandTip.Position;



                                // Front View Detection and Measurements

                                if (Math.Abs(((int)(shoulderRightPoint.Z * 100) - (int)(shoulderLeftPoint.Z * 100))) <= 1)
                                {
                                    if (colorFrame != null && depthFrame != null && bodyIndexFrame != null && frontFrameDetected_flag != 1)
                                    {

                                        frontFrameDetected_flag = 1;

                                        // Front Frame Detection Check Box
                                        front_detected_CheckBox.IsChecked = true;

                                        _reader.IsPaused = false;

                                        frontDepthFrame = depthFrame;
                                        frontBodyIndexFrame = bodyIndexFrame;

                                        frontbody = body;

                                        ushort[,] frontdarray = depthFrameToPixelsData(depthFrame, bodyIndexFrame);

                                        // CSV 

                                        string csv = ArrayToCsv(frontdarray);
                                        File.WriteAllText("frontdarray.csv", csv);

                                        // Front Photo

                                        //var frontbitmap = depthFrame.ToBitmap();
                                        //frontbitmap.Save("FrontViewDepth.jpg");

                                        // Waist Point Calculation
                                        waist.X = ((spineMid.Position.X + spineBase.Position.X) / 2 + spineBase.Position.X) / 2;
                                        waist.Y = ((spineMid.Position.Y + spineBase.Position.Y) / 2 + spineBase.Position.Y) / 2;
                                        waist.Z = ((spineMid.Position.Z + spineBase.Position.Z) / 2 + spineBase.Position.Z) / 2;

                                        // Waist Measurement



                                        waist_front = fvlength(waist, frontdarray, 'w');
                                        Console.WriteLine("Waist Front Length : {0}", waist_front);

                                        waist_Front_Label.Content = waist_front.ToString();

                                        // Neck measuremnt



                                        neck_custom = neckPoint;

                                        neck_front = fvlength(neckPoint, frontdarray, 'x');
                                        Console.WriteLine("Neck Front Length  : {0}", neck_front);

                                        //neck_Front_Label.Content = neck_front.ToString();

                                        // Shoulder Front Measurement



                                        shoulder_front = Distance(shoulderLeftPoint, neckPoint) + Distance(neckPoint, shoulderRightPoint);

                                        shoulder_Front_Label.Content = shoulder_front.ToString();

                                        // Chest Front Measurement



                                        // Waist Point Calculation
                                        chest.X = ((spineMidPoint.X + shoulderMidPoint.X) / 2 + spineMidPoint.X) / 2;
                                        chest.Y = ((spineMidPoint.Y + shoulderMidPoint.Y) / 2 + spineMidPoint.Y) / 2;
                                        chest.Z = ((spineMidPoint.Z + shoulderMidPoint.Z) / 2 + spineMidPoint.Z) / 2;

                                        chest_front = fvlength(chest, frontdarray, 'c');

                                        chest_Front_Label.Content = chest_front.ToString();

                                        // Sleeve Length Measurement

                                        sleeve_front = Distance(neckPoint, shoulderRightPoint) + Distance(shoulderRightPoint, handRightPoint) + Distance(handTipRightPoint, handRightPoint);

                                        sleeve_Front_Label.Content = sleeve_front.ToString();

                                        // Bottom Length Measurement

                                        bottomPoint = hipRightPoint;
                                        bottom_front = fvlength(hipRightPoint, frontdarray, 'b');

                                        bottom_Front_Label.Content = bottom_front.ToString();

                                        // Front Frame Image Display
                                        var frontbitmap = colorFrame.GreenScreen(depthFrame, bodyIndexFrame);
                                        FrontFrameImageDisplay.Source = frontbitmap;

                                        GreenScreenCustom(colorFrame, depthFrame, bodyIndexFrame);
                                        
                                        // Sending the Front Frame Data to ROS
                                        rosImagesMessager();

                                        //string rcsv = ArrayToCsv(rarray_final);
                                        //File.WriteAllText("rarray_final.csv", rcsv);

                                    }
                                }

                                // Side View Detection and Measurement

                                if ((Math.Abs(((int)(kneeRight.Position.X * 100) - (int)(kneeLeft.Position.X * 100))) <= 1) && frontFrameDetected_flag == 1 && sideFrameDetected_flag == 0)
                                {
                                    sideDepthFrame = depthFrame;
                                    sideBodyIndexFrame = bodyIndexFrame;

                                    sidebody = body;

                                    sideFrameDetected_flag = 1;

                                    _reader.IsPaused = true;

                                    // Side Frame Detection Check Box
                                    side_detected_Checkbox.IsChecked = true;

                                    ushort[,] sidedarray = depthFrameToPixelsData(depthFrame, bodyIndexFrame);

                                    // CSV

                                    string csv = ArrayToCsv(sidedarray);
                                    File.WriteAllText("sidedarray.csv", csv);

                                    // Side Photo

                                    //var sidebitmap = depthFrame.ToBitmap();
                                    //sidebitmap.Save("SideViewDepth.jpg");

                                    // Waist Side Measurement

                                    waist_side = fvlength(waist, sidedarray, 'x');
                                    Console.WriteLine("Waist Side Length : {0}", waist_side);

                                    waist_Side_Label.Content = waist_side.ToString();

                                    waist_size = eperimetercalculator(waist_front, waist_side);

                                    waist_Size_Label.Content = waist_size.ToString();

                                    // Neck Side Measurment

                                    neck_side = fvlength(neckPoint, sidedarray, 'x');
                                    Console.WriteLine("Neck Side Length : {0}", neck_side);

                                    //neck_Side_Label.Content = neck_side.ToString();

                                    // Chest Side measurement

                                    chest_side = fvlength(chest, sidedarray, 'x');
                                    chest_Side_Label.Content = chest_side.ToString();

                                    chest_size = eperimetercalculator(chest_front, chest_side);

                                    chest_Size_Label.Content = chest_size.ToString();

                                    // Bottom Side measurement

                                    bottom_side = fvlength(bottomPoint, sidedarray, 'x');

                                    bottom_Side_Label.Content = bottom_side.ToString();

                                    bottom_size = eperimetercalculator(bottom_front, bottom_side);

                                    bottom_Size_Label.Content = bottom_size.ToString();

                                    rosMeasurementsMessager();

                                }


                                // Size and Fit measurement

                                //Console.WriteLine("Fit {0}",(int)fit_combobox.Tag);

                                //Console.WriteLine("Fit {0}", fit_combobox.SelectedValue);

                                //if (size_combobox.SelectedIndex != -1 && fit_combobox.SelectedIndex != -1)
                                //{
                                //    int sf_index = -1;
                                //    sf_index = (int)fit_combobox.SelectedValue + (int)size_combobox.SelectedIndex;

                                //    double chest_diff = -99;
                                //    double waist_diff = -99;
                                //    double bottom_diff = -99;
                                //    double sleeve_diff = -99;

                                //    //chest_diff = 

                                //    // Chest
                                //    chest_diff = (garment[sf_index,0] - chest_size);
                                //    chest_fit = fitDecider(chest_diff, gbdifference[sf_index,0]);

                                //    // Waist
                                //    waist_diff = (garment[sf_index, 1] - waist_size);
                                //    waist_fit = fitDecider(chest_diff, gbdifference[sf_index, 1]);

                                //    // Bottom
                                //    bottom_diff = (garment[sf_index, 2] - bottom_size);
                                //    bottom_fit = fitDecider(bottom_diff, gbdifference[sf_index, 2]);

                                //    // Display

                                //    setColorCodedImage();

                                //}





                            }
                        }
                    }

                }


            }

        }

        // Functions 

        //CSV

        // Convert array data into CSV format.
        private string ArrayToCsv(ushort[,] values)
        {
            // Get the bounds.
            int num_rows = values.GetUpperBound(0) + 1;
            int num_cols = values.GetUpperBound(1) + 1;

            // Convert the array into a CSV string.
            StringBuilder sb = new StringBuilder();
            for (int row = 0; row < num_rows; row++)
            {
                // Add the first field in this row.
                sb.Append(values[row, 0]);

                // Add the other fields in this row separated by commas.
                for (int col = 1; col < num_cols; col++)
                    sb.Append("," + values[row, col]);

                // Move to the next line.
                sb.AppendLine();
            }

            // Return the CSV format string.
            return sb.ToString();
        }

        public double fvlength(CameraSpacePoint center_cp, ushort[,] deptharray, char bp)
        {

            Console.WriteLine("Center : {0} {1} {2}", center_cp.X, center_cp.Y, center_cp.Z);

            DepthSpacePoint center = new DepthSpacePoint();

            center = _sensor.CoordinateMapper.MapCameraPointToDepthSpace(center_cp);

            //
            CameraSpacePoint RightPoint_cp = new CameraSpacePoint();
            CameraSpacePoint LeftPoint_cp = new CameraSpacePoint();

            DepthSpacePoint RightPoint_dp = new DepthSpacePoint();
            DepthSpacePoint LeftPoint_dp = new DepthSpacePoint();

            int right_flag = 0;
            int left_flag = 0;

            int r = (int)center.Y;

            //Console.WriteLine("D Cnter {0} {1}", center.Y, center.X);
            //Console.WriteLine("C Center {0} {1} {2}", center_cp.X, center_cp.Y, center_cp.Z);

            for (int x = 0; x < 510 && right_flag == 0; x++)
            {
                if (left_flag == 1)
                {
                    if (deptharray[r, x] == 0)
                    {
                        if (deptharray[r, x - 1] == 0)
                        {
                            if (deptharray[r, x - 2] == 0)
                            {
                                RightPoint_dp.X = x - 3;
                                RightPoint_dp.Y = r;
                                RightPoint_cp = _sensor.CoordinateMapper.MapDepthPointToCameraSpace(RightPoint_dp, (ushort)(center_cp.Z * 1000));
                                right_flag = 1;
                            }
                        }

                    }
                }
                if (left_flag != 1)
                {
                    if (deptharray[r, x] != 0)
                    {
                        if (deptharray[r, x + 1] != 0)
                        {
                            if (deptharray[r, x + 2] != 0)
                            {
                                LeftPoint_dp.X = x;
                                LeftPoint_dp.Y = r;
                                LeftPoint_cp = _sensor.CoordinateMapper.MapDepthPointToCameraSpace(LeftPoint_dp, (ushort)(center_cp.Z * 1000));
                                left_flag = 1;
                            }
                        }

                    }
                }
            }

            if (bp == 'w')
            {
                waistRightPoint = RightPoint_dp;
                waistLeftPoint = LeftPoint_dp;
            }
            else if (bp == 'c')
            {
                chestRightPoint = RightPoint_dp;
                chestLeftPoint = LeftPoint_dp;
            }
            else if (bp == 'b')
            {
                bottomRightPoint = RightPoint_dp;
                bottomLeftPoint = LeftPoint_dp;
            }

            //Console.WriteLine("Depth : {0}", deptharray[r, (int)center.X]);

            //Console.WriteLine("D Left {0} {1}", LeftPoint_dp.Y, LeftPoint_dp.X);
            //Console.WriteLine("D Right {0} {1}", RightPoint_dp.Y, RightPoint_dp.X);


            //Console.WriteLine("C Left {0} {1} {2}", LeftPoint_cp.X, LeftPoint_cp.Y, LeftPoint_cp.Z);
            //Console.WriteLine("C Right {0} {1} {2}", RightPoint_cp.X, RightPoint_cp.Y, RightPoint_cp.Z);


            //


            //CameraSpacePoint RightPoint_cp = new CameraSpacePoint();
            //CameraSpacePoint LeftPoint_cp = new CameraSpacePoint();

            //DepthSpacePoint RightPoint_dp = new DepthSpacePoint();
            //DepthSpacePoint LeftPoint_dp = new DepthSpacePoint();


            //int r = 0, c_l = 0, c_r = 0;

            //int right_flag = 0;
            //int left_flag = 0;

            //r = (int)center.Y;

            //c_r = (int)center.X;
            //c_l = (int)center.X;

            //while (right_flag != 1 || left_flag != 1)
            //{


            //    if (right_flag != 1)
            //    {

            //        if (deptharray[r, c_r] == 0)
            //        {
            //            if (deptharray[r, c_r - 1] == 0)
            //            {
            //                if (deptharray[r, c_r - 2] == 0)
            //                {
            //                    RightPoint_dp.X = c_r - 3;
            //                    RightPoint_dp.Y = r;
            //                    right_flag = 1;

            //                    //RightPoint_cp = _sensor.CoordinateMapper.MapDepthPointToCameraSpace(RightPoint_dp, deptharray[r, c_r - 3]);
            //                    RightPoint_cp = _sensor.CoordinateMapper.MapDepthPointToCameraSpace(RightPoint_dp, deptharray[r, (int)center.X]);
            //                }
            //            }
            //        }
            //        c_r++;
            //    }

            //    if (left_flag != 1)
            //    {

            //        if (deptharray[r, c_l] == 0)
            //        {
            //            if (deptharray[r, c_l + 1] == 0)
            //            {
            //                if (deptharray[r, c_l + 2] == 0)
            //                {
            //                    LeftPoint_dp.X = c_l;
            //                    LeftPoint_dp.Y = r;
            //                    left_flag = 1;

            //                    //LeftPoint_cp = _sensor.CoordinateMapper.MapDepthPointToCameraSpace(LeftPoint_dp, deptharray[r, c_l + 3]);
            //                    LeftPoint_cp = _sensor.CoordinateMapper.MapDepthPointToCameraSpace(LeftPoint_dp, deptharray[r, (int)center.X]);
            //                }
            //            }
            //        }
            //        c_l--;
            //    }
            //}

            double CalculatedWidth = 0;
            CalculatedWidth = Distance(RightPoint_cp, LeftPoint_cp);

            //CalculatedWidth = Math.Sqrt(Math.Pow((RightPoint_cp.X - LeftPoint_cp.X), 2));

            return CalculatedWidth;
        }

        public ushort[,] depthFrameToPixelsData(DepthFrame depth_Frame, BodyIndexFrame bodyIndex_Frame)
        {
            ushort[,] darray = new ushort[424, 512];


            if (depth_Frame != null && bodyIndex_Frame != null)
            {

                int width = depth_Frame.FrameDescription.Width;
                int height = depth_Frame.FrameDescription.Height;

                byte[] _bodyData = new byte[width * height];

                bodyIndex_Frame.CopyFrameDataToArray(_bodyData);

                //Console.WriteLine("Width" + width);
                //Console.WriteLine("Height" + height);

                PixelFormat format = PixelFormats.Bgr32;
                stride = width * ((format.BitsPerPixel + 7) / 8);

                ushort minDepth = depth_Frame.DepthMinReliableDistance;
                ushort maxDepth = depth_Frame.DepthMaxReliableDistance;

                ushort[] pixelData = new ushort[width * height];
                byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];

                depth_Frame.CopyFrameDataToArray(pixelData);

                int j = 0, k = 0;

                for (int i = 0; i < (424 * 512) && j < 424; i++)
                {

                    if (_bodyData[i] != 0xff)
                    {
                        darray[j, k] = pixelData[i];
                    }
                    else
                    {
                        darray[j, k] = 0;
                    }

                    k++;

                    if (k % 512 == 0)
                    {
                        k = 0;
                        j++;
                    }


                }

            }


            return darray;

        }

        public double Distance(CameraSpacePoint point1, CameraSpacePoint point2)
        {
            double dist = 0;
            
            //dist = Math.Sqrt(
            //    Math.Pow((point1.X - point2.X), 2) +
            //    Math.Pow((point1.Y - point2.Y), 2) +
            //    Math.Pow((point1.Z - point2.Z), 2)
            //);

            dist = Math.Sqrt(Math.Pow((point1.X - point2.X), 2));

            return dist;
        }

        public double eperimetercalculator(double a, double b)
        {
            a = a / 2.0;
            b = b / 2.0;
            double perimeter = 0;
            perimeter = Math.PI * (3 * (a + b) - Math.Sqrt(((3 * a) + b) * (a + (3 * b))));
            return perimeter;
        }

        public ushort[,] greenscreentorgb(WriteableBitmap bitmap, char color)
        {
            byte[] rectarr = new byte[512 * 424 * 4];

            ushort[,] bg_r = new ushort[424, 512];
            bitmap.CopyPixels(rectarr, stride, 0);

            ushort[,] barray = new ushort[424, 512];
            ushort[,] garray = new ushort[424, 512];
            ushort[,] rarray = new ushort[424, 512];
            ushort[] pixelData = new ushort[512 * 424 * 4];
            byte[] pixels = new byte[512 * 424 * 4];

            int j = 0, k = 0;

            for (int i = 0; i < ((424 * 512 * 4) - 3) && j < 424; i = i + 4)
            {
                barray[j, k] = pixelData[i];
                garray[j, k] = pixelData[i + 1];
                rarray[j, k] = pixelData[i + 2];

                k++;

                if (k % 512 == 0)
                {
                    k = 0;
                    j++;
                }
            }

            if (color == 'r')
            {
                return rarray;
            }
            if (color == 'g')
            {
                return rarray;
            }
            if (color == 'b')
            {
                return rarray;
            }
            return rarray;
        }

        public BitmapSource GreenScreenCustom(ColorFrame colorFrame, DepthFrame depthFrame, BodyIndexFrame bodyIndexFrame)
        {

            //

            ushort[] _depthData = null;
            byte[] _bodyData = null;
            byte[] _colorData = null;
            byte[] _displayPixels = null;
            ColorSpacePoint[] _colorPoints = null;
            CoordinateMapper _coordinateMapper = _sensor.CoordinateMapper;

            int BYTES_PER_PIXEL = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
            PixelFormat FORMAT = PixelFormats.Bgra32;
            double DPI = 96.0;

            WriteableBitmap _bitmap = null;



            //

            int colorWidth = colorFrame.FrameDescription.Width;
            int colorHeight = colorFrame.FrameDescription.Height;

            int depthWidth = depthFrame.FrameDescription.Width;
            int depthHeight = depthFrame.FrameDescription.Height;

            int bodyIndexWidth = bodyIndexFrame.FrameDescription.Width;
            int bodyIndexHeight = bodyIndexFrame.FrameDescription.Height;

            if (_displayPixels == null)
            {
                _depthData = new ushort[depthWidth * depthHeight];
                _bodyData = new byte[depthWidth * depthHeight];
                _colorData = new byte[colorWidth * colorHeight * BYTES_PER_PIXEL];
                _displayPixels = new byte[depthWidth * depthHeight * BYTES_PER_PIXEL];
                _colorPoints = new ColorSpacePoint[depthWidth * depthHeight];
                _bitmap = new WriteableBitmap(depthWidth, depthHeight, DPI, DPI, FORMAT, null);
            }

            if (((depthWidth * depthHeight) == _depthData.Length) && ((colorWidth * colorHeight * BYTES_PER_PIXEL) == _colorData.Length) && ((bodyIndexWidth * bodyIndexHeight) == _bodyData.Length))
            {
                depthFrame.CopyFrameDataToArray(_depthData);

                if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    colorFrame.CopyRawFrameDataToArray(_colorData);
                }
                else
                {
                    colorFrame.CopyConvertedFrameDataToArray(_colorData, ColorImageFormat.Bgra);
                }

                bodyIndexFrame.CopyFrameDataToArray(_bodyData);

                _coordinateMapper.MapDepthFrameToColorSpace(_depthData, _colorPoints);

                Array.Clear(_displayPixels, 0, _displayPixels.Length);
                Array.Clear(rarray_final, 0, rarray_final.Length);
                Array.Clear(garray_final, 0, rarray_final.Length);
                Array.Clear(barray_final, 0, rarray_final.Length);


                for (int y = 0; y < depthHeight; ++y)
                {
                    for (int x = 0; x < depthWidth; ++x)
                    {
                        int depthIndex = (y * depthWidth) + x;

                        byte player = _bodyData[depthIndex];

                        if (player != 0xff)
                        {
                            ColorSpacePoint colorPoint = _colorPoints[depthIndex];

                            int colorX = (int)Math.Floor(colorPoint.X + 0.5);
                            int colorY = (int)Math.Floor(colorPoint.Y + 0.5);

                            if ((colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight))
                            {
                                int colorIndex = ((colorY * colorWidth) + colorX) * BYTES_PER_PIXEL;
                                int displayIndex = depthIndex * BYTES_PER_PIXEL;

                                _displayPixels[displayIndex + 0] = _colorData[colorIndex];
                                _displayPixels[displayIndex + 1] = _colorData[colorIndex + 1];
                                _displayPixels[displayIndex + 2] = _colorData[colorIndex + 2];
                                _displayPixels[displayIndex + 3] = 0xff;

                                barray_final[y, x] = _colorData[colorIndex];
                                garray_final[y, x] = _colorData[colorIndex + 1];
                                rarray_final[y, x] = _colorData[colorIndex + 2];

                                // For ROS


                                _finaldisplayPixels[displayIndex + 0] = _colorData[colorIndex + 2];
                                _finaldisplayPixels[displayIndex + 1] = _colorData[colorIndex + 1];
                                _finaldisplayPixels[displayIndex + 2] = _colorData[colorIndex];
                                _finaldisplayPixels[displayIndex + 3] = 0xff;
                            }
                        }
                    }
                }

                _bitmap.Lock();

                Marshal.Copy(_displayPixels, 0, _bitmap.BackBuffer, _displayPixels.Length);
                _bitmap.AddDirtyRect(new Int32Rect(0, 0, depthWidth, depthHeight));

                _bitmap.Unlock();
            }

            return _bitmap;
        }

        public void setColorCodedImage()
        {
            int r = 0;
            int g = 0;
            int b = 0;
            int fit = 0;


            byte[] _displayPixels = null;

            // Chest

            int[] chest_color = fit2color(chest_fit);
            int[] waist_color = fit2color(waist_fit);
            int[] bottom_color = fit2color(bottom_fit);


            pixelsUpdater(chestRightPoint, chest_color);
            pixelsUpdater(waistRightPoint, waist_color);
            pixelsUpdater(bottomRightPoint, bottom_color);

            generateColorCodedImage();

        }

        public int[] fit2color(int fit)
        {
            int r = 0;
            int g = 0;
            int b = 0;

            if (fit == 1)
            {
                r = 75;
                g = 0;
                b = 130;
            }
            else if (fit == 2)
            {
                r = 64;
                g = 224;
                b = 208;
            }
            else if (fit == 3)
            {
                r = 0;
                g = 128;
                b = 0;
            }
            else if (fit == 4)
            {
                r = 255;
                g = 105;
                b = 0;
            }
            else if (fit == 5)
            {
                r = 202;
                g = 0;
                b = 42;
            }

            int[] rgbarr = new int[3]
            {r,
            g,
            b
            };

            return rgbarr;
        }


        private void Fit_combobox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (size_combobox.SelectedIndex != -1 && fit_combobox.SelectedIndex != -1)
            {
                int sf_index = -1;
                sf_index = int.Parse(fit_combobox.SelectedValue.ToString()) + (int)size_combobox.SelectedIndex;

                double chest_diff = -99;
                double waist_diff = -99;
                double bottom_diff = -99;
                double sleeve_diff = -99;

                //chest_diff = 

                // Chest
                chest_diff = (garment[sf_index, 0] - (chest_size * 100));
                chest_fit = fitDecider(chest_diff, gbdifference[sf_index, 0]);

                // Waist
                waist_diff = (garment[sf_index, 1] - (waist_size * 100));
                waist_fit = fitDecider(waist_diff, gbdifference[sf_index, 1]);

                // Bottom
                bottom_diff = (garment[sf_index, 2] - (bottom_size * 100));
                bottom_fit = fitDecider(bottom_diff, gbdifference[sf_index, 2]);

                // Display

                setColorCodedImage();

                // Publish Images to ROS
                rosImagesMessager();
                rosFitsMesssager();
            }
        }

        public void pixelsUpdater(DepthSpacePoint right, int[] color)
        {
            int write_flag = 0;

            for (int i = ((int)right.Y - 3); i < ((int)right.Y + 4); i++)
            {
                for (int t = 3; t <= 509; t++)
                {
                    // Left Point Detected
                    if (rarray_final[i, t] != 0)
                    {
                        if (rarray_final[i, t + 1] != 0)
                        {
                            if (rarray_final[i, t + 2] != 0)
                            {
                                write_flag = 1;
                            }
                        }

                    }

                    // Right Point Detected
                    if (rarray_final[i, t] == 0)
                    {
                        if (rarray_final[i, t + 1] == 0)
                        {
                            if (rarray_final[i, t + 2] == 0)
                            {
                                write_flag = 0;
                            }
                        }

                    }


                    if (write_flag == 1)
                    {
                        rarray_final[i, t] = (ushort)color[0];
                        garray_final[i, t] = (ushort)color[1];
                        barray_final[i, t] = (ushort)color[2];
                    }

                }
            }
        }

        public void generateColorCodedImage()
        {
            byte[] _displayPixels = new byte[512 * 424 * 4];

            int displayIndex = 0;

            for (int y = 0; y < 424; y++)
            {
                for (int x = 0; x < 512; x++)
                {
                    _displayPixels[displayIndex] = (byte)barray_final[y, x];
                    _displayPixels[displayIndex + 1] = (byte)garray_final[y, x];
                    _displayPixels[displayIndex + 2] = (byte)rarray_final[y, x];
                    _displayPixels[displayIndex + 3] = 0xff;

                    _finaldisplayPixels[displayIndex] = (byte)rarray_final[y, x];
                    _finaldisplayPixels[displayIndex + 1] = (byte)garray_final[y, x];
                    _finaldisplayPixels[displayIndex + 2] = (byte)barray_final[y, x];
                    _finaldisplayPixels[displayIndex + 3] = 0xff;

                    displayIndex += 4;
                }
            }


            WriteableBitmap _bitmap = new WriteableBitmap(512, 424, 96, 96, PixelFormats.Bgra32, null);

            _bitmap.Lock();

            Marshal.Copy(_displayPixels, 0, _bitmap.BackBuffer, _displayPixels.Length);
            _bitmap.AddDirtyRect(new Int32Rect(0, 0, 512, 424));

            _bitmap.Unlock();

            if (unityui_flag == 0)
            {
                FrontFrameImageDisplay.Source = _bitmap;
            }
        }

        public int fitDecider(double acutalDIff, double goodDiff)
        {
            int fit = 0;

            if ((goodDiff - acutalDIff) <= 3.5 && (goodDiff - acutalDIff) >= -3.5)
            {
                fit = 3;
            }
            else if ((goodDiff - acutalDIff) < -2.5 && (goodDiff - acutalDIff) >= -7.5)
            {
                fit = 2;
            }
            else if ((goodDiff - acutalDIff) < -7.5)
            {
                fit = 1;
            }
            else if ((goodDiff - acutalDIff) <= 7.5 && (goodDiff - acutalDIff) >= 2.5)
            {
                fit = 4;
            }
            else if ((goodDiff - acutalDIff) > 7.5)
            {
                fit = 5;
            }

            return fit;
        }

        public void rosMeasurementsMessager()
        {
            bm_msgs.bm_message measurements_message = new bm_msgs.bm_message
            {
                chest_size_msg = Math.Round((chest_size * 100), 2),
                waist_size_msg = Math.Round((waist_size * 100), 2),
                bottom_size_msg = Math.Round((bottom_size * 100),2),
                shoulder_size_msg = Math.Round((shoulder_front * 100),2),
                sleeve_size_msg = Math.Round((sleeve_front * 100),2)
            };
            rosSocket.Publish(rosBMMeasurements_Id, measurements_message);
        }

        public void rosFitsMesssager()
        {
            bm_msgs.bm_fits fits_message = new bm_msgs.bm_fits
            {
                chest_fit_msg = chest_fit,
                waist_fit_msg = waist_fit,
                bottom_fit_msg = bottom_fit
            };
            rosSocket.Publish(rosBMFits_Id, fits_message);
        }

        public void SizeFitSubscriptionHandler(bm_msgs.bm_sizefit sizefit_message)
        {
            unityui_flag = 1;
            size = sizefit_message.size_msg;
            fit = sizefit_message.fit_msg;

            Console.WriteLine("Size : {0}", size);
            Console.WriteLine("Fit : {0}", fit);

            //size_combobox.SelectedIndex = size;

            //fit_combobox.SelectedItem = sizefit_message.fit_msg;

            //
            int sf_index = size + fit;

            double chest_diff = -99;
            double waist_diff = -99;
            double bottom_diff = -99;
            double sleeve_diff = -99;

            //chest_diff = 

            // Chest
            chest_diff = (garment[sf_index, 0] - (chest_size * 100));
            chest_fit = fitDecider(chest_diff, gbdifference[sf_index, 0]);

            // Waist
            waist_diff = (garment[sf_index, 1] - (waist_size * 100));
            waist_fit = fitDecider(waist_diff, gbdifference[sf_index, 1]);

            // Bottom
            bottom_diff = (garment[sf_index, 2] - (bottom_size * 100));
            bottom_fit = fitDecider(bottom_diff, gbdifference[sf_index, 2]);

            // Display
            Console.WriteLine("------------------------{0} {1} {2}", chest_fit,waist_fit, bottom_fit);
            setColorCodedImage();

            // Publish Images to ROS
            rosImagesMessager();
            rosFitsMesssager();

            //
            unityui_flag = 0;
        }

        public void rosImagesMessager()
        {
            bm_imsgs.Image image_message = new bm_imsgs.Image
            {
                height = 424,
                width = 512,
                //encoding = "bgra32",
                is_bigendian = 0,
                step = 2048,
                data = _finaldisplayPixels
            };
            rosSocket.Publish(rosBMImage_Id, image_message);
        }


    }
}
