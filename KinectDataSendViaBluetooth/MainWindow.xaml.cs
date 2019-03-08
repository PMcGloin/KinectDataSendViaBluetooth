﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Threading;
using InTheHand.Net.Bluetooth;
using InTheHand.Net.Sockets;
using System.IO;
using Microsoft.Kinect;
using LightBuzz.Vitruvius;

namespace KinectDataSendViaBluetooth
{
    public static class JointOrientationExtensions
    {
        /// <summary>
        /// Rotates the specified quaternion around the X axis.
        /// </summary>
        /// <param name="quaternion">The orientation quaternion.</param>
        /// <returns>The rotation in degrees.</returns>
        public static double Pitch(this Vector4 quaternion)
        {
            double value1 = 2.0 * (quaternion.W * quaternion.X + quaternion.Y * quaternion.Z);
            double value2 = 1.0 - 2.0 * (quaternion.X * quaternion.X + quaternion.Y * quaternion.Y);

            double roll = Math.Atan2(value1, value2);

            return roll * (180.0 / Math.PI);
        }

        /// <summary>
        /// Rotates the specified quaternion around the Y axis.
        /// </summary>
        /// <param name="quaternion">The orientation quaternion.</param>
        /// <returns>The rotation in degrees.</returns>
        public static double Yaw(this Vector4 quaternion)
        {
            double value = 2.0 * (quaternion.W * quaternion.Y - quaternion.Z * quaternion.X);
            value = value > 1.0 ? 1.0 : value;
            value = value < -1.0 ? -1.0 : value;

            double pitch = Math.Asin(value);

            return pitch * (180.0 / Math.PI);
        }

        /// <summary>
        /// Rotates the specified quaternion around the Z axis.
        /// </summary>
        /// <param name="quaternion">The orientation quaternion.</param>
        /// <returns>The rotation in degrees.</returns>
        public static double Roll(this Vector4 quaternion)
        {
            double value1 = 2.0 * (quaternion.W * quaternion.Z + quaternion.X * quaternion.Y);
            double value2 = 1.0 - 2.0 * (quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z);

            double yaw = Math.Atan2(value1, value2);

            return yaw * (180.0 / Math.PI);
        }
    }
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        List<string> bluetoothDevices;
        // index for the currently tracked body
        private int bodyIndex;
        // flag to asses if a body is currently tracked
        private bool bodyTracked = false;
        public MainWindow()
        {
            bluetoothDevices = new List<string>();
            InitializeComponent();
        }
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

        }
        private void Window_Closed(object sender, EventArgs e)
        {
            if (bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }
            if (kinectSensor != null)
            {
                kinectSensor.Close();
                kinectSensor = null;
            }
        }
        private void GoButton_Click(object sender, RoutedEventArgs e)
        {
            if (connectionRunning == true)
            {
                UpdateUI("Connection Running");
                return;
            }
            if (ReceiveRadioButton.IsChecked == true)
            {
                ConnectAsReceiver();
            }
            else
            {
                StartScan();
            }
        }
        private void StartScan()
        {
            //DevicesListBox.DataSource = null;
            DevicesListBox.Items.Clear();
            bluetoothDevices.Clear();
            Thread bluetoothScanThread = new Thread(new ThreadStart(Scan));
            bluetoothScanThread.Start();
        }
        BluetoothDeviceInfo[] bluetoothDeviceInfo;
        private void Scan()
        {
            UpdateUI("Starting Scan ...");
            BluetoothClient bluetoothClient = new BluetoothClient();
            bluetoothDeviceInfo = bluetoothClient.DiscoverDevicesInRange();
            UpdateUI("Scan Complete");

            UpdateUI(bluetoothDeviceInfo.Length.ToString() + " Devices Discovered");
            foreach (BluetoothDeviceInfo deviceInfo in bluetoothDeviceInfo)
            {
                bluetoothDevices.Add(deviceInfo.DeviceName);
            }
            UpdateDevicesList();
        }
        private void ConnectAsReceiver()
        {
            Thread bluetoothReceiverThread = new Thread(new ThreadStart(ReceiverConnectThread));
            bluetoothReceiverThread.Start();
        }
        //Guid mUUID = new Guid("00001101-0000-1000-8000-00805F9B34FB"); replaced by BluetoothService.SerialPort
        bool connectionRunning = false;
        public void ReceiverConnectThread()
        {
            connectionRunning = true;
            UpdateUI("Started ... Waiting for Connection ...");
            BluetoothListener bluetoothListener = new BluetoothListener(BluetoothService.SerialPort);
            bluetoothListener.Start();
            BluetoothClient bluetoothConnection = bluetoothListener.AcceptBluetoothClient();
            UpdateUI("Device Connected");
            Stream mStream = bluetoothConnection.GetStream();
            while (true)
            {
                try
                {
                    //handle receiver connection
                    byte[] receivedData = new byte[1024];
                    mStream.Read(receivedData, 0, receivedData.Length);
                    UpdateUI("Data Received: " + Encoding.ASCII.GetString(receivedData));
                }
                catch (IOException exception)
                {
                    UpdateUI("Bluetooth Disconnected");
                }
            }
        }
        private void UpdateUI(string message)
        {
            Dispatcher.Invoke(() =>
            {
                InfoTextBox.AppendText(message + Environment.NewLine);
                InfoTextBox.ScrollToEnd();
            });
            
        }
        private void UpdateDevicesList()
        {
            Dispatcher.Invoke(() =>
            {
                foreach(string device in bluetoothDevices)
                {
                    DevicesListBox.Items.Add(device);
                }
            });
        }
        BluetoothDeviceInfo deviceInfo;
        private void DevicesListBox_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            deviceInfo = bluetoothDeviceInfo.ElementAt(DevicesListBox.SelectedIndex);
            UpdateUI(deviceInfo.DeviceName + " was selected ... atempting to connect");
            if (PairDevice())
            {
                UpdateUI("Device Paired ... Starting Connection Thread");
                Thread bluetoothSenderThread = new Thread(new ThreadStart(SenderConnectThread));
                bluetoothSenderThread.Start();
            }
            else
            {
                UpdateUI("Pair Failed");
            }
        }
        private void SenderConnectThread()
        {
            BluetoothClient bluetoothClient = new BluetoothClient();
            UpdateUI("Atempting to Connect");
            bluetoothClient.BeginConnect(deviceInfo.DeviceAddress, BluetoothService.SerialPort, new AsyncCallback(BluetoothSenderConnectCallback), bluetoothClient);
        }
        KinectSensor kinectSensor;
        BodyFrameReader bodyFrameReader;
        Body[] bodies;
        byte[] sendAngles;
        string sendAnglesString;

        private void BluetoothSenderConnectCallback(IAsyncResult asyncResult)
        {
            BluetoothClient bluetoothClient = (BluetoothClient)asyncResult.AsyncState;
            if (bluetoothClient.Connected == true)
            {
                UpdateUI("Bluetooth Connected ... Starting Kinect");
                kinectSensor = KinectSensor.GetDefault();
                bodyFrameReader = kinectSensor.BodyFrameSource.OpenReader();
                kinectSensor.Open();
                if (bodyFrameReader != null)
                {
                    bodyFrameReader.FrameArrived += Reader_FrameArrived;
                }
            }
            else
            {
                UpdateUI("Not Connected");
            }
            bluetoothClient.EndConnect(asyncResult);
            Stream bluetoothStream = bluetoothClient.GetStream();
            bluetoothStream.ReadTimeout = 1000;
            while (true)
            {
                while (!ready) ;
                bluetoothStream.Write(sendAngles, 0, sendAngles.Length);
                sendAnglesString = Encoding.ASCII.GetString(sendAngles);
                UpdateUI("Data sent: " + sendAnglesString);

                ready = false;
            }
        }
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            UpdateUI("Frame arrived");
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (bodies == null)
                    {
                        bodies = new Body[bodyFrame.BodyCount];
                        UpdateUI("Number of body " + bodyFrame.BodyCount);
                    }
                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(bodies);
                    dataReceived = true;
                }
            }
            if (dataReceived)
            {
                UpdateUI("Data Recieved");
                Body body = null;
                if (bodyTracked)
                {
                    if (bodies[bodyIndex].IsTracked)
                    {
                        body = bodies[bodyIndex];
                    }
                    else
                    {
                        bodyTracked = false;
                    }
                }
                if (!bodyTracked)
                {
                    for (int i = 0; i < bodies.Length; ++i)
                    {
                        if (bodies[i].IsTracked)
                        {
                            bodyIndex = i;
                            bodyTracked = true;
                            break;
                        }
                    }
                }
                //single currently tracked body
                if (body != null && bodyTracked && body.IsTracked)
                {
                    var shoulderOrintation = body.JointOrientations[JointType.ShoulderRight].Orientation;
                    var shoulderRotationX = (int)shoulderOrintation.Pitch();
                    var shoulderRotationY = shoulderOrintation.Yaw();
                    var shoulderRotationZ = shoulderOrintation.Roll();
                    //sendAngles = Encoding.ASCII.GetBytes(rotationX.ToString());
                    

                    var wristOrintation = body.JointOrientations[JointType.WristRight].Orientation;
                    var wristRotationX = (int)wristOrintation.Pitch();
                    var wristRotationY = wristOrintation.Yaw();
                    var wristRotationZ = wristOrintation.Roll();
                    

                    Joint spine = body.Joints[JointType.SpineShoulder];
                    Joint shoulder = body.Joints[JointType.ShoulderRight];
                    Joint elbow = body.Joints[JointType.ElbowRight];
                    Joint wrist = body.Joints[JointType.WristRight];
                    Joint hand = body.Joints[JointType.HandTipRight];
                    
                    int elbowAngle = (int)elbow.Angle(shoulder, wrist);
                    int shoulderAngle = (int)shoulder.Angle(spine, elbow);
                    int wristAngle = (int)wrist.Angle(elbow, hand);
                    
                    UpdateUI("Shoulder Rotation " + shoulderRotationX.ToString());
                    UpdateUI("Shoulder angle: " + shoulderAngle.ToString());
                    UpdateUI("Elbow angle: " + elbowAngle.ToString());
                    UpdateUI("Wrist angle: " + wristAngle.ToString());
                    UpdateUI("Wrist Rotation " + wristRotationX.ToString());
                    UpdateUI("Hand State " + body.HandRightState.ToString());

                    sendAngles = Encoding.ASCII.GetBytes(shoulderRotationX.ToString() + shoulderAngle.ToString() + 
                        elbowAngle.ToString() + wristAngle.ToString() + wristRotationX.ToString() + 
                        body.HandRightState.ToString());
                    
                    //test code
                    //sendAngles = Encoding.ASCII.GetBytes(body.HandRightState.ToString());

                    ready = true;
                }
            }
        }
        string myPIN = "1234";
        private bool PairDevice()
        {
            if (!deviceInfo.Authenticated)
            {
                if (!BluetoothSecurity.PairRequest(deviceInfo.DeviceAddress, myPIN))
                {
                    return false;
                }
            }
            return true;
        }
        bool ready = false;
        byte[] message;
        private void SendTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                message = Encoding.ASCII.GetBytes(SendTextBox.Text);
                ready = true;
                SendTextBox.Clear();
            }
        }
    }
}