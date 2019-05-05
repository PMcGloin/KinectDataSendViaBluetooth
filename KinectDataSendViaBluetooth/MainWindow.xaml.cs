using System;
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
namespace KinectDataSendViaBluetooth{
    public static class JointOrientationExtensions{
        /// <summary>
        /// Rotates the specified quaternion around the X axis.
        /// </summary>
        /// <param name="quaternion">The orientation quaternion.</param>
        /// <returns>The rotation in degrees.</returns>
        public static double Pitch(this Vector4 quaternion){
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
        public static double Yaw(this Vector4 quaternion){
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
        public static double Roll(this Vector4 quaternion){
            double value1 = 2.0 * (quaternion.W * quaternion.Z + quaternion.X * quaternion.Y);
            double value2 = 1.0 - 2.0 * (quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z);
            double yaw = Math.Atan2(value1, value2);
            return yaw * (180.0 / Math.PI);
        }
    }
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window{
        List<string> bluetoothDevices;
        // index for the currently tracked body
        private int bodyIndex;
        // flag to asses if a body is currently tracked
        private bool bodyTracked = false;
        bool ready = false;
        bool closeConnection = false;
        public MainWindow(){
            bluetoothDevices = new List<string>();
            InitializeComponent();
        }
        /// <summary>
        /// On window load start bluetooth scan
        /// </summary>
        private void Window_Loaded(object sender, RoutedEventArgs e){
            StartScan();
        }
        /// <summary>
        /// On window close dispose of kinect frames and close the sensor
        /// </summary>
        private void Window_Closed(object sender, EventArgs e){
            if (bodyFrameReader != null){
                // BodyFrameReader is IDisposable
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }
            if (kinectSensor != null){
                kinectSensor.Close();
                kinectSensor = null;
            }
            closeConnection = true;
        }
        /// <summary>
        /// StartScan method
        /// </summary>
        private void StartScan(){
            ClearListBox();
            bluetoothDevices.Clear();
            Thread bluetoothScanThread = new Thread(new ThreadStart(Scan));
            bluetoothScanThread.Start();
        }
        BluetoothDeviceInfo[] bluetoothDeviceInfo;
        /// <summary>
        /// Scan method
        /// </summary>
        private void Scan(){
            UpdateUI("Starting Scan ...");
            BluetoothClient bluetoothClient = new BluetoothClient();
            bluetoothDeviceInfo = bluetoothClient.DiscoverDevicesInRange();
            UpdateUI("Scan Complete");
            UpdateUI(bluetoothDeviceInfo.Length.ToString() + " Devices Discovered");
            if (bluetoothDeviceInfo.Length == 0){
                UpdateUI("Scan Complete ... No devices found ... Ensure devices are powered on and discoverable ... Restarting scan");
                StartScan();
            }
            else{
                foreach (BluetoothDeviceInfo deviceInfo in bluetoothDeviceInfo){
                    bluetoothDevices.Add(deviceInfo.DeviceName);
                }
                UpdateDevicesList();
            }
        }
        /// <summary>
        /// Acessing data on different Threads
        /// </summary>
        private void UpdateUI(string message){
            Dispatcher.Invoke(() => {
                InfoTextBox.AppendText(message + Environment.NewLine);
                InfoTextBox.ScrollToEnd();
            });
        }
        private void UpdateKinectDataTextBox(string message, int charValue){
            Dispatcher.Invoke(() => {
                KinectDataTextBox.AppendText(message + charValue + Environment.NewLine);
                KinectDataTextBox.ScrollToEnd();
            });
        }
        private void ClearListBox(){
            Dispatcher.Invoke(() =>{
                DevicesListBox.Items.Clear();
            });
        }
        /// <summary>
        /// Writes discovered divices to listbox in window
        /// </summary>
        private void UpdateDevicesList(){
            Dispatcher.Invoke(() =>{
                foreach(string device in bluetoothDevices){
                    DevicesListBox.Items.Add(device);
                }
            });
        }
        BluetoothDeviceInfo deviceInfo;
        /// <summary>
        /// Handles double click of item in devices listbox by creating and starting thread to handle sent data
        /// </summary>
        private void DevicesListBox_MouseDoubleClick(object sender, MouseButtonEventArgs e){
            deviceInfo = bluetoothDeviceInfo.ElementAt(DevicesListBox.SelectedIndex);
            UpdateUI(deviceInfo.DeviceName + " was selected ... atempting to connect");
            if (PairDevice()){
                UpdateUI("Device Paired ... Starting Connection Thread");
                Thread bluetoothSenderThread = new Thread(new ThreadStart(SenderConnectThread));
                bluetoothSenderThread.Start();
            }
            else{
                UpdateUI("Pair Failed");
            }
        }
        string myPIN = "1234";
        /// <summary>
        /// Ensures bluetooth device is paired
        /// </summary>
        private bool PairDevice(){
            if (!deviceInfo.Authenticated){
                if (!BluetoothSecurity.PairRequest(deviceInfo.DeviceAddress, myPIN)){
                    return false;
                }
            }
            return true;
        }
        /// <summary>
        /// Creates Bluetooth connection for sending information
        /// </summary>
        private void SenderConnectThread(){
            BluetoothClient bluetoothClient = new BluetoothClient();
            UpdateUI("Atempting to Connect");
            bluetoothClient.BeginConnect(deviceInfo.DeviceAddress, BluetoothService.SerialPort, new AsyncCallback(BluetoothSenderConnectCallback), bluetoothClient);
        }
        KinectSensor kinectSensor;
        BodyFrameReader bodyFrameReader;
        Body[] bodies;
        byte[] sendAngles;
        string sendAnglesString;
        char handState, prevHandState = (char)10;
        /// <summary>
        /// When a send connection connects starts kinect sensor then handles data to be sent
        /// </summary>
        private void BluetoothSenderConnectCallback(IAsyncResult asyncResult){
            BluetoothClient bluetoothClient = (BluetoothClient)asyncResult.AsyncState;
            if (bluetoothClient.Connected == true){
                UpdateUI("Bluetooth Connected ... Starting Kinect");
                kinectSensor = KinectSensor.GetDefault();
                bodyFrameReader = kinectSensor.BodyFrameSource.OpenReader();
                kinectSensor.Open();
                if (bodyFrameReader != null){
                    bodyFrameReader.FrameArrived += Reader_FrameArrived;
                }
            }
            else{
                UpdateUI("Not Connected");
            }
            bluetoothClient.EndConnect(asyncResult);
            Stream bluetoothStream = bluetoothClient.GetStream();
            bluetoothStream.ReadTimeout = 1000;
            while (true){
                while (!ready) ;
                bluetoothStream.Write(sendAngles, 0, sendAngles.Length);
                Array.Clear(sendAngles, 0, sendAngles.Length);
                UpdateUI("Data sent: " + sendAnglesString);
                ready = false;
                if (closeConnection == true){
                    bluetoothStream.Dispose();
                }
            }
        }
        /// <summary>
        /// Handles and processes frame data, allowing only one users data (right arm) to be sent
        /// </summary>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e){
            bool dataReceived = false;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame()){
                if (bodyFrame != null){
                    if (bodies == null){
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
            if (dataReceived){
                Body body = null;
                if (bodyTracked){
                    if (bodies[bodyIndex].IsTracked){
                        body = bodies[bodyIndex];
                    }
                    else{
                        bodyTracked = false;
                    }
                }
                if (!bodyTracked){
                    for (int i = 0; i < bodies.Length; ++i){
                        if (bodies[i].IsTracked){
                            bodyIndex = i;
                            bodyTracked = true;
                            break;
                        }
                    }
                }
                if (body != null && bodyTracked && body.IsTracked) {        //single currently tracked body
                    /* Robotic arm servo angles
                    1) Shoulder Rotation, allowed values from 0 to 180 degrees
                    2) Shoulder Angle, allowed values from 15 to 165 degrees
                    3) Elbow Angle, allowed values from 0 to 180 degrees
                    4) Wrist Angle, allowed values from 0 to 180 degrees
                    5) Wrist Rotation, allowed values from 0 to 180 degrees
                    6) Gripper, allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
                    */
                    var shoulderOrintation = body.JointOrientations[JointType.ShoulderRight].Orientation;
                    char shoulderRotationX = (char)shoulderOrintation.Pitch();
                    if (shoulderRotationX < (char)0){           //less than 0
                        shoulderRotationX = (char)0;
                    }
                    else if (shoulderRotationX > (char)180){    //greater than 180
                        shoulderRotationX = (char)180;
                    }

                    char shoulderRotationY = (char)shoulderOrintation.Yaw();
                    if (shoulderRotationY < (char)0){           //less than 0
                        shoulderRotationY = (char)0;
                    }
                    else if (shoulderRotationY > (char)180){    //greater than 180
                        shoulderRotationY = (char)180;
                    }

                    char shoulderRotationZ = (char)shoulderOrintation.Roll();
                    if (shoulderRotationZ < (char)0){            //less than 0
                        shoulderRotationZ = (char)0;
                    }
                    else if (shoulderRotationZ > (char)180){    //greater than 180
                        shoulderRotationZ = (char)180;
                    }
                    var wristOrintation = body.JointOrientations[JointType.WristRight].Orientation;
                    char wristRotationX = (char)wristOrintation.Pitch();
                    if (wristRotationX < (char)0){              //less than 0
                        wristRotationX = (char)0;
                    }
                    else if (wristRotationX > (char)180){       //greater than 180
                        wristRotationX = (char)180;
                    }
                    char wristRotationY = (char)wristOrintation.Yaw();
                    if (wristRotationY < (char)0){              //less than 0
                        wristRotationY = (char)0;
                    }
                    else if (wristRotationY > (char)180){       //greater than 180
                        wristRotationY = (char)180;
                    }
                    char wristRotationZ = (char)wristOrintation.Roll();
                    if (wristRotationZ < (char)0){              //less than 0
                        wristRotationZ = (char)0;
                    }
                    else if (wristRotationZ > (char)180){       //greater than 180
                        wristRotationZ = (char)180;
                    }
                    Joint spine = body.Joints[JointType.SpineShoulder];
                    Joint shoulder = body.Joints[JointType.ShoulderRight];
                    Joint elbow = body.Joints[JointType.ElbowRight];
                    Joint wrist = body.Joints[JointType.WristRight];
                    Joint hand = body.Joints[JointType.HandTipRight];
                    char elbowAngle = (char)(elbow.Angle(shoulder, wrist)-90);
                    if (elbowAngle < (char)0 || elbowAngle > 65000){  //less than 0 or over run
                        elbowAngle = (char)0;
                    }
                    else if(elbowAngle > (char)90){             //greater than 90
                        elbowAngle = (char)90;
                    }
                    char shoulderAngle = (char)(shoulder.Angle(spine, elbow) - 90); //angle off by 90 deg
                    if (shoulderAngle < (char)15 || shoulderAngle > 65000){   //less than 15 or over run
                        shoulderAngle = (char)15;
                    }
                    else if (shoulderAngle > (char)165){        //greater than 165
                        shoulderAngle = (char)165;
                    }
                    char wristAngle = (char)(wrist.Angle(elbow, hand)-90);
                    if (wristAngle < (char)0){                  //less than 0
                        wristAngle = (char)0;
                    }
                    else if (wristAngle > (char)180){           //greater than 180
                        wristAngle = (char)180;
                    }
                    switch (body.HandRightState){
                        case HandState.Open:
                            handState = (char)10;   //10 degrees
                            prevHandState = handState;
                            break;
                        case HandState.Closed:
                            handState = (char)73;    //73degrees
                            prevHandState = handState;
                            break;
                        default:
                            handState = prevHandState;
                            break;
                    }
                    UpdateKinectDataTextBox("Shoulder RotationX: ", shoulderRotationX);
                    UpdateKinectDataTextBox("Shoulder RotationY: ", shoulderRotationY);
                    UpdateKinectDataTextBox("Shoulder RotationZ: ", shoulderRotationZ);
                    UpdateKinectDataTextBox("Shoulder angle: ", shoulderAngle);
                    UpdateKinectDataTextBox("Elbow angle: ", elbowAngle);
                    UpdateKinectDataTextBox("Wrist angle: ", wristAngle);
                    UpdateKinectDataTextBox("Wrist RotationX: ", wristRotationX);
                    UpdateKinectDataTextBox("Wrist RotationY: ", wristRotationY);
                    UpdateKinectDataTextBox("Wrist RotationZ: ", wristRotationZ);
                    UpdateKinectDataTextBox("Hand State: ", handState);
                    sendAnglesString = elbowAngle.ToString() + wristAngle.ToString() +
                    wristRotationZ.ToString() + handState.ToString() + shoulderRotationZ.ToString()
                    + shoulderAngle.ToString();
                    sendAngles = Encoding.ASCII.GetBytes(sendAnglesString);
                    ready = true;
                }
            }
        }
    }
}