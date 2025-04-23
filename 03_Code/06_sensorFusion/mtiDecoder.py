import sys
import xsensdeviceapi as xda
from threading import Lock

class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size=5):
        super(XdaCallback, self).__init__()
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = []
        self.m_lock = Lock()

    def packetAvailable(self):
        with self.m_lock:
            return len(self.m_packetBuffer) > 0

    def getNextPacket(self):
        with self.m_lock:
            assert self.m_packetBuffer, "No packet available"
            oldest = self.m_packetBuffer.pop(0)
        return xda.XsDataPacket(oldest)

    def onLiveDataAvailable(self, dev, packet):
        with self.m_lock:
            assert packet != 0
            while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
                self.m_packetBuffer.pop()
            self.m_packetBuffer.append(packet)

class MTi_G_710:
    def __init__(self, max_buffer_size=5):
        print("Creating XsControl object...")
        self.control = xda.XsControl_construct()
        assert self.control != 0, "Failed to construct XsControl"
        self.callback = None
        self.device = None
        self.mtPort = None

    def initialize(self):
        # Show XDA version
        version = xda.XsVersion()
        xda.xdaVersion(version)
        print("Using XDA version %s" % version.toXsString())

        try:
            
            print("Scanning for devices...")
            portInfoArray = xda.XsScanner_scanPorts()
            # Find an MTi device
            self.mtPort = xda.XsPortInfo()
            for i in range(portInfoArray.size()):
                if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                    self.mtPort = portInfoArray[i]
                    break

            if self.mtPort.empty():
                raise RuntimeError("No MTi device found. Aborting.")

            did = self.mtPort.deviceId()
            print("Found a device with:")
            print(" Device ID: %s" % did.toXsString())
            print(" Port name: %s" % self.mtPort.portName())

            # Open port
            print("Opening port...")
            if not self.control.openPort(self.mtPort.portName(), self.mtPort.baudrate()):
                raise RuntimeError("Could not open port. Aborting.")

            self.device = self.control.device(did)
            assert self.device != 0, "Could not construct device object"

            print("Device: %s, with ID: %s opened." % (self.device.productCode(), self.device.deviceId().toXsString()))
            
            self.callback = XdaCallback()
            self.device.addCallbackHandler(self.callback)
            # Call for the configuration of the same class
            self.configure()
            # Start the device in measurement mode
            self.start()
        except RuntimeError as error:
            print(error)
            sys.exit(1)
        except:
            print("An unknown fatal error has occurred. Aborting.")
            sys.exit(1)
        else:
            print("Successful exit.")

    def configure(self):
        print("Putting device into configuration mode...")
        if not self.device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        print("Configuring the device...")
        configArray = xda.XsOutputConfigurationArray()
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))

        sddid = self.device.deviceId()
        if sddid.isImu():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))
        elif sddid.isVru() or sddid.isAhrs():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
        elif sddid.isGnss():
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon, 100))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid, 100))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_VelocityXYZ, 100))
        else:
            raise RuntimeError("Unknown device while configuring. Aborting.")

        if not self.device.setOutputConfiguration(configArray):
            raise RuntimeError("Could not configure the device. Aborting.")

    def start(self):
        print("Putting device into measurement mode...")
        if not self.device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")
        print("Started. Press Ctrl+C to stop.\n")

    def read_loop(self):
        try:
            if self.callback.packetAvailable():
                packet = self.callback.getNextPacket()
                s = ""
                if packet.containsCalibratedData():
                    acc = packet.calibratedAcceleration()
                    s += f"Acc X: {acc[0]:.2f}, Y: {acc[1]:.2f}, Z: {acc[2]:.2f}"
                    gyr = packet.calibratedGyroscopeData()
                    s += f" |Gyr X: {gyr[0]:.2f}, Y: {gyr[1]:.2f}, Z: {gyr[2]:.2f}"
                    mag = packet.calibratedMagneticField()
                    s += f" |Mag X: {mag[0]:.2f}, Y: {mag[1]:.2f}, Z: {mag[2]:.2f}"

                if packet.containsOrientation():
                    quaternion = packet.orientationQuaternion()
                    s = f"q0: {quaternion[0]:.2f}, q1: {quaternion[1]:.2f}, q2: {quaternion[2]:.2f}, q3: {quaternion[3]:.2f}"
                    euler = packet.orientationEuler()
                    s += f" |Roll: {euler.x():.2f}, Pitch: {euler.y():.2f}, Yaw: {euler.z():.2f}"

                if packet.containsLatitudeLongitude():
                    latlon = packet.latitudeLongitude()
                    s += f" |Lat: {latlon[0]:7.2f}, Lon: {latlon[1]:7.2f}"

                if packet.containsAltitude():
                    s += f" |Alt: {packet.altitude():7.2f}"

                if packet.containsVelocity():
                    vel = packet.velocity(xda.XDI_CoordSysEnu)
                    s += f" |E: {vel[0]:7.2f}, N: {vel[1]:7.2f}, U: {vel[2]:7.2f}"

                print(f"{s}\r", end="", flush=True)
        except KeyboardInterrupt:
            print("\nStopped by user.")

    def shutdown(self):
        print("Removing callback handler...")
        if self.device:
            self.device.removeCallbackHandler(self.callback)
        print("Closing port...")
        if self.mtPort:
            self.control.closePort(self.mtPort.portName())
        print("Closing XsControl object...")
        if self.control:
            self.control.close()
        print("Done.")