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
        # 1) Timestamps
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
            # 2) Orientation (device’s built-in filter)
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
            # 3) Inertial data for dead-reckoning
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
            # 4) (Optional) Magnetometer for heading correction
            configArray.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))
            # 5) (Optional) Fusion with GNSS
            #configArray.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon, 100))
            #configArray.push_back(xda.XsOutputConfiguration(xda.XDI_VelocityXYZ, 100))
            #configArray.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid, 100))
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
        MTi_data = {}
        try:
            if self.callback.packetAvailable():
                packet = self.callback.getNextPacket()
                if packet.containsCalibratedData():
                    acc = packet.calibratedAcceleration()
                    gyr = packet.calibratedGyroscopeData()
                    mag = packet.calibratedMagneticField()

                    MTi_data["acceleration"] = {"x": acc[0], "y": acc[1], "z": acc[2]}
                    MTi_data["gyroscope"] = {"x": gyr[0], "y": gyr[1], "z": gyr[2]}
                    MTi_data["magnetometer"] = {"x": mag[0], "y": mag[1], "z": mag[2]}

                if packet.containsOrientation():
                    quaternion = packet.orientationQuaternion()
                    euler = packet.orientationEuler()

                    MTi_data["quaternion"] = {"q0": quaternion[0], "q1": quaternion[1], "q2": quaternion[2], "q3": quaternion[3]}
                    MTi_data["euler"] = {"roll": euler.x(), "pitch": euler.y(), "yaw": euler.z()}

                if packet.containsLatitudeLongitude():
                    latlon = packet.latitudeLongitude()
                    MTi_data["position"] = {"latitude": latlon[0], "longitude": latlon[1]}

                if packet.containsAltitude():
                    MTi_data["altitude"] = packet.altitude()

                if packet.containsVelocity():
                    vel = packet.velocity(xda.XDI_CoordSysEnu)
                    MTi_data["velocity"] = {"east": vel[0], "north": vel[1], "up": vel[2]}
                return MTi_data
            else:
                return MTi_data
        except:
            return 0

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