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
        self.callback = XdaCallback(max_buffer_size)
        self.device = None
        self.port = None

    def initialize(self):
        # Show XDA version
        version = xda.XsVersion()
        xda.xdaVersion(version)
        print("Using XDA version %s" % version.toXsString())

        # Scan ports
        print("Scanning for devices...")
        ports = xda.XsScanner_scanPorts()
        for i in range(ports.size()):
            if ports[i].deviceId().isMti() or ports[i].deviceId().isMtig():
                self.port = ports[i]
                break

        if not self.port or self.port.empty():
            raise RuntimeError("No MTi device found. Aborting.")

        did = self.port.deviceId()
        print("Found a device with:")
        print(" Device ID: %s" % did.toXsString())
        print(" Port name: %s" % self.port.portName())

        # Open port
        print("Opening port...")
        if not self.control.openPort(self.port.portName(), self.port.baudrate()):
            raise RuntimeError("Could not open port. Aborting.")

        self.device = self.control.device(did)
        assert self.device != 0, "Could not construct device object"
        print("Device: %s, with ID: %s opened." % (self.device.productCode(), self.device.deviceId().toXsString()))

        self.device.addCallbackHandler(self.callback)

    def configure(self):
        print("Putting device into configuration mode...")
        if not self.device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        print("Configuring the device...")
        cfg = xda.XsOutputConfigurationArray()
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))

        did = self.device.deviceId()
        if did.isImu():
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_MagneticField, 100))
        elif did.isVru() or did.isAhrs():
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
        elif did.isGnss():
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_LatLon, 100))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_AltitudeEllipsoid, 100))
            cfg.push_back(xda.XsOutputConfiguration(xda.XDI_VelocityXYZ, 100))
        else:
            raise RuntimeError("Unknown device while configuring. Aborting.")

        if not self.device.setOutputConfiguration(cfg):
            raise RuntimeError("Could not configure the device. Aborting.")

    def start(self):
        print("Putting device into measurement mode...")
        if not self.device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")
        print("Started. Press Ctrl+C to stop.\n")

    def read_loop(self):
        try:
            while True:
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
                        q = packet.orientationQuaternion()
                        s = f"q0: {q[0]:.2f}, q1: {q[1]:.2f}, q2: {q[2]:.2f}, q3: {q[3]:.2f}"
                        e = packet.orientationEuler()
                        s += f" |Roll: {e.x():.2f}, Pitch: {e.y():.2f}, Yaw: {e.z():.2f}"

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
        if self.port:
            self.control.closePort(self.port.portName())
        print("Closing XsControl object...")
        if self.control:
            self.control.close()
        print("Done.")