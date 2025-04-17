"""!
@file radarSensor.py
@brief Configures and communicates with the IWR6843 radar sensor via UART.

@details This module provides an interface for configuring the radar sensor through
UART communication. It sends initialization commands to the sensor over a specified
COM port, ensuring proper startup configuration.

@defgroup Radar_Sensor Radar Sensor Communication
@brief Provides functions for configuring and communicating with the radar sensor.
@{
"""

import serial
import time

__all__ = ['sendConfiguration']


def send_configuration(configuration_commands, configuration_port, baudrate=115200):
    """! 
    @brief Configures the IWR6843 radar sensor via UART.

    @param in configuration_commands   List of commands to initialize the sensor.
    @param in configuration_port   COM-Port for sending the commands to the sensor.
    @param in baudrate Baudrate used for communicating with the configuration COM-Port (standard 115200).

    @return None
    """
    
    ser = serial.Serial(configuration_port, baudrate, timeout=1)
    time.sleep(2)

    for command in configuration_commands:
        ser.write((command + '\n').encode())
        print(f"Sent: {command}")
        time.sleep(0.1)
    ser.close()

## @}  # End of Radar_Sensor group