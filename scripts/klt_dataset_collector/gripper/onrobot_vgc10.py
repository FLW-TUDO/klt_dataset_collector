#!/usr/bin/env python
import requests
import time

class OnRobotVGC10():
    def __init__(self, ip_address='192.168.1.1'):
        self.ip_address = 'http://' + ip_address + '/api/dc/vg10'
        self.release_all_channels()
        try:
            self.set_current_limit(500)
        except:
            raise Exception("couldn't connect to gripper, check IP address and network connection")

    # Power limit set
    def set_current_limit(self, power_value):
        try:
            self.check_power_limits(power_value)
            URL = self.ip_address + '/set_current_limit/0/' + str(int(power_value))
            return requests.get(url=URL)
        except:
            r = requests.Response()
            r.status_code = 416
            return r

    # Channel A pressure set
    def set_pressure_channel_A(self, pressure_value):
        try:
            self.check_pressure_limits(pressure_value)
            URL = self.ip_address + '/set_grip/0/0/' + str(int(pressure_value))
            return requests.get(url=URL)
        except:
            r = requests.Response()
            r.status_code = 416
            return r

    # Channel A release
    def release_channel_A(self):
        URL = self.ip_address + '/set_release/0/0'
        return requests.get(url=URL)

    # Channel B pressure set
    def set_pressure_channel_B(self, pressure_value):
        try:
            self.check_pressure_limits(pressure_value)
            URL = self.ip_address + '/set_grip/0/1/' + str(int(pressure_value))
            return requests.get(url=URL)
        except:
            r = requests.Response()
            r.status_code = 416
            return r

    # Channel B release
    def release_channel_B(self):
        URL = self.ip_address + '/set_release/0/1'
        return requests.get(url=URL)

    # Channel All pressure set
    def set_pressure_all_channels(self, pressure_value):
        try:
            self.check_pressure_limits(pressure_value)
            URL = self.ip_address + '/set_grip_all/0/' + str(int(pressure_value)) + '/' + str(int(pressure_value))
            return requests.get(url=URL)
        except:
            r = requests.Response()
            r.status_code = 416
            return r

    # Channel release All
    def release_all_channels(self):
        URL = self.ip_address + '/set_release_all/0'
        return requests.get(url=URL)

    def check_power_limits(self, power_value):
        if power_value < 100 or power_value > 1000:
            raise ValueError("Power value is outside of range (100-1000)")

    def check_pressure_limits(self, pressure_value):
        if pressure_value < 0 or pressure_value > 80:
            raise ValueError("Pressure value is outside of range (0-80)")


def test():
    gripper = OnRobotVGC10('172.28.60.250')
    gripper.set_current_limit(1000)

    print(gripper.set_pressure_channel_A(60))
    time.sleep(3)
    print(gripper.release_channel_A())

    time.sleep(1)
    print(gripper.set_pressure_channel_B(60))
    time.sleep(3)
    print(gripper.release_channel_B())

    time.sleep(1)
    print(gripper.set_pressure_all_channels(60))
    time.sleep(3)
    print(gripper.release_all_channels())

    return


if __name__ == '__main__':
    test()
