from __future__ import print_function
import argparse
import math
import time
import sys
import random
from triad_openvr import triad_openvr
from pythonosc import osc_message_builder
from pythonosc import osc_bundle_builder
from pythonosc import udp_client
from reprint import output
from colorama import Fore, Back, Style
import openvr

def get_controller_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
    left = None
    right = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right


def from_controller_state_to_dict(pControllerState):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting
    return d

print(Back.CYAN + Fore.WHITE + Style.BRIGHT +
"""                            \n      OpenVR OSC 1.0        \n                            \n"""
 + Style.RESET_ALL)

# Initialize Tria's OpenVR wrapper and print discovered objects
v = triad_openvr.triad_openvr()
print(Style.DIM)
v.print_discovered_objects()
print(Style.RESET_ALL)

# Sort through all discovered devices and keep track by type
deviceCount = 0
devices = {
    'tracker': [],
    'hmd': [],
    'controller': [],
    'tracking reference': []
}

for deviceName, device in v.devices.items():
    device._id = deviceName.split("_").pop()
    devices[device.device_class.lower()].append(device)
    deviceCount += 1

if __name__ == "__main__":
    # Parse CLI arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="127.0.0.1", help="ip of the OSC server")
    parser.add_argument("--port", type=int, default=7000, help="port the OSC server is listening on")
    parser.add_argument("--track", nargs="*", default=["hmd", "tracker", "controller"], help="devices to track (hmd, tracker, controller)")
    parser.add_argument("--freq", type=int, default=250, help="tracking frequency (in ms)")
    parser.add_argument("--mode", choices=['euler', 'quaternion'], default="euler", help="get pose data in euler angles or quaternions")
    args = parser.parse_args()

    # pose tracking interval
    interval = 1/250

    # initialize OSC client
    client = udp_client.SimpleUDPClient(args.ip, args.port)
    print(client)

    # print some stuff
    print(Fore.GREEN + "\rSending OSC tracking data on " + args.ip + ":" + str(args.port), end="\n\n")
    print(Fore.YELLOW + '{0: <13}'.format("OSC address") + '{0: <9}'.format("X") + '{0: <9}'.format("Y") + '{0: <9}'.format("Z") + '{0: <9}'.format("Yaw") + '{0: <9}'.format("Pitch") + '{0: <9}'.format("Roll"))

    # Get the controllers ID's 
    vrsystem = openvr.VRSystem()

    left_id, right_id = None, None
    print("===========================")
    print("Waiting for controllers...")
    try:
        while left_id is None or right_id is None:
            left_id, right_id = get_controller_ids(vrsystem)
            if left_id and right_id:
                break
            print("Waiting for controllers...")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
        openvr.shutdown()

    time.sleep(10.0)
    while(True):
        start = time.time()

        # Initialize OSC bundle for all tracked controllers
        bundle = osc_bundle_builder.OscBundleBuilder(osc_bundle_builder.IMMEDIATELY)

        # Get Left controller buttons' actions
        # result, pControllerState = vrsystem.getControllerState(left_id)
        # d = from_controller_state_to_dict(pControllerState)

        # # Get Right controller buttons' actions
        # result, pControllerState = vrsystem.getControllerState(right_id)
        # d = from_controller_state_to_dict(pControllerState)


        # iterate over tracked device types and build OSC messages
        di = 0
        try:
            for deviceType in args.track:
                for device in devices[deviceType]:
                    # get device post
                    pose = device.get_pose_euler()

                    # Build message and add to bundle
                    msg = osc_message_builder.OscMessageBuilder(address="/" + deviceType + "/" + device._id)
                    # msg.add_arg(device.get_pose_euler())
                    
                    msg.add_arg(pose[0]) # X
                    msg.add_arg(pose[1]) # Y
                    msg.add_arg(pose[2]) # Z
                    msg.add_arg(pose[3]) # Yaw
                    msg.add_arg(pose[4]) # Pitch
                    msg.add_arg(pose[5]) # Roll

                    if deviceType == 'controller':
                        result, pControllerState = vrsystem.getControllerState(int(device._id))
                        # result, pControllerState = vrsystem.getControllerState(left_id)
                        d = from_controller_state_to_dict(pControllerState)
                        print(d)
                        # on trigger .y is always 0.0 says the docs
                        msg.add_arg(d['trigger'])
                        # 0.0 on trigger is fully released
                        # -1.0 to 1.0 on joystick and trackpads
                        msg.add_arg(d['trackpad_x'])
                        msg.add_arg(d['trackpad_y'])
                        # These are published and always 0.0
                        # for i in range(2, 5):
                        #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
                        #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
                        msg.add_arg(d['ulButtonPressed']/8589934592)
                        print(type(d['ulButtonTouched']))
                        msg.add_arg(d['ulButtonTouched']/8589934592)
                        # To make easier to understand what is going on
                        # Second bit marks menu button
                        msg.add_arg(d['menu_button'])
                        # 32 bit marks trackpad
                        msg.add_arg(d['trackpad_pressed'])
                        msg.add_arg(d['trackpad_touched']/4294967296)
                        # third bit marks grip button
                        msg.add_arg(d['grip_button'])
                        
                    bundle.add_content(msg.build())

                    # print(pose)

                    di += 1

            # Send the bundle
            client.send(bundle.build())

        except:
            pass

        # wait for next tick
        sleep_time = interval-(time.time()-start)
        if sleep_time>0:
            time.sleep(sleep_time)
            # time.sleep(1)
