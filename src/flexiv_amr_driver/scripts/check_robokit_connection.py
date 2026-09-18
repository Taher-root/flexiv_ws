#!/usr/bin/env python3
"""
Test script to verify Robokit TCP API connection.
Run this before launching the full ROS 2 stack.

Usage:
    python3 scripts/check_robokit_connection.py

Named check_* rather than test_* so pytest does not collect it during
`colcon test` and try to open sockets to the robot.
"""

import socket
import json
import sys
import os

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'flexiv_amr_driver')
)
from robokit_protocol import (
    API_PORT_STATE,
    API_PORT_CTRL,
    robot_status_loc_req,
    robot_control_motion_req,
    packMsg,
    unpackHead,
    create_stop_command
)

AMR_IP = '192.168.1.110'

def test_state_connection():
    """Test connection to STATE API (port 19204)"""
    print("\n=== Test 1: STATE API Connection ===")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((AMR_IP, API_PORT_STATE))
        print(f"✓ Connected to STATE API at {AMR_IP}:{API_PORT_STATE}")
        
        # Query location
        sock.send(packMsg(1, robot_status_loc_req, {}))
        data = sock.recv(16)
        jsonLen, reqNum = unpackHead(data)
        
        if jsonLen > 0:
            data = sock.recv(1024)
            result = json.loads(data)
            print(f"✓ Received location data:")
            print(f"  x: {result.get('x', 'N/A')}")
            print(f"  y: {result.get('y', 'N/A')}")
            print(f"  angle: {result.get('angle', 'N/A')}")
        
        sock.close()
        return True
    except Exception as e:
        print(f"✗ STATE API test failed: {e}")
        return False

def test_ctrl_connection():
    """Test connection to CTRL API (port 19205)"""
    print("\n=== Test 2: CTRL API Connection ===")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((AMR_IP, API_PORT_CTRL))
        print(f"✓ Connected to CTRL API at {AMR_IP}:{API_PORT_CTRL}")
        
        # Send stop command
        cmd = create_stop_command()
        sock.send(packMsg(1, robot_control_motion_req, cmd))
        print(f"✓ Sent stop command: {cmd}")
        
        try:
            data = sock.recv(16)
            print(f"✓ Received response from robot")
        except socket.timeout:
            print(f"✓ Command sent (no response expected)")
        
        sock.close()
        return True
    except Exception as e:
        print(f"✗ CTRL API test failed: {e}")
        return False

def main():
    print("=" * 60)
    print("Robokit TCP API Connection Test")
    print("=" * 60)
    print(f"Target Robot: {AMR_IP}")
    print(f"Testing ports: {API_PORT_STATE} (STATE), {API_PORT_CTRL} (CTRL)")
    
    state_ok = test_state_connection()
    ctrl_ok = test_ctrl_connection()
    
    print("\n" + "=" * 60)
    print("Test Results:")
    print("=" * 60)
    print(f"STATE API: {'✓ PASS' if state_ok else '✗ FAIL'}")
    print(f"CTRL API:  {'✓ PASS' if ctrl_ok else '✗ FAIL'}")
    
    if state_ok and ctrl_ok:
        print("\n✓ All tests passed! Ready to launch ROS 2 nodes.")
        return 0
    else:
        print("\n✗ Some tests failed. Check network connection and AMR IP.")
        return 1

if __name__ == '__main__':
    sys.exit(main())
