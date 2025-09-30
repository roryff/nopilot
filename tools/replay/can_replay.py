#!/usr/bin/env python3
import argparse
import os
import time
import usb1
import threading

os.environ['FILEREADER_CACHE'] = '1'

from openpilot.common.realtime import config_realtime_process, Ratekeeper, DT_CTRL
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.tools.lib.logreader import LogReader
from openpilot.common.params import Params
from cereal import messaging

# set both to cycle power or ignition
PWR_ON = int(os.getenv("PWR_ON", "0"))
PWR_OFF = int(os.getenv("PWR_OFF", "0"))
IGN_ON = int(os.getenv("ON", "0"))
IGN_OFF = int(os.getenv("OFF", "0"))
ENABLE_IGN = IGN_ON > 0 and IGN_OFF > 0
ENABLE_PWR = PWR_ON > 0 and PWR_OFF > 0


def send_thread_can_msgs():
  """Send real CAN messages from route data through messaging system"""
  params = Params()
  pm = messaging.PubMaster(['canReplay'])  # Use separate topic to avoid conflict

  rk = Ratekeeper(1 / DT_CTRL, print_delay_threshold=None)
  print("Starting CAN message replay through canReplay messaging topic")

  while True:
    try:
      is_onroad = params.get_bool("IsOnroad")

      if is_onroad and len(CAN_MSGS) > 0:
        # Get the next set of CAN messages from the route data
        send = CAN_MSGS[rk.frame % len(CAN_MSGS)]
        send = list(filter(lambda x: x[-1] <= 2, send))  # Filter to buses 0-2

        if send:
          # Create messaging format for canReplay topic
          msg = messaging.new_message('can', len(send))
          for i, (addr, data, src) in enumerate(send):
            msg.can[i].address = addr
            msg.can[i].dat = data
            msg.can[i].src = src

          pm.send('canReplay', msg)

      rk.keep_time()
    except Exception as e:
      print(f"Error in CAN replay: {e}")
      time.sleep(1)





def connect():
  config_realtime_process(3, 55)

  # Send CAN messages through messaging system (no hardware required)
  send_thread_can_msgs()


def load_route(route_or_segment_name):
  print("Loading log...")
  lr = LogReader(route_or_segment_name)
  CP = lr.first("carParams")
  print(f"carFingerprint: '{CP.carFingerprint}'")
  mbytes = [m.as_builder().to_bytes() for m in lr if m.which() == 'can']
  return [m[1] for m in can_capnp_to_list(mbytes)]


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Replay CAN messages from a route through messaging system.",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("route_or_segment_name", nargs='?', help="The route or segment name to replay. If not specified, a default public route will be used.")
  args = parser.parse_args()

  if args.route_or_segment_name is None:
    args.route_or_segment_name = "77611a1fac303767/2020-03-24--09-50-38/2:4"

  CAN_MSGS = load_route(args.route_or_segment_name)
  print(f"Loaded {len(CAN_MSGS)} CAN message frames from route")

  if ENABLE_PWR:
    print(f"Cycling power: on for {PWR_ON}s, off for {PWR_OFF}s")
  if ENABLE_IGN:
    print(f"Cycling ignition: on for {IGN_ON}s, off for {IGN_OFF}s")

  connect()
