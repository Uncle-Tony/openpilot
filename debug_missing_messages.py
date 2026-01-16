#!/usr/bin/env python3
"""
Debug script to detect which CAN messages are missing that the code expects.
Run this on your comma device while connected to the car.

Usage: python3 debug_missing_messages.py
"""

import time
from collections import defaultdict
from cereal import messaging

# Messages actually read in carstate.py and carstate_ext.py
# Based on get_can_parsers() in carstate.py
EXPECTED_MESSAGES = {
  0: [  # Bus 0 (Powertrain)
    (0x208, "ESP_Status"),              # 520 - Vehicle speed
    (0x150, "VDM_PropStatus"),          # 336 - Gas pedal, gear
    (0x390, "EPAS_AdasStatus"),         # 912 - Steering angle, rate, fault detection
    (0x380, "EPAS_SystemStatus"),       # 896 - Steering torque
    (0x38f, "iBESP2"),                  # 911 - Brake pressed
    (0x33a, "RCM_Status"),              # 826 - Seatbelt (RCM_Status_IND_WARN_BELT_DRIVER)
    (0x321, "SCCM_WheelTouch"),          # 801 - Wheel touch (read for passthrough)
    (0x162, "VDM_AdasSts"),             # 354 - ADAS status (read for passthrough, used for cancel)
  ],
  1: [  # Bus 1 (ADAS)
    (0x370, "ACM_tsrCmd"),               # 880 - Speed limit
    (0x4f1, "Cluster"),                 # 1265 - Speed unit, vehicle speed
    (0x235, "IndicatorLights"),         # 565 - Doors, blinkers
  ],
  2: [  # Bus 2 (Camera)
    (0x120, "ACM_lkaHbaCmd"),           # 288 - LKAS/HBA command (read for passthrough, sent when OP enabled)
    (0x100, "ACM_Status"),              # 256 - Cruise enabled (ACM_FeatureStatus), fault status
    (0x101, "ACM_AebRequest"),          # 257 - AEB request (stockAeb)
  ],
}

# Optional messages (only if LONGITUDINAL_HARNESS_UPGRADE flag is set)
OPTIONAL_MESSAGES = {
  5: [  # Bus 5 (Park/Alt)
    (0x31A, "WheelButtons"),           # Speed adjustment buttons
    (0x350, "BSM_BlindSpotIndicator"),  # Blindspot detection
  ],
}

def main():
  sm = messaging.SubMaster(['can'])

  seen_messages = defaultdict(set)
  message_counts = defaultdict(lambda: defaultdict(int))
  start_time = time.time()

  print("=" * 80)
  print("🔍 Rivian CAN Message Debug Tool")
  print("=" * 80)
  print("\nListening to CAN messages for 10 seconds...")
  print("Make sure the car is on and openpilot is running!\n")

  while time.time() - start_time < 10:
    sm.update(1000)

    for msg in sm['can']:
      bus = msg.src
      addr = msg.address
      seen_messages[bus].add(addr)
      message_counts[bus][addr] += 1

  elapsed = time.time() - start_time
  print(f"✅ Captured {elapsed:.1f} seconds of data\n")
  print("=" * 80)

  # Check each bus
  missing_count = 0
  found_count = 0

  for bus_num in [0, 1, 2]:
    print(f"\n🚌 BUS {bus_num}:")
    expected = EXPECTED_MESSAGES.get(bus_num, [])

    if not expected:
      print("  No expected messages")
      continue

    for addr, name in expected:
      if addr in seen_messages[bus_num]:
        count = message_counts[bus_num][addr]
        hz = count / elapsed
        print(f"  ✅ 0x{addr:03X} {name:30s} {hz:5.1f} Hz ({count} msgs)")
        found_count += 1
      else:
        print(f"  ❌ 0x{addr:03X} {name:30s} NOT FOUND!")
        missing_count += 1

  # Check optional messages
  optional_missing = 0
  optional_found = 0
  for bus_num in [5]:
    optional = OPTIONAL_MESSAGES.get(bus_num, [])
    if optional:
      print(f"\n🚌 BUS {bus_num} (Optional - LONGITUDINAL_HARNESS_UPGRADE only):")
      for addr, name in optional:
        if addr in seen_messages[bus_num]:
          count = message_counts[bus_num][addr]
          hz = count / elapsed
          print(f"  ✅ 0x{addr:03X} {name:30s} {hz:5.1f} Hz ({count} msgs)")
          optional_found += 1
        else:
          print(f"  ⚠️  0x{addr:03X} {name:30s} NOT FOUND (optional)")
          optional_missing += 1

  print("\n" + "=" * 80)
  print(f"📊 SUMMARY:")
  print(f"  ✅ Found: {found_count} required messages")
  print(f"  ❌ Missing: {missing_count} required messages")
  if optional_found > 0 or optional_missing > 0:
    print(f"  ✅ Optional found: {optional_found}")
    print(f"  ⚠️  Optional missing: {optional_missing}")
  print("=" * 80)

  if missing_count > 0:
    print("\n⚠️  MISSING REQUIRED MESSAGES WILL CAUSE:")
    print("   - KeyError exceptions if code tries to read them")
    print("   - CAN faults if they're subscribed in parser")
    print("\n💡 These messages need to be added to your tap or made optional in code.")
  else:
    print("\n✅ All required messages are present!")

  # Show all messages seen (for reference)
  print(f"\n\n📋 ALL MESSAGES SEEN ON EACH BUS:")
  for bus_num in sorted(seen_messages.keys()):
    print(f"\n  Bus {bus_num} ({len(seen_messages[bus_num])} unique messages):")
    expected_addrs = {addr for addr, _ in EXPECTED_MESSAGES.get(bus_num, [])}
    optional_addrs = {addr for addr, _ in OPTIONAL_MESSAGES.get(bus_num, [])}
    for addr in sorted(seen_messages[bus_num]):
      count = message_counts[bus_num][addr]
      hz = count / elapsed
      if addr in expected_addrs:
        marker = "✅"
      elif addr in optional_addrs:
        marker = "⚠️ "
      else:
        marker = "🔵"
      print(f"    {marker} 0x{addr:03X}: {hz:5.1f} Hz ({count} msgs)")

if __name__ == "__main__":
  main()
