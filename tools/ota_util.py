#!/usr/bin/env python3
"""
OTA Firmware Utility Script

This script generates MD5 checksums for firmware files and creates
C code for embedding the checksum in the firmware.
"""

import hashlib
import sys
import os
import argparse
from datetime import datetime

def calculate_md5(file_path):
    """Calculate MD5 checksum of a file"""
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def generate_c_code(md5_hex, output_file=None):
    """Generate C code for MD5 checksum"""
    # Convert hex string to byte array
    md5_bytes = [f"0x{md5_hex[i:i+2]}" for i in range(0, 32, 2)]

    c_code = f"""/*
 * Auto-generated MD5 checksum for OTA firmware
 * Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
 * MD5: {md5_hex}
 */

#include <stdint.h>

// MD5 checksum for firmware validation
const uint8_t firmware_md5[16] = {{
    {', '.join(md5_bytes)}
}};

"""

    if output_file:
        with open(output_file, 'w') as f:
            f.write(c_code)
        print(f"C code written to {output_file}")
    else:
        print(c_code)

def main():
    parser = argparse.ArgumentParser(description="OTA Firmware Utility")
    parser.add_argument("firmware", help="Path to firmware .bin file")
    parser.add_argument("-c", "--c-output", help="Output C file for checksum")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")

    args = parser.parse_args()

    if not os.path.exists(args.firmware):
        print(f"Error: Firmware file '{args.firmware}' not found")
        return 1

    # Calculate MD5
    md5_hex = calculate_md5(args.firmware)

    if args.verbose:
        file_size = os.path.getsize(args.firmware)
        print(f"Firmware file: {args.firmware}")
        print(f"File size: {file_size} bytes")
        print(f"MD5 checksum: {md5_hex}")

    # Generate C code if requested
    if args.c_output:
        generate_c_code(md5_hex, args.c_output)
    else:
        print(f"MD5: {md5_hex}")

    return 0

if __name__ == "__main__":
    sys.exit(main())