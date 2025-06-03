import time
import struct

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.ah_api import create_read_reg_msg, create_write_reg_msg
from ah_wrapper.ppp_stuffing import PPPUnstuff


def main():
    """This is an example of writing code for directly writing to the registers
    see Table of addresses in data sheet for more information"""
    client = AHSerialClient(write_thread=False, read_thread=False)
    client.start_time = time.time()

    # Default gain values
    descriptions = [
        "P gain value",
        "P gain radix",
        "I gain value",
        "I gain radix",
        "D gain value",
        "D gain radix",
        "I division value",
        "I saturation value",
        "output right-shift",
        "output_saturation (read only)",
    ]
    index = [3, 4, 1, 21, 0, 10, 1, 1773, 4]
    middle = [3, 4, 1, 21, 0, 10, 1, 1773, 4]
    ring = [3, 4, 1, 21, 0, 10, 1, 1773, 4]
    pinky = [3, 4, 1, 21, 0, 10, 1, 1773, 4]
    thumbflexor = [3, 4, 1, 21, 0, 10, 1, 1773, 4]
    thumbrotator = [500, 0, 5, 3, 14, 6, 1, 1773, 8]
    gains = [index, middle, ring, pinky, thumbflexor, thumbrotator]

    channels = [0, 1, 2, 3, 4, 5]
    channel_lookup = (
        "index",
        "middle",
        "ring",
        "pinky",
        "thumbflexor",
        "thumbrotator",
    )
    for channel in channels:
        for enumval in range(0, len(descriptions) - 1):
            val_addr = channel * len(descriptions) + enumval + 1

            # Read Value
            rv = read_uart_register(val_addr=val_addr, client=client)
            if rv:
                v = struct.unpack("<i", bytearray(rv[4:8]))[0]
                print(f"{channel_lookup[channel]} Val: {enumval} Initial =", v)
            else:
                print(f"Failed to read address {val_addr}")

            # Write Value
            rc = write_uart_register(
                val_addr=val_addr, val=gains[channel][enumval], client=client
            )
            v = struct.unpack("<i", bytearray(rc[0:4]))[0]
            if v:
                if v != 0x4E11FEA4:
                    print("failed to write")
                else:
                    print(
                        f"wrote {descriptions[enumval]} = {gains[channel][enumval]} to address {hex(val_addr)} succeeded"
                    )
            else:
                print("failed to write")

            # Get Updated Value
            rv = read_uart_register(val_addr=val_addr, client=client)
            if rv:
                v = struct.unpack("<i", bytearray(rv[4:8]))[0]
                print(
                    f"{channel_lookup[channel]} Val: {enumval} Updated Val =",
                    v,
                )
            else:
                print(f"Failed to read address {val_addr}")

            print("---------------------------------------------------")

    rv = read_uart_register(val_addr=61, client=client)
    v = struct.unpack("<i", bytearray(rv[4:8]))[0]
    print("reconstructed value:", hex(v))
    rv = read_uart_register(val_addr=62, client=client)
    v = struct.unpack("<i", bytearray(rv[4:8]))[0]
    print("reconstructed value:", hex(v))
    client.close()


def read_uart_register(val_addr: int, client: AHSerialClient):
    msg = create_read_reg_msg(val_addr)
    response = write_and_get_response(client=client, msg=msg)
    return response


def write_uart_register(val_addr: int, val: int, client: AHSerialClient):
    msg = create_write_reg_msg(val_addr=val_addr, val=val)
    response = write_and_get_response(client=client, msg=msg)
    return response


def write_and_get_response(client: AHSerialClient, msg: bytearray):
    max_attempts = 30
    sleep_time = 0.001
    while True:
        attempts = 0
        client._conn.write(msg)
        client._conn.n_writes += 1
        unstuffer = PPPUnstuff()
        while attempts < max_attempts:
            time.sleep(sleep_time)
            msg = client._conn.read(512)
            attempts += 1
            if msg:
                for b in msg:
                    frame = unstuffer.unstuff_byte(b)
                    if frame:
                        client.n_reads += 1
                        return frame


if __name__ == "__main__":
    main()
