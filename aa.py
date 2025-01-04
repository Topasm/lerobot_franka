import pyrealsense2 as rs


def get_realsense_serial_numbers():
    context = rs.context()
    serial_numbers = []

    for device in context.query_devices():
        serial_number = device.get_info(rs.camera_info.serial_number)
        serial_numbers.append(serial_number)

    return serial_numbers


if __name__ == "__main__":
    serials = get_realsense_serial_numbers()
    print("Connected RealSense serial numbers:", serials)
