# Python script to convert flight data logged as binary data into a CSV file
import ctypes
import sys

if len(sys.argv) < 3:
    print(f'Usage: {sys.argv[0]} input_file.data output_file.csv')
    exit(1)

log_file = sys.argv[1]
out_file = sys.argv[2]
log_file_stream = open(log_file, 'rb')
out_file_stream = open(out_file, 'w')

# Header for output file
out_file_stream.write(f'time,high_g_x,high_g_y,high_g_z,'
                      f'bmx_x_magn,bmx_y_magn,bmx_z_magn,'
                      f'bmx_x_gyro,bmx_y_gyro,bmx_z_gyro,'
                      f'bmx_x_accel,bmx_y_accel,bmx_z_accel,'
                      f'cpu_temp,real_temp,'
                      f'baro_height,gps_height,gps_satCount,'
                      f'gps_lat,gps_lon,gps_ascent,'
                      f'gps_groundSpeed,'
                      f'telemetrum_board.arm_status,'
                      f'telemetrum_board.current,'
                      f'telemetrum_board.voltage,'
                      f'stratologger_board.arm_status,'
                      f'stratologger_board.current,'
                      f'stratologger_board.voltage,'
                      f'camera_board.arm_status,'
                      f'camera_board.current,'
                      f'camera_board.voltage,'
                      f'v3_rail_voltage,v5_rail_voltage,'
                      f'mainBatteryVoltage,mainBatteryTemperature,'
                      '\n')


# Simple class to store data for each arming board
class ArmingBoard:
    arm_status = False
    # current in mA
    current = 0
    voltage = 0


# Define struct for each of the arming boards
telemetrum_board = ArmingBoard()
stratologger_board = ArmingBoard()
camera_board = ArmingBoard()

# count which line we are on
line_num = 1

# These bytes need to be stored into variable so we can check end of file
time_bytes = log_file_stream.read(4)

while len(time_bytes) == 4:
    # 4 bytes per float value
    time = ctypes.c_float.from_buffer_copy(time_bytes).value

    high_g_x = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    high_g_y = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    high_g_z = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value

    bmx_x_magn = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    bmx_y_magn = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    bmx_z_magn = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value

    bmx_x_gyro = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    bmx_y_gyro = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    bmx_z_gyro = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value

    bmx_x_accel = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value
    bmx_y_accel = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value
    bmx_z_accel = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value

    cpu_temp = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    real_temp = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value

    baro_height = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value

    gps_height = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value

    # sat count is uint8 not float
    gps_satCount = ctypes.c_uint8.from_buffer_copy(
        log_file_stream.read(1)).value

    gps_lat = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    gps_lon = ctypes.c_float.from_buffer_copy(log_file_stream.read(4)).value
    gps_ascent = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value
    gps_groundSpeed = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value

    # Write data recived from CAN bus
    # ser.write(bytes(ctypes.c_float(mainBatteryCurrent)))
    # ser.write(bytes(ctypes.c_float(mainBatteryVoltage)))

    # Note the order:
    for arm_board in [telemetrum_board, stratologger_board, camera_board]:
        # Arm status is bool, but stored as 1 or 0 for CSV
        arm_board.arm_status = ctypes.c_uint8.from_buffer_copy(
            log_file_stream.read(1)).value
        # Floats for current and voltage
        arm_board.current = ctypes.c_float.from_buffer_copy(
            log_file_stream.read(4)).value
        arm_board.voltage = ctypes.c_float.from_buffer_copy(
            log_file_stream.read(4)).value

    v3_rail_voltage = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value
    v5_rail_voltage = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value
    mainBatteryVoltage = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value
    mainBatteryTemperature = ctypes.c_float.from_buffer_copy(
        log_file_stream.read(4)).value

    # Check that we have new line after reading data
    if log_file_stream.read(1).decode('ascii') != '\n':
        print('FAILED TO DECODE FILE.')
        print(f'Failed on line {line_num}')

    # Write a big line to the outfile with all the data:
    out_file_stream.write(f'{time},{high_g_x},{high_g_y},{high_g_z},'
                          f'{bmx_x_magn},{bmx_y_magn},{bmx_z_magn},'
                          f'{bmx_x_gyro},{bmx_y_gyro},{bmx_z_gyro},'
                          f'{bmx_x_accel},{bmx_y_accel},{bmx_z_accel},'
                          f'{cpu_temp},{real_temp},'
                          f'{baro_height},{gps_height},{gps_satCount},'
                          f'{gps_lat},{gps_lon},{gps_ascent},'
                          f'{gps_groundSpeed},'
                          f'{telemetrum_board.arm_status},'
                          f'{telemetrum_board.current},'
                          f'{telemetrum_board.voltage},'
                          f'{stratologger_board.arm_status},'
                          f'{stratologger_board.current},'
                          f'{stratologger_board.voltage},'
                          f'{camera_board.arm_status},'
                          f'{camera_board.current},'
                          f'{camera_board.voltage},'
                          f'{v3_rail_voltage},{v5_rail_voltage},'
                          f'{mainBatteryVoltage},{mainBatteryTemperature},'
                          '\n')

    line_num = line_num + 1
    # Need to start next line now to can check end of file on while condition
    time_bytes = log_file_stream.read(4)

out_file_stream.close()
log_file_stream.close()
