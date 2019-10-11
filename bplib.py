import datetime
from threading import Event
from time import sleep
from typing import List
import numpy as np
import pandas as pd
from mbientlab.metawear import MetaWear
from mbientlab.metawear import libmetawear
from mbientlab.metawear import parse_value, create_voidp, create_voidp_int
from mbientlab.metawear.cbindings import *
from mbientlab.warble import *

AD7745_ADDR = 0b01001000

REG_STATUS = 0x00  # Read
REG_CAP_DATA = 0x01  # Read
REG_VT_DATA = 0x04  # Read
REG_CAP_SETUP = 0x07  # Read/Write
REG_VT_SETUP = 0x08  # Read/Write
REG_EXC_SETUP = 0x09  # Read/Write
REG_CONFIGURATION = 0x0A  # Read/Write
REG_CAP_DAC_A = 0x0B  # Read/Write
REG_CAP_DAC_B = 0x0C  # Read/Write
REG_CAP_OFFSET = 0x0D  # Read/Write
REG_CAP_GAIN = 0x0F  # Read/Write
REG_VOLT_GAIN = 0x12  # Read/Write

RESET_ADDR = 0xBF

CAP_SETUP_PROP = 0b10000000
VT_SETUP_PROP = 0b00000001
EXC_SETUP_PROP = 0b00001011
CONFIGURATION_PROP = 0b00000001
CAP_DAC_A_PROP = 0b10011111

# data = {"TIMESTAMP":[0], "BP":[0], "ACC_X":[0], "ACC_Y":[0], "ACC_Z":[0], "GYRO_X":[0], "GYRO_Y":[0], "GYRO_Z":[0]}

# df = pd.DataFrame(columns=["TIMESTAMP","DATE","BP"])
# df2 = pd.DataFrame(columns=["TIMESTAMP", "DATA", "ACC_X", "ACC_Y", "ACC_Z"])
# df3 = pd.DataFrame(columns=["TIMESTAMP", "DATA", "GYRO_X", "GYRO_Y", "GYRO_Z"])

selection = -1
devices = None


class State:
    parameters = None
    timer = None
    timer_id = None
    i2c_signal = None

    i2c = None
    timeStamp = None
    acc_x = []
    acc_y = []
    acc_z = []
    gyro_x = []
    gyro_y = []
    gyro_z = []
    timeStamp_acc = []
    date_acc = []
    timeStamp_gyro = []
    date_gyro = []

    def __init__(self, device):
        self.device = device
        self.callback_i2c = FnVoid_VoidP_DataP(self.data_handler_i2c)
        self.callback_acc = FnVoid_VoidP_DataP(self.data_handler_acc)
        self.callback_gyro = FnVoid_VoidP_DataP(self.data_handler_gyro)

    def data_handler_i2c(self, ctx, data):
        cap_list = parse_value(data)
        timeStamp = data.contents.epoch
        # print("%s -> %s   %s" % (self.device.address, cap_list, ctx))
        value = self.calculateCAP(cap_list)
        # date = datetime.datetime.fromtimestamp(float(timeStamp)/1000).strftime('%Y-%m-%d %H:%M:%S:%f')
        date = datetime.datetime.fromtimestamp(float(timeStamp) / 1000).strftime('%H%M%S.%f')
        date2 = round(float(date), 3)

        cap = (value - 0x800000) / 0x800000 * 4.096
        temp = [float(timeStamp), cap]
        try:
            self.i2c = np.vstack((self.i2c, temp))
            # self.timeStamp = np.vstack((self.timeStampe, date2))
        except (ValueError, IndexError) as e:
            self.i2c = np.asarray(temp)
            self.i2c = np.reshape(self.i2c, (1, 2))

    @staticmethod
    def calculateCAP(data):
        num = 1
        value = 0
        for i in data:
            value += i << 8 * (len(data) - num)
            num += 1

        return value

    def data_handler_acc(self, ctx, data):
        timeStamp = data.contents.epoch
        date = datetime.datetime.fromtimestamp(float(timeStamp) / 1000).strftime('%Y-%m-%d %H:%M:%S:%f')

        self.acc_x.append(parse_value(data).x)
        self.acc_y.append(parse_value(data).y)
        self.acc_z.append(parse_value(data).z)
        self.timeStamp_acc.append(timeStamp)
        self.date_acc.append(date)

    def data_handler_gyro(self, ctx, data):
        timeStamp = data.contents.epoch
        date = datetime.datetime.fromtimestamp(float(timeStamp) / 1000).strftime('%Y-%m-%d %H:%M:%S:%f')

        self.gyro_x.append(parse_value(data).x)
        self.gyro_y.append(parse_value(data).y)
        self.gyro_z.append(parse_value(data).z)
        self.timeStamp_gyro.append(timeStamp)
        self.date_gyro.append(date)


def searchDevice() -> List:
    print("scanning for devices...")
    devices = []

    def handler(result):
        if result.name == "MetaWear":
            devices.append(result.mac)

    BleScanner.set_handler(handler)
    BleScanner.start()
    sleep(2)
    BleScanner.stop()
    return devices


def connectDevice(address):
    print("Connecting to %s..." % (address))
    device = MetaWear(address)
    device.connect()
    print("Device information: " + str(device.info))

    return device


def disconnectDevice(device):
    # Stop the LED
    libmetawear.mbl_mw_led_stop(device.board)
    sleep(1.0)
    libmetawear.mbl_mw_metawearboard_tear_down(device.board)
    sleep(1.0)
    device.disconnect()
    sleep(1.0)
    print("Disconnected")


def turnonLED(mmr):
    # To identify whether this device is communicated or not, LED works
    pattern = LedPattern(repeat_count=Const.LED_REPEAT_INDEFINITELY)
    libmetawear.mbl_mw_led_load_preset_pattern(byref(pattern), LedPreset.BLINK)
    libmetawear.mbl_mw_led_write_pattern(mmr.device.board, byref(pattern), LedColor.GREEN)
    libmetawear.mbl_mw_led_play(mmr.device.board)


def setDevice(mmr):
    libmetawear.mbl_mw_settings_set_connection_parameters(mmr.device.board, 7.5, 7.5, 0, 6000)
    sleep(1.5)

    # Get the signal from i2c communication
    # (device.board address, the size of data(byte), id : Numerical value identifying the data)
    mmr.i2c_signal = libmetawear.mbl_mw_i2c_get_data_signal(mmr.device.board, 3, 0xa)
    # Get the signal from Acceleration
    acc_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(mmr.device.board)
    # get the signal from Gyro
    gyro_signal = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(mmr.device.board)

    # Subscribe the i2c signal
    # (i2c signal value, context(Pointer to additional data for the call back function
    # , Callback function to handle data received from the signal))
    libmetawear.mbl_mw_datasignal_subscribe(mmr.i2c_signal, None, mmr.callback_i2c)

    # Subscribe the acc acc_signal
    # libmetawear.mbl_mw_datasignal_subscribe(acc_signal, None, mmr.callback_acc)

    # Subscribe the gyro gyro acc_signal
    # libmetawear.mbl_mw_datasignal_subscribe(gyro_signal, None, mmr.callback_gyro)

    # device_addr = 7bit(the slave device address) + 1bit(W : 0 / R : 1)
    # Parameter : (device address, register address : CAP DATA REGISTER(0x01))
    # libmetawear.mbl_mw_i2c_write(mmr.device.board, AD7745_ADDR_WRITE, RESET_ADDR, value, 1)

    # Setup the configuration for I2C Communication
    libmetawear.mbl_mw_i2c_write(mmr.device.board, AD7745_ADDR, REG_CAP_SETUP, c_uint8(CAP_SETUP_PROP), 1)
    sleep(0.5)
    libmetawear.mbl_mw_i2c_write(mmr.device.board, AD7745_ADDR, REG_EXC_SETUP, c_uint8(EXC_SETUP_PROP), 1)
    sleep(0.5)
    libmetawear.mbl_mw_i2c_write(mmr.device.board, AD7745_ADDR, REG_CONFIGURATION, c_uint8(CONFIGURATION_PROP), 1)
    sleep(0.5)
    libmetawear.mbl_mw_i2c_write(mmr.device.board, AD7745_ADDR, REG_CAP_DAC_A, c_uint8(CAP_DAC_A_PROP), 1)
    sleep(0.5)


def createTimer(mmr):
    mmr.parameters = I2cReadParameters(device_addr=AD7745_ADDR, register_addr=REG_CAP_DATA)

    e = Event()

    mmr.timer = create_voidp(lambda fn: libmetawear.mbl_mw_timer_create_indefinite(mmr.device.board, 11, 0, None, fn),
                             resource="timer", event=e)
    libmetawear.mbl_mw_event_record_commands(mmr.timer)
    libmetawear.mbl_mw_datasignal_read_with_parameters(mmr.i2c_signal, byref(mmr.parameters))
    create_voidp_int(lambda fn: libmetawear.mbl_mw_event_end_record(mmr.timer, None, fn), event=e)
    print(mmr.timer)
    mmr.timer_id = libmetawear.mbl_mw_timer_get_id(mmr.timer)
    print("id" + str(mmr.timer_id))


def startRecording(mmr):
    # libmetawear.mbl_mw_gyro_bmi160_write_config(mmr.device.board)
    # libmetawear.mbl_mw_acc_enable_acceleration_sampling(mmr.device.board)
    # libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(mmr.device.board)
    # libmetawear.mbl_mw_acc_start(mmr.device.board)
    # libmetawear.mbl_mw_gyro_bmi160_start(mmr.device.board)

    # logger = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(signal, None, fn), resource = "acc_logger")

    # parameter_VT = I2cReadParameters(device_addr = AD7745_ADDR, register_addr=REG_VT_DATA)

    # libmetawear.mbl_mw_datasignal_read_with_parameters(i2c_signal, byref(parameter_VT))
    # libmetawear.mbl_mw_datasignal_read_with_parameters(i2c_signal, byref(parameter_VT))
    # libmetawear.mbl_mw_datasignal_read_with_parameters(i2c_signal, byref(parameter_VT))

    timer = libmetawear.mbl_mw_timer_lookup_id(mmr.device.board, mmr.timer_id)

    libmetawear.mbl_mw_timer_start(timer)


def stopRecording(mmr):
    timer = libmetawear.mbl_mw_timer_lookup_id(mmr.device.board, mmr.timer_id)
    print("id" + str(timer))

    libmetawear.mbl_mw_timer_stop(timer)
    print("complete timer stop")


def writeData(mmr):
    print("writing...")
    people = "practice"
    status = "fixedcap"
    ts = mmr.i2c[1, 0]
    date = datetime.datetime.fromtimestamp(float(ts) / 1000).strftime('%Y_%m_%d')
    time = datetime.datetime.fromtimestamp(float(ts) / 1000).strftime('%H_%M_%S')
    path = "./" + people + "/" + status + "/" + date + "/" + time
    os.makedirs(path, exist_ok=True)

    pd.DataFrame(mmr.i2c, columns=["DATE", "BP"]).to_csv(path + "/bp.csv", header=True, index=False)

    df_new = pd.read_csv(path + '/bp.csv')
    writer = pd.ExcelWriter(path + '/bp.xlsx')
    df_new.to_excel(writer, index=False)
    writer.save()
