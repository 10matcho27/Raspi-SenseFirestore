import firebase_admin
from firebase_admin import firestore
#from google.cloud import firestore
from firebase_admin import credentials
import bme680
import time
import smbus
import datetime

def firebase_init():
    cred = credentials.Certificate("/home/pi/Desktop/Firebase/firebase-adminsdk.json")
    firebase_admin.initialize_app(cred)
    db = firestore.client()
    return db

def bme680_init():
    try:
        sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
    except (RuntimeError, IOError):
        sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)
    print('Calibration data:')
    for name in dir(sensor.calibration_data):

        if not name.startswith('_'):
            value = getattr(sensor.calibration_data, name)

            if isinstance(value, int):
                print('{}: {}'.format(name, value))

    # These oversampling settings can be tweaked to
    # change the balance between accuracy and noise in
    # the data.

    sensor.set_humidity_oversample(bme680.OS_8X)
    sensor.set_pressure_oversample(bme680.OS_16X)
    sensor.set_temperature_oversample(bme680.OS_16X)
    # sensor.set_temperature_oversample(bme680.OS_1X)
    sensor.set_filter(bme680.FILTER_SIZE_3)
    sensor.set_gas_status(bme680.DISABLE_GAS_MEAS)
    # sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)

    print('\n\nInitial reading:')
    for name in dir(sensor.data):
        value = getattr(sensor.data, name)

        if not name.startswith('_'):
            print('{}: {}'.format(name, value))
    return sensor

def TSL2572_init():
    i2c = smbus.SMBus(1)
    time.sleep(1)
    #TSL2572 Register Set
    TSL2572_ADR      = 0x39
    TSL2572_COMMAND  = 0x80
    TSL2572_TYPE_REP = 0x00
    TSL2572_TYPE_INC = 0x20
    TSL2572_ALSIFC   = 0x66

    TSL2572_SAI   = 0x40
    TSL2572_AIEN  = 0x10
    TSL2572_WEN   = 0x80
    TSL2572_AEN   = 0x02
    TSL2572_PON   = 0x01

    TSL2572_ENABLE   = 0x00
    TSL2572_ATIME    = 0x01
    TSL2572_WTIME    = 0x03
    TSL2572_AILTL    = 0x04
    TSL2572_AILTH    = 0x05
    TSL2572_AIHTL    = 0x06
    TSL2572_AIHTH    = 0x07
    TSL2572_PRES     = 0x0C
    TSL2572_CONFIG   = 0x0D
    TSL2572_CONTROL  = 0x0F
    TSL2572_ID       = 0x12
    TSL2572_STATUS   = 0x13
    TSL2572_C0DATA   = 0x14
    TSL2572_C0DATAH  = 0x15
    TSL2572_C1DATA   = 0x16
    TSL2572_C1DATAH  = 0x17

    #TSL2572 setings
    atime = 0xC0
    gain = 0.24
    initStat = 0

    dat = i2c.read_i2c_block_data(TSL2572_ADR, TSL2572_COMMAND | TSL2572_TYPE_INC | TSL2572_ID, 1)
    if (dat!=[0x34]) :
        #check TSL2572 ID
        initStat = -1
    # i2c.write_byte_data(TSL2572_ADR,reg,dat)
    i2c.write_byte_data(TSL2572_ADR, TSL2572_COMMAND | TSL2572_TYPE_INC | TSL2572_CONTROL, 0x00)
    i2c.write_byte_data(TSL2572_ADR, TSL2572_COMMAND | TSL2572_TYPE_INC | TSL2572_CONFIG, 0x00)
    i2c.write_byte_data(TSL2572_ADR, TSL2572_COMMAND | TSL2572_TYPE_INC | TSL2572_ATIME, atime)
    i2c.write_byte_data(TSL2572_ADR, TSL2572_COMMAND | TSL2572_TYPE_INC | TSL2572_ENABLE,TSL2572_AEN | TSL2572_PON)
    
    return [i2c, atime, gain], [TSL2572_ADR, TSL2572_COMMAND, TSL2572_TYPE_INC, TSL2572_C0DATA]

def getTSL2572adc(values, commands):
    cpl = 0.0
    lux1 = 0.0
    lux2 = 0.0
    adc = []
    i2c = values[0]
    atime = values[1]
    gain = values[2]
    
    dat = values[0].read_i2c_block_data(commands[0], commands[1] | commands[2] | commands[3], 4)
    adc.append((dat[1] << 8) | dat[0])
    adc.append((dat[3] << 8) | dat[2])
    
    cpl = (2.73 * (256 - atime) * gain)/(60.0)
    lux1 = ((adc[0] * 1.00) - (adc[1] * 1.87)) / cpl
    lux2 = ((adc[0] * 0.63) - (adc[1] * 1.00)) / cpl
    if ((lux1 <= 0) and (lux2 <= 0)) :
        print("0 Lx")
    elif (lux1 > lux2) :
        return lux1
    elif (lux1 < lux2) :
        return lux2
    return lux1

def getBME680adc(sensor):
    if sensor.get_sensor_data():
        output = [sensor.data.temperature, sensor.data.pressure, sensor.data.humidity]
    return output

def firebase_push_data(db, output, doc_name):
    # tim = time.strftime('%Y,%m,%d,%H:%M:%S')
    dt = str(datetime.datetime.now())
    doc_ref_bme = db.collection("BME680").document(dt)
    doc_ref_bme.set({"Temperature": {output[0]}, "Pressure": {output[1]}, "Humidity": {output[2]}, "Timestamp": firestore.SERVER_TIMESTAMP})
    
    doc_ref_tsl = db.collection("TSL2572").document(dt)
    doc_ref_tsl.set({"Lux": {output[3]}, "Timestamp": firestore.SERVER_TIMESTAMP})
    doc_name.append(dt)
    # print(dt)
    # print(doc_name)

    return doc_name

def firebase_del_doc(db, count, col_name, name):
    count = count - 1
    for col in col_name:
        res_del = db.collection(col).document(name).delete()
    # res_del_bme = db.collection("BME680").document(name).delete()
    # res_del_tsl = db.collection("TSL2572").document(name).delete()
    return count

def firebase_push_data_daily_ave(db, output, dt):
    doc_ref = db.collection(str(dt.month)).document(str(dt.day))
    doc_ref.set({"Temperature": {output[0]}, "Pressure": {output[1]}, "Humidity": {output[2]}, "Lux": {output[3]}, "Timestamp": str(dt.year) + "/" + str(dt.month) + "/" + str(dt.day)})

if __name__ == '__main__':
    sensor = bme680_init()
    values, commands = TSL2572_init()
    db = firebase_init()
    push_count = 0
    doc_name = []
    sum_data = [0, 0, 0, 0]
    loop_num = 0
    prev_day = datetime.datetime.now()
    
    while(True): #main loop
        output = getBME680adc(sensor)
        output.append(getTSL2572adc(values, commands))
        print("Now: " + str(output))
        # print(time.strftime('%Y,%m,%d,%H:%M:%S'))
        doc_name = firebase_push_data(db, output, doc_name)
        push_count = push_count + 1
        if(push_count > 100):
            push_count = firebase_del_doc(db, push_count, ["BME680", "TSL2572"], doc_name.pop(0))
        sum_data = list(map(sum, zip(sum_data, output)))
        loop_num += 1
        print("Today's current average: " + str(list(map(lambda x: x / loop_num, sum_data))))
        if(prev_day.day != datetime.datetime.now().day):
            firebase_push_data_daily_ave(db, list(map(lambda x: x / loop_num, sum_data)), prev_day)
            sum_data = [0, 0, 0, 0]
            loop_num = 0
            prev_day = datetime.datetime.now()
        time.sleep(48) # store about 2h's data [(48sec * 150times) / (3600sec/1h) = 2h]
    

#https://firebase.google.com/docs/database/admin/save-data?hl=ja
#https://firebase.google.com/docs/database/web/read-and-write?hl=ja
#https://firebase.google.com/docs/firestore/query-data/get-data?hl=ja
