import serial
import time


def read_data(ser):
    while(True):
        while(ser.in_waiting >= 9):
            #print (ser.read())
            if((b'Y' == ser.read()) and ( b'Y' == ser.read())):
                Dist_L = ser.read()
                Dist_H = ser.read()
                Dist_Total = (ord(Dist_H) * 256) + (ord(Dist_L))
                for i in range (0,5):
                    ser.read()
                    print(Dist_Total)

if __name__ == '__main__':
    ser = serial.Serial('/dev/serial0',115200,timeout = 1)
    try:
        print('Starting Script')
        if ser.is_open == False:
            print('Opening Port')
            ser.open()
        else:
            print('Port already open')
        
        print('Listening for Response')
        read_data(ser)
    except KeyboardInterrupt:   # Ctrl+C
        if ser != None:
            print('Closing Port')
            ser.close()