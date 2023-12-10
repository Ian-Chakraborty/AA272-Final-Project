import serial
import time



def send_command(ser):

    #ser.write(0x42)
    ser.write(bytes(b'B'))
    #ser.write(0x57)
    ser.write(bytes(b'W'))

    #ser.write(0x02)
    ser.write(bytes(2))
    #ser.write(0x00)
    ser.write(bytes(0))
    #ser.write(0x00)
    ser.write(bytes(0))
    #ser.write(0x00)
    ser.write(bytes(0))        
    #ser.write(0x01)
    ser.write(bytes(1))         
    #ser.write(0x06)
    ser.write(bytes(6))


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
        
        print('Sending Command')
        send_command(ser)
        print('Listening for Response')
        read_data(ser)
    except KeyboardInterrupt:   # Ctrl+C
        if ser != None:
            print('Closing Port')
            ser.close()