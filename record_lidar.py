import pigpio
import time
import datetime
import csv
import sys

RX = 17
pi = pigpio.pi()
pi.set_mode(RX, pigpio.INPUT)
pigpio.exceptions = False
pi.bb_serial_read_close(RX)
pi.bb_serial_read_open(RX, 115200) 

data_file = sys.argv[1]

pi = pigpio.pi()

pi.set_mode(RX, pigpio.INPUT)
pigpio.exceptions = False
pi.bb_serial_read_close(RX)
pi.bb_serial_read_open(RX, 115200) 




# Collect data for 30 seconds
data = []
t_end = time.time() + 60
print('Starting 60s Data Collection')
while time.time() < t_end:
    time.sleep(0.05)	#change the value if needed
    (count, recv) = pi.bb_serial_read(RX)
    now = datetime.datetime.now()
    if count > 8:
      for i in range(0, count-9):
        if recv[i] == 89 and recv[i+1] == 89: # 0x59 is 89
          checksum = 0
          for j in range(0, 8):
            checksum = checksum + recv[i+j]
            checksum = checksum % 256
          if checksum == recv[i+8]:
            distance = recv[i+2] + recv[i+3] * 256
            strength = recv[i+4] + recv[i+5] * 256
            # if distance <= 1200 and strength < 2000:
            data.append([str(now.time()),str(distance), str(strength)])

pi.bb_serial_read_close(RX)
pi.stop()

# Write to CSV File
with open(data_file, 'w', newline="") as file:
    csvwriter = csv.writer(file) 
    csvwriter.writerows(data) 

print('Saved data to ' + data_file)