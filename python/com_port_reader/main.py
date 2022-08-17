import serial
import re
import csv
import pandas as pd
from pathlib import Path

result_csv_filename = "rchi_sin1.csv"
path_to_csv_save = Path("../../matlab/csv/" + result_csv_filename)


def print_hi(name):
   # Use a breakpoint in the code line below to debug your script.
   print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
   print_hi('PyCharm')

   serialPort = serial.Serial(port="COM6", baudrate=250000, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
   if not serialPort.isOpen():
      serialPort.open()

   serialString = ""  # Used to hold data coming over UART

   mstr = "q: 15 was: 45.5 ddI: 0.018"
   words = re.findall('[a-zA-Z]+', mstr)
   print(words)

   mlist = [1, 2, 3, 4, 5]
   df = pd.DataFrame([mlist])
   df = pd.concat([df, pd.DataFrame([mlist])])
   df = pd.concat([df, pd.DataFrame([mlist])])
   print(df)

   is_start = True

   data_frame = pd.DataFrame()
   words = []
   count = 0

   try:
      while 1:
         if serialPort.in_waiting > 0:
            serialString = serialPort.readline()
            com_str = (serialString.decode())[:-1]
            numbers = re.findall(r"[-+]?\d*\.\d+|\d+", com_str)
            # words = re.findall('[a-zA-Z]+', com_str)

            if numbers:
               # print("NUMBERS", numbers)
               if not is_start:
                  if count == len(numbers):
                     data_frame = pd.concat([data_frame, pd.DataFrame([numbers], columns=words)])

               # for i in range(len(numbers)):
               #    print("number", numbers[i])

            # if words:
            #    print("WORDS", words)
               # for i in range(len(words)):
               #    print("word", words[i])

            if is_start:
               is_start = False
               # print("WORDS", words)
               # print("NUMBERS", numbers)

               print(com_str)
               words = re.findall('[a-zA-Z]+', com_str)
               count = len(words)

               data_frame = pd.DataFrame([numbers], columns=words)
               data_frame.columns = words

   except KeyboardInterrupt:
      print("end")
      serialPort.close()
      print(data_frame)
      data_frame.to_csv(path_to_csv_save)
