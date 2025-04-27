import json
from serial import Serial

ser = Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.1)
ser.close() # To set parameters

# Change settings for Arduino default Serial.begin:
ser.baudrate=115200
ser.port='/dev/ttyUSB0'
ser.bytesize=8
ser.parity='N'
ser.stopbits=1
ser.timeout=0.1
ser.open()
ser.reset_input_buffer()

while True:
    try:
        N = ser.in_waiting        
        data = ser.readline() 
        data = data.decode('utf-8').strip() # Strip any leading/trailing whitespace
        
        if data and data.startswith('{'): # Only process JSON data
            print(f"data: {data}")
            f = json.loads(data)
            
            dis = {} 
            for i in f['links']:
                if int(i["A"])==1786:
                    dis["r1"]=float(i["R"])
                if int(i["A"])==1787:
                    dis["r2"]=float(i["R"])
                if int(i["A"])==1788:
                    dis["r3"]=float(i["R"])
                if int(i["A"])==1789:
                    dis["r4"]=float(i["R"])
            print(f"dis: {dis}")
            print("***")
            
    except Exception as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        ser.close()
        break

    # print(f) 