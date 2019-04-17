import serial
import time
import threading
port = serial.Serial("/dev/serial0", baudrate=9600, timeout=3.0)

def readchar(port=port):
    return port.read();

def get_value(port=serial.Serial("/dev/serial0", baudrate=9600, timeout=3.0)):
    val = port.read()
    value = int.from_bytes(val, byteorder='little')
    return value

def cont_get_value(uart_val, uart_lock= threading.Lock()):
    while(True):
        val = port.read()
        #print(val)
        val = int.from_bytes(val, byteorder='little')
        value = uart_val[0] | val
        #print(value)
        uart_lock.acquire()
        uart_val[:] = [value]
        uart_lock.release()

def main():
    uart_val =[0x00]
    uart_t = threading.Thread(target=cont_get_value, args=(uart_val,)) 
    uart_t.daemon = True
    uart_t.start()
    

    
    while True:
        time.sleep(.23)
        value = uart_val[0] 
       # value = int.from_bytes(value, byteorder='little')
        if ((value & (0x80)) == 0x80):
            print("1-axis U")
        if ((value & (0x40)) == 0x40):
            print("1-axis D")
        if ((value & (0x20)) == 0x20):
            print("0-axis U")
        if ((value & (0x10)) == 0x10):
            print("0-axis D")
        if ((value & (0x08)) == 0x08):
            print("select")
        if ((value & 0x04) == 0x04):
            print("Green")
        if ((value & 0x02) == 0x02):
            print("Red")
        if ((value & 0x01) == 0x01):
            print("Yellow")
        uart_val[:] = [0] 
    print('\n')

if __name__ == "__main__":
    main()
