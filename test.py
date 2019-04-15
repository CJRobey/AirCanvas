from dust import *
import time

model = dust('10.138.232.207')

print(model.exportAsObj(1))
time.sleep(2)
model.close()
