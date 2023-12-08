import random
import string

f = open("usb_benchmark.txt", "w")
f.write("echo state.upTime\r\n")

for i in range(10000):
    comment = ''.join(random.choices(string.ascii_uppercase +
                                string.digits, k=128))
    f.write(";{}\r\n".format(comment))

f.write("echo state.upTime\r\n")
f.close()