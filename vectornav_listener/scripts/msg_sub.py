import lcm
from ins import ins_t

def my_handler(channel, data):
    msg = ins_t.decode(data)
    print("Received message on channel \"%s\"" % channel)

lc = lcm.LCM()
subscription = lc.subscribe("loc-ins", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass