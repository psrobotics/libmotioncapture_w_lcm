import lcm
from exlcm import motion_t

def my_handler(channel, data):
    msg = motion_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   latency   = %s" % str(msg.latency))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.orientation))
    print("   rb_name        = '%s'" % msg.rb_name)
    print("   enabled     = %s" % str(msg.enabled))
    print("")

lc = lcm.LCM()
subscription = lc.subscribe("VICON_LCM", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass