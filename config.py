mode = "tag"
pin_irq = 16
pin_cs = 8
pin_rst = 12
eid = "7D:00:22:EA:82:60:3B:00"
pan = 0xdeca

# Tag specific
anchor_list = [b"\x0a\x3b", b"\x0b\x3b", b"\x0c\x3b", b"\x0d\x3b"]
anchor_positions = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]]
logfile = "/home/pi/uwb.log"
rxrfto_limit = 1
tries_limit = 10