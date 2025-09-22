# Data structure to store NAME, number, number pairs
class AddrInfo:
    def __init__(self, addr, bytesize):
        self.addr = addr
        self.bytesize = bytesize

# You can store them in a dict for lookup by name:
ADDR = {
    "DEBUG_MODE": AddrInfo(14, 1),
    "XM_TORQUE_ENABLE": AddrInfo(169, 1),
    "ADDR_X_VEL": AddrInfo(170, 4),
    "ADDR_Y_VEL": AddrInfo(174, 4),
    "ADDR_Z_ANG": AddrInfo(190, 4),
    "AX_TORQUE_ENABLE": AddrInfo(210, 4),
}

print(ADDR["DEBUG_MODE"].addr)  # Outputs: 14
print(ADDR["DEBUG_MODE"].bytesize)  # Outputs: 1

