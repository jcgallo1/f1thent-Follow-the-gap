import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juangallo/Escritorio/vehicle_nt/proyecto_1erParte/install/reactive_race'
