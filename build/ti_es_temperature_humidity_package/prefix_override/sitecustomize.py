import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david/tinlabs_project/src/ti_es_temperature_humidity_package/install/ti_es_temperature_humidity_package'
