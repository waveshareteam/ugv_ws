import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rocketsky/Library/ugv_ws/install/ugv_chat_ai'
