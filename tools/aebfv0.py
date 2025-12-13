import ctypes
import platform
import sys

# Load the correct library for each platform
def load_library():
    system = platform.system()
    
    if system == "Windows":
        return ctypes.CDLL("aebfv0.dll")
    elif system == "Darwin":  # macOS
        return ctypes.CDLL("./build/libaebfv0.dylib")
    else:  # Linux
        return ctypes.CDLL("./build/libaebfv0.so")

# Load once
lib = load_library()

# Setup function signatures (SAME FOR ALL PLATFORMS)
lib.aebp_is_frame_cplt.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
lib.aebp_is_frame_cplt.restype = ctypes.c_bool

lib.aebp_encode_frame.argtypes = [
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8, ctypes.c_uint16, 
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8
]
lib.aebp_encode_frame.restype = ctypes.c_uint8

lib.aebp_decode_frame.argtypes = [
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint16),
    ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8)
]
lib.aebp_decode_frame.restype = ctypes.c_uint8

# Wrapper functions (unchanged)
def is_frame_complete(frame_data, frame_len):
    return bool(lib.aebp_is_frame_cplt(frame_data, frame_len))

def encode_frame(device_id, service_id, payload):
    output =  (ctypes.c_uint8 * (len(payload) + 7))()
    payload_array = (ctypes.c_uint8 * len(payload))(*payload)
    result = lib.aebp_encode_frame(output, device_id, service_id, 
                                   payload_array , len(payload))
    if result == 0:
        return bytes(output)
    else:
        raise Exception(f"Encoding failed with code {result}")

def decode_frame(frame_data, frame_len):
    device_id = ctypes.c_uint8()
    service_id = ctypes.c_uint16()
    payload_len = ctypes.c_uint8()
    payload = (ctypes.c_uint8 * 245)()
    
    # Convert bytes to uint8 array
    frame_array = (ctypes.c_uint8 * len(frame_data))(*frame_data)

    result = lib.aebp_decode_frame(
        frame_array, frame_len,
        ctypes.byref(device_id), ctypes.byref(service_id),
        payload, ctypes.byref(payload_len)
    )
    
    if result == 0:
        return {
            'device_id': device_id.value,
            'service_id': service_id.value,
            'payload': bytes(payload[:payload_len.value])
        }
    elif result == 1:
        return {'status': 'incomplete'}
    else:
        return {'status': 'corrupted'}