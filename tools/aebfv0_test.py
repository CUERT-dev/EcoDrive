import sys
import os

# Add current directory to path if needed
sys.path.insert(0, os.path.dirname(__file__))

# Import your module
try:
    from aebfv0 import encode_frame, decode_frame, is_frame_complete
    print("✓ Module imported successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

# Test 1: Encode a simple frame
print("\n=== Test 1: Encoding ===")
payload = b"I WANNA CUM INSIDE YOU\0"
print(f"Payload: {payload}")
print(f"Payload length: {len(payload)} bytes")

encoded = encode_frame(device_id=50, service_id=300, payload=payload)
print(f"✓ Encoded frame: {encoded.hex(' ')}")
print(f"  Frame length: {len(encoded)} bytes")

# Test 2: Decode the frame
print("\n=== Test 2: Decoding ===")

result = decode_frame(encoded, len(encoded))

if 'device_id' in result:
    print(f"✓ Decoding successful!")
    print(f"  Device ID: {result['device_id']}")
    print(f"  Service ID: {result['service_id']}")
    print(f"  Payload: {result['payload']}")
    print(f"  Payload matches original: {result['payload'] == payload}")
else:
    print(f"✗ Decoding failed: {result['status']}")
        
print("\n=== All tests completed ===")