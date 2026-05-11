# A validated RTCM frame with metadata and raw bytes.

std_msgs/Header header
uint16 message_type
uint16 payload_length
uint32 frame_length
uint8[] data
