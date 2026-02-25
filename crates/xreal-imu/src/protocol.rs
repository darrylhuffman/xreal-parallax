use crate::types::RawImuSample;
use glam::Vec3;
use std::collections::VecDeque;
use thiserror::Error;

/// Packet framing markers from the XReal One TCP protocol.
const HEADER: [u8; 6] = [0x28, 0x36, 0x00, 0x00, 0x00, 0x80];
const FOOTER: [u8; 19] = [
    0x00, 0x00, 0x00, 0xcf, 0xf7, 0x53, 0xe3, 0xa5, 0x9b, 0x00, 0x00, 0xdb, 0x34, 0xb6, 0xd7,
    0x82, 0xde, 0x1b, 0x43,
];
const SENSOR_MSG: [u8; 6] = [0x00, 0x40, 0x1f, 0x00, 0x00, 0x40];

/// Bytes to skip after HEADER (session ID).
const SESSION_ID_LEN: usize = 8;
/// Bytes to skip at the start of the payload (timestamp + invariant + static).
const DATA_START_OFFSET: usize = 20;
/// Bytes to trim from the end of the payload (sensor_msg + date_info).
const DATA_END_OFFSET: usize = 26;
/// Size of the IMU data region: 6 x f32 = 24 bytes.
const IMU_DATA_LEN: usize = 24;

#[derive(Debug, Error)]
pub enum ProtocolError {
    #[error("Packet too short after stripping framing")]
    PacketTooShort,
    #[error("Not a sensor data packet (missing SENSOR_MSG marker)")]
    NotSensorData,
}

/// Streaming parser for the XReal One IMU TCP protocol.
///
/// Feed raw TCP bytes via `push_data`, then drain parsed samples via `next_sample`.
pub struct ProtocolParser {
    buffer: VecDeque<u8>,
}

impl ProtocolParser {
    pub fn new() -> Self {
        Self {
            buffer: VecDeque::with_capacity(8192),
        }
    }

    /// Append received bytes to the internal buffer.
    pub fn push_data(&mut self, data: &[u8]) {
        self.buffer.extend(data);
    }

    /// Try to extract the next complete IMU sample from the buffer.
    /// Returns `None` if no complete packet is available yet.
    pub fn next_sample(&mut self) -> Option<Result<RawImuSample, ProtocolError>> {
        // Scan for a complete HEADER...FOOTER frame and extract it.
        let (message, message_end) = {
            let buf = self.buffer.make_contiguous();

            let header_pos = find_pattern(buf, &HEADER)?;
            let search_start = header_pos + HEADER.len();
            if search_start >= buf.len() {
                return None;
            }
            let footer_pos = find_pattern(&buf[search_start..], &FOOTER)?;
            let footer_abs = search_start + footer_pos;
            let end = footer_abs + FOOTER.len();

            (buf[header_pos..end].to_vec(), end)
        };

        // Advance buffer past this message.
        self.buffer.drain(..message_end);

        Some(parse_message(&message))
    }
}

/// Parse a complete message (HEADER to FOOTER inclusive) into an IMU sample.
fn parse_message(message: &[u8]) -> Result<RawImuSample, ProtocolError> {
    // Strip framing:
    //   Front: HEADER (6) + session_id (8) = 14 bytes
    //   Back:  FOOTER (19) + tail (31) = 50 bytes
    let front_strip = HEADER.len() + SESSION_ID_LEN;
    let back_strip = FOOTER.len() + 31;

    if message.len() < front_strip + back_strip {
        return Err(ProtocolError::PacketTooShort);
    }

    let payload = &message[front_strip..message.len() - back_strip];

    // Validate sensor message marker at end of payload.
    if payload.len() < SENSOR_MSG.len() || !payload.ends_with(&SENSOR_MSG) {
        return Err(ProtocolError::NotSensorData);
    }

    // Extract IMU data region.
    if payload.len() < DATA_START_OFFSET + DATA_END_OFFSET {
        return Err(ProtocolError::PacketTooShort);
    }

    let imu_region = &payload[DATA_START_OFFSET..payload.len() - DATA_END_OFFSET];

    if imu_region.len() < IMU_DATA_LEN {
        return Err(ProtocolError::PacketTooShort);
    }

    // Decode 6 little-endian f32 values.
    let f = |offset: usize| -> f32 {
        let bytes: [u8; 4] = imu_region[offset..offset + 4].try_into().unwrap();
        f32::from_le_bytes(bytes)
    };

    // Layout: [gx, gy, gz, az, ay, ax]
    let gx = f(0);
    let gy = f(4);
    let gz = f(8);
    let az = f(12);
    let ay = f(16);
    let ax = f(20);

    Ok(RawImuSample {
        gyro: Vec3::new(gx, gy, gz),
        accel: Vec3::new(ax, ay, az),
    })
}

/// Find the first occurrence of `pattern` in `data`.
fn find_pattern(data: &[u8], pattern: &[u8]) -> Option<usize> {
    data.windows(pattern.len())
        .position(|window| window == pattern)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic packet for testing.
    fn make_test_packet(gx: f32, gy: f32, gz: f32, az: f32, ay: f32, ax: f32) -> Vec<u8> {
        let mut packet = Vec::new();

        // HEADER
        packet.extend_from_slice(&HEADER);
        // Session ID (8 bytes of zeros)
        packet.extend_from_slice(&[0u8; 8]);
        // Payload front padding (DATA_START_OFFSET = 20 bytes)
        packet.extend_from_slice(&[0u8; 20]);
        // IMU data: 6 x f32 LE
        packet.extend_from_slice(&gx.to_le_bytes());
        packet.extend_from_slice(&gy.to_le_bytes());
        packet.extend_from_slice(&gz.to_le_bytes());
        packet.extend_from_slice(&az.to_le_bytes());
        packet.extend_from_slice(&ay.to_le_bytes());
        packet.extend_from_slice(&ax.to_le_bytes());
        // Payload end padding (DATA_END_OFFSET - SENSOR_MSG = 20 bytes)
        packet.extend_from_slice(&[0u8; 20]);
        // SENSOR_MSG marker
        packet.extend_from_slice(&SENSOR_MSG);
        // Tail (31 bytes)
        packet.extend_from_slice(&[0u8; 31]);
        // FOOTER
        packet.extend_from_slice(&FOOTER);

        packet
    }

    #[test]
    fn parse_single_packet() {
        let packet = make_test_packet(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let mut parser = ProtocolParser::new();
        parser.push_data(&packet);

        let sample = parser.next_sample().unwrap().unwrap();
        assert!((sample.gyro.x - 1.0).abs() < 1e-6);
        assert!((sample.gyro.y - 2.0).abs() < 1e-6);
        assert!((sample.gyro.z - 3.0).abs() < 1e-6);
        assert!((sample.accel.x - 6.0).abs() < 1e-6);
        assert!((sample.accel.y - 5.0).abs() < 1e-6);
        assert!((sample.accel.z - 4.0).abs() < 1e-6);

        // No more packets.
        assert!(parser.next_sample().is_none());
    }

    #[test]
    fn parse_fragmented_data() {
        let packet = make_test_packet(0.5, -0.5, 0.1, 9.8, 0.0, 0.0);
        let mid = packet.len() / 2;

        let mut parser = ProtocolParser::new();

        // Feed first half — no complete packet yet.
        parser.push_data(&packet[..mid]);
        assert!(parser.next_sample().is_none());

        // Feed second half — now we can parse.
        parser.push_data(&packet[mid..]);
        let sample = parser.next_sample().unwrap().unwrap();
        assert!((sample.gyro.x - 0.5).abs() < 1e-6);
    }

    #[test]
    fn parse_multiple_packets() {
        let p1 = make_test_packet(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let p2 = make_test_packet(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        let mut parser = ProtocolParser::new();
        parser.push_data(&p1);
        parser.push_data(&p2);

        let s1 = parser.next_sample().unwrap().unwrap();
        assert!((s1.gyro.x - 1.0).abs() < 1e-6);

        let s2 = parser.next_sample().unwrap().unwrap();
        assert!((s2.gyro.x - 2.0).abs() < 1e-6);

        assert!(parser.next_sample().is_none());
    }
}
