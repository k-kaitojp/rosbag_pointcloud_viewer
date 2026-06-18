// Minimal CDR (Common Data Representation) reader for ROS2 serialized messages.
//
// ROS2 (rmw_fastrtps / rmw_cyclonedds) serializes messages using OMG CDR with a
// 4-byte RTPS encapsulation header:
//   byte 0: 0x00
//   byte 1: encapsulation kind (0x00 = CDR_BE, 0x01 = CDR_LE, 0x02/0x03 = PL_CDR)
//   byte 2-3: options
//
// Primitive alignment is computed relative to the *start of the body* (i.e. right
// after the 4-byte encapsulation header), matching the RTPS spec.

const textDecoder = new TextDecoder('utf-8');

export class CdrReader {
  /** @param {Uint8Array} data full serialized message including encapsulation header */
  constructor(data) {
    this.array = data;
    this.view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    const kind = this.view.getUint8(1);
    // 0x01 = CDR_LE, 0x03 = PL_CDR_LE
    this.littleEndian = kind === 1 || kind === 3;
    this.offset = 4; // skip encapsulation header
  }

  align(size) {
    const rel = (this.offset - 4) % size;
    if (rel !== 0) this.offset += size - rel;
  }

  int8() {
    const v = this.view.getInt8(this.offset);
    this.offset += 1;
    return v;
  }

  uint8() {
    const v = this.view.getUint8(this.offset);
    this.offset += 1;
    return v;
  }

  int16() {
    this.align(2);
    const v = this.view.getInt16(this.offset, this.littleEndian);
    this.offset += 2;
    return v;
  }

  uint16() {
    this.align(2);
    const v = this.view.getUint16(this.offset, this.littleEndian);
    this.offset += 2;
    return v;
  }

  int32() {
    this.align(4);
    const v = this.view.getInt32(this.offset, this.littleEndian);
    this.offset += 4;
    return v;
  }

  uint32() {
    this.align(4);
    const v = this.view.getUint32(this.offset, this.littleEndian);
    this.offset += 4;
    return v;
  }

  float32() {
    this.align(4);
    const v = this.view.getFloat32(this.offset, this.littleEndian);
    this.offset += 4;
    return v;
  }

  float64() {
    this.align(8);
    const v = this.view.getFloat64(this.offset, this.littleEndian);
    this.offset += 8;
    return v;
  }

  /** Reads a uint32 sequence/array length prefix. */
  sequenceLength() {
    return this.uint32();
  }

  /** Reads a ROS2 string (uint32 length incl. null terminator, then bytes). */
  string() {
    const length = this.uint32();
    if (length === 0) return '';
    const bytes = this.array.subarray(this.offset, this.offset + length - 1);
    this.offset += length; // includes the trailing null byte
    return textDecoder.decode(bytes);
  }

  /** Returns a view of `length` raw bytes and advances the cursor. */
  uint8Array(length) {
    const out = this.array.subarray(this.offset, this.offset + length);
    this.offset += length;
    return out;
  }
}
