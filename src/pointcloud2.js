// Decoding of sensor_msgs/msg/PointCloud2 from CDR-serialized ROS2 messages,
// and extraction of renderable point positions + colors.

import { CdrReader } from './cdr.js';

// sensor_msgs/PointField datatype constants
export const PF = {
  INT8: 1,
  UINT8: 2,
  INT16: 3,
  UINT16: 4,
  INT32: 5,
  UINT32: 6,
  FLOAT32: 7,
  FLOAT64: 8,
};

export const PF_NAME = {
  1: 'INT8',
  2: 'UINT8',
  3: 'INT16',
  4: 'UINT16',
  5: 'INT32',
  6: 'UINT32',
  7: 'FLOAT32',
  8: 'FLOAT64',
};

/**
 * Decode a serialized sensor_msgs/msg/PointCloud2 message.
 * @param {Uint8Array} data CDR message (with encapsulation header)
 */
export function decodePointCloud2(data) {
  const r = new CdrReader(data);

  // std_msgs/Header
  const sec = r.int32();
  const nanosec = r.uint32();
  const frame_id = r.string();

  const height = r.uint32();
  const width = r.uint32();

  const fieldCount = r.sequenceLength();
  const fields = [];
  for (let i = 0; i < fieldCount; i++) {
    const name = r.string();
    const offset = r.uint32();
    const datatype = r.uint8();
    const count = r.uint32();
    fields.push({ name, offset, datatype, count });
  }

  const is_bigendian = r.uint8() !== 0;
  const point_step = r.uint32();
  const row_step = r.uint32();

  const dataLen = r.sequenceLength();
  const cloudData = r.uint8Array(dataLen);

  const is_dense = r.uint8() !== 0;

  return {
    stamp: { sec, nanosec },
    frame_id,
    height,
    width,
    fields,
    is_bigendian,
    point_step,
    row_step,
    data: cloudData,
    is_dense,
  };
}

function readValue(view, byteOffset, datatype, littleEndian) {
  switch (datatype) {
    case PF.INT8:
      return view.getInt8(byteOffset);
    case PF.UINT8:
      return view.getUint8(byteOffset);
    case PF.INT16:
      return view.getInt16(byteOffset, littleEndian);
    case PF.UINT16:
      return view.getUint16(byteOffset, littleEndian);
    case PF.INT32:
      return view.getInt32(byteOffset, littleEndian);
    case PF.UINT32:
      return view.getUint32(byteOffset, littleEndian);
    case PF.FLOAT32:
      return view.getFloat32(byteOffset, littleEndian);
    case PF.FLOAT64:
      return view.getFloat64(byteOffset, littleEndian);
    default:
      return NaN;
  }
}

/**
 * Extract point positions and per-field scalar data from a decoded PointCloud2.
 * Non-finite points (NaN/Inf) are dropped.
 *
 * @returns {{
 *   positions: Float32Array,   // length = count*3
 *   count: number,
 *   bounds: {min:[number,number,number], max:[number,number,number], center:[number,number,number], size:number},
 *   scalars: Record<string, Float32Array>, // numeric field -> per-point value (e.g. intensity)
 *   rgb: Float32Array|null     // length = count*3, normalized 0..1, if an rgb/rgba field exists
 * }}
 */
export function extractPoints(pc) {
  const { data, point_step, width, height, fields, is_bigendian } = pc;
  const littleEndian = !is_bigendian;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);

  const totalPoints =
    width * height > 0 ? width * height : Math.floor(data.length / point_step);

  const fieldMap = {};
  for (const f of fields) fieldMap[f.name] = f;

  const fx = fieldMap.x;
  const fy = fieldMap.y;
  const fz = fieldMap.z;
  if (!fx || !fy || !fz) {
    throw new Error(
      `PointCloud2 is missing x/y/z fields (found: ${fields
        .map((f) => f.name)
        .join(', ')})`,
    );
  }

  // Scalar fields worth coloring by (intensity, ring, etc.)
  const scalarFields = fields.filter(
    (f) => !['x', 'y', 'z', 'rgb', 'rgba'].includes(f.name),
  );
  // Packed color field (PCL convention: float/uint32 holding 0x00RRGGBB / 0xAARRGGBB)
  const colorField = fieldMap.rgb || fieldMap.rgba || null;

  const positions = new Float32Array(totalPoints * 3);
  const scalars = {};
  for (const f of scalarFields) scalars[f.name] = new Float32Array(totalPoints);
  const rgb = colorField ? new Float32Array(totalPoints * 3) : null;

  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];

  let count = 0;
  for (let i = 0; i < totalPoints; i++) {
    const base = i * point_step;
    if (base + point_step > data.length) break;

    const x = readValue(view, base + fx.offset, fx.datatype, littleEndian);
    const y = readValue(view, base + fy.offset, fy.datatype, littleEndian);
    const z = readValue(view, base + fz.offset, fz.datatype, littleEndian);

    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      continue;
    }

    const o = count * 3;
    positions[o] = x;
    positions[o + 1] = y;
    positions[o + 2] = z;

    if (x < min[0]) min[0] = x;
    if (y < min[1]) min[1] = y;
    if (z < min[2]) min[2] = z;
    if (x > max[0]) max[0] = x;
    if (y > max[1]) max[1] = y;
    if (z > max[2]) max[2] = z;

    for (const f of scalarFields) {
      scalars[f.name][count] = readValue(
        view,
        base + f.offset,
        f.datatype,
        littleEndian,
      );
    }

    if (rgb) {
      // Read the 4 color bytes as a uint32 regardless of declared datatype.
      const packed = view.getUint32(base + colorField.offset, littleEndian);
      rgb[o] = ((packed >> 16) & 0xff) / 255;
      rgb[o + 1] = ((packed >> 8) & 0xff) / 255;
      rgb[o + 2] = (packed & 0xff) / 255;
    }

    count++;
  }

  // Trim to actual valid count
  const trimmedPositions = positions.subarray(0, count * 3);
  const trimmedScalars = {};
  for (const name of Object.keys(scalars)) {
    trimmedScalars[name] = scalars[name].subarray(0, count);
  }
  const trimmedRgb = rgb ? rgb.subarray(0, count * 3) : null;

  if (count === 0) {
    min[0] = min[1] = min[2] = 0;
    max[0] = max[1] = max[2] = 0;
  }
  const center = [
    (min[0] + max[0]) / 2,
    (min[1] + max[1]) / 2,
    (min[2] + max[2]) / 2,
  ];
  const size = Math.max(
    max[0] - min[0],
    max[1] - min[1],
    max[2] - min[2],
    0.001,
  );

  return {
    positions: trimmedPositions,
    count,
    bounds: { min, max, center, size },
    scalars: trimmedScalars,
    rgb: trimmedRgb,
  };
}
