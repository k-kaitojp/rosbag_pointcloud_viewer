// Verifies the lazy SQLite reader against a real database produced by sql.js,
// using the same schema rosbag2 uses. Exercises interior b-tree pages and
// overflow-page chains (large BLOBs), then round-trips a PointCloud2 message.

import initSqlJs from 'sql.js';
import { SqliteReader } from '../src/sqlite/reader.js';
import { decodePointCloud2, extractPoints } from '../src/pointcloud2.js';

function assert(cond, msg) {
  if (!cond) {
    console.error('❌ ' + msg);
    process.exit(1);
  }
}

// Build a synthetic PointCloud2 CDR message with N points (FLOAT32 x,y,z,intensity).
function buildPointCloud2(n, frameId) {
  const dv = new DataView(new ArrayBuffer(256 + n * 16));
  let off = 0;
  const align = (k) => {
    const rel = (off - 4) % k;
    if (rel) off += k - rel;
  };
  const u8 = (v) => {
    dv.setUint8(off, v);
    off += 1;
  };
  const i32 = (v) => {
    align(4);
    dv.setInt32(off, v, true);
    off += 4;
  };
  const u32 = (v) => {
    align(4);
    dv.setUint32(off, v, true);
    off += 4;
  };
  const str = (s) => {
    u32(s.length + 1);
    for (const c of s) u8(c.charCodeAt(0));
    u8(0);
  };
  dv.setUint8(0, 0);
  dv.setUint8(1, 1); // CDR_LE
  off = 4;
  i32(7);
  u32(8);
  str(frameId);
  u32(1);
  u32(n);
  u32(4);
  const field = (name, offset, dt) => {
    str(name);
    u32(offset);
    u8(dt);
    u32(1);
  };
  field('x', 0, 7);
  field('y', 4, 7);
  field('z', 8, 7);
  field('intensity', 12, 7);
  u8(0);
  u32(16);
  u32(16 * n);
  u32(16 * n);
  for (let i = 0; i < n; i++) {
    dv.setFloat32(off, i, true);
    off += 4;
    dv.setFloat32(off, i + 0.5, true);
    off += 4;
    dv.setFloat32(off, i + 1, true);
    off += 4;
    dv.setFloat32(off, i * 0.1, true);
    off += 4;
  }
  u8(1); // is_dense
  return new Uint8Array(dv.buffer, 0, off);
}

const SQL = await initSqlJs();
const db = new SQL.Database();
db.run(`
  CREATE TABLE topics(
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    type TEXT NOT NULL,
    serialization_format TEXT NOT NULL,
    offered_qos_profiles TEXT NOT NULL);
  CREATE TABLE messages(
    id INTEGER PRIMARY KEY,
    topic_id INTEGER NOT NULL,
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);
`);

const topics = [
  { id: 1, name: '/scan_points', type: 'sensor_msgs/msg/PointCloud2' },
  { id: 2, name: '/tf', type: 'tf2_msgs/msg/TFMessage' },
  { id: 3, name: '/lidar/points', type: 'sensor_msgs/msg/PointCloud2' },
];
const insT = db.prepare(
  'INSERT INTO topics VALUES (?, ?, ?, ?, ?)',
);
for (const t of topics) insT.run([t.id, t.name, t.type, 'cdr', '']);
insT.free();

// Insert many messages. PointCloud2 messages are large enough (thousands of
// points) to spill onto overflow pages; enough rows to force interior pages.
const insM = db.prepare('INSERT INTO messages VALUES (?, ?, ?, ?)');
const expected = new Map(); // messageId -> { topicId, bytes }
let msgId = 1;
const baseTs = 1_700_000_000_000_000_000; // ns since epoch (> 2^53)
for (let i = 0; i < 30; i++) {
  // alternate topics, vary sizes (some big -> overflow, some tiny -> inline)
  const topicId = i % 3 === 1 ? 2 : i % 2 === 0 ? 1 : 3;
  const npts = topicId === 2 ? 2 : 500 + i * 50; // topic 2 isn't a cloud
  const bytes =
    topicId === 2
      ? new Uint8Array([1, 2, 3, 4]) // dummy non-cloud payload
      : buildPointCloud2(npts, `frame_${i}`);
  const ts = baseTs + i * 100_000_000;
  insM.run([msgId, topicId, ts, bytes]);
  expected.set(msgId, { topicId, bytes });
  msgId++;
}
insM.free();

const fileBytes = db.export(); // Uint8Array image of the DB
db.close();
console.log(`Generated test DB: ${(fileBytes.length / 1024).toFixed(1)} KB`);

// --- Read it back with the lazy reader (buffer-backed source) ---
const source = {
  size: fileBytes.length,
  async read(offset, length) {
    return fileBytes.subarray(offset, Math.min(offset + length, fileBytes.length));
  },
};

const reader = new SqliteReader(source);
await reader.init();
console.log(`page size: ${reader.pageSize}, pages: ${reader.pageCount}`);

const schema = await reader.loadSchema();
assert(schema.has('topics') && schema.has('messages'), 'schema has topics+messages');

// Read topics
const readTopics = [];
await reader.walkTable(
  schema.get('topics').rootpage,
  (buf, p, rowid, len) => {
    const rec = reader.localRecord(buf, p, len);
    // body: [id(NULL), name, type, serialization_format, offered_qos_profiles]
    readTopics.push({ id: rowid, name: rec.text(1), type: rec.text(2) });
  },
);
readTopics.sort((a, b) => a.id - b.id);
assert(readTopics.length === 3, `topics count = 3 (got ${readTopics.length})`);
assert(readTopics[0].name === '/scan_points', 'topic 1 name');
assert(
  readTopics[2].type === 'sensor_msgs/msg/PointCloud2',
  'topic 3 type',
);
console.log('✓ topics read correctly');

// Scan messages -> bucket PointCloud2 frames (topics 1 and 3)
const pcIds = new Set([1, 3]);
const frames = new Map([
  [1, []],
  [3, []],
]);
await reader.walkTable(
  schema.get('messages').rootpage,
  (buf, p, rowid, len) => {
    const rec = reader.localRecord(buf, p, len);
    // body: [id(NULL), topic_id, timestamp, data]
    const topicId = Number(rec.int(1));
    if (pcIds.has(topicId)) {
      frames.get(topicId).push({ id: rowid, timestamp: Number(rec.int(2)) });
    }
  },
);

const expCount1 = [...expected.values()].filter((e) => e.topicId === 1).length;
const expCount3 = [...expected.values()].filter((e) => e.topicId === 3).length;
assert(
  frames.get(1).length === expCount1,
  `topic 1 frame count ${frames.get(1).length} === ${expCount1}`,
);
assert(
  frames.get(3).length === expCount3,
  `topic 3 frame count ${frames.get(3).length} === ${expCount3}`,
);
console.log(
  `✓ frame index correct (topic1=${expCount1}, topic3=${expCount3})`,
);

// Fetch every PointCloud2 message by rowid and compare bytes exactly.
let checked = 0;
for (const [id, info] of expected) {
  if (!pcIds.has(info.topicId)) continue;
  const payload = await reader.getRowPayload(schema.get('messages').rootpage, id);
  const rec = reader.parse(payload);
  const data = rec.bytes(3);
  assert(
    data.length === info.bytes.length,
    `msg ${id} byte length ${data.length} === ${info.bytes.length}`,
  );
  for (let k = 0; k < data.length; k++) {
    if (data[k] !== info.bytes[k]) {
      assert(false, `msg ${id} byte mismatch at ${k}`);
    }
  }
  checked++;
}
console.log(`✓ ${checked} BLOBs round-tripped byte-for-byte (incl. overflow)`);

// End-to-end: decode one fetched message as PointCloud2.
const firstId = [...expected.keys()].find((id) => expected.get(id).topicId === 1);
const payload = await reader.getRowPayload(schema.get('messages').rootpage, firstId);
const pc = decodePointCloud2(reader.parse(payload).bytes(3));
const ex = extractPoints(pc);
assert(pc.frame_id.startsWith('frame_'), 'decoded frame_id');
assert(ex.count > 0, 'extracted points > 0');
console.log(
  `✓ decoded PointCloud2: frame_id=${pc.frame_id}, points=${ex.count}`,
);

console.log('\n✅ SQLITE READER TEST PASSED');
