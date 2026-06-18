// Reads ROS2 rosbag2 SQLite3 storage (.db3) — the default format in ROS2 Humble.
//
// rosbag2 sqlite schema (relevant tables):
//   topics(id, name, type, serialization_format, offered_qos_profiles, ...)
//   messages(id, topic_id, timestamp, data BLOB)

import initSqlJs from 'sql.js';
import sqlWasmUrl from 'sql.js/dist/sql-wasm.wasm?url';

let SQL = null;

async function getSql() {
  if (!SQL) {
    SQL = await initSqlJs({ locateFile: () => sqlWasmUrl });
  }
  return SQL;
}

const POINTCLOUD2_TYPE = 'sensor_msgs/msg/PointCloud2';

export class Db3Source {
  constructor(db) {
    this.db = db;
  }

  static async open(arrayBuffer) {
    const sql = await getSql();
    const db = new sql.Database(new Uint8Array(arrayBuffer));
    return new Db3Source(db);
  }

  /** Returns all topics whose type is PointCloud2: [{ id, name, type, count }] */
  listPointCloudTopics() {
    const topics = [];
    const stmt = this.db.prepare(
      'SELECT id, name, type FROM topics WHERE type = :t ORDER BY name',
    );
    stmt.bind({ ':t': POINTCLOUD2_TYPE });
    while (stmt.step()) {
      const row = stmt.getAsObject();
      topics.push({ id: row.id, name: row.name, type: row.type });
    }
    stmt.free();

    for (const t of topics) {
      const c = this.db.prepare(
        'SELECT COUNT(*) AS n FROM messages WHERE topic_id = :id',
      );
      c.bind({ ':id': t.id });
      c.step();
      t.count = c.getAsObject().n;
      c.free();
    }
    return topics;
  }

  /** Returns frame index for a topic: [{ id, timestamp }] sorted by time. */
  listFrames(topicId) {
    const frames = [];
    const stmt = this.db.prepare(
      'SELECT id, timestamp FROM messages WHERE topic_id = :id ORDER BY timestamp ASC',
    );
    stmt.bind({ ':id': topicId });
    while (stmt.step()) {
      const row = stmt.getAsObject();
      frames.push({ id: row.id, timestamp: row.timestamp });
    }
    stmt.free();
    return frames;
  }

  /** Returns the raw serialized message bytes for a frame (from listFrames). */
  getFrameData(_topicId, frame) {
    const stmt = this.db.prepare('SELECT data FROM messages WHERE id = :id');
    stmt.bind({ ':id': frame.id });
    let data = null;
    if (stmt.step()) {
      data = stmt.getAsObject().data; // Uint8Array
    }
    stmt.free();
    return data;
  }

  close() {
    this.db.close();
  }
}
