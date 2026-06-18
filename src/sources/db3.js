// Reads ROS2 rosbag2 SQLite3 storage (.db3) — the default format in ROS2 Humble.
//
// Uses a lazy, page-on-demand SQLite reader (see ../sqlite/reader.js) so that
// multi-GB bags can be opened in the browser without loading the whole file
// into memory (sql.js would copy the entire DB into the WASM heap and fail).
//
// rosbag2 sqlite schema (relevant tables):
//   topics(id, name, type, serialization_format, ...)
//   messages(id, topic_id, timestamp, data BLOB)

import { SqliteReader, fileSource } from '../sqlite/reader.js';

const POINTCLOUD2_TYPE = 'sensor_msgs/msg/PointCloud2';

export class Db3Source {
  constructor(reader, { topics, framesByTopic, messagesRoot }) {
    this.reader = reader;
    this.topics = topics; // [{ id, name, type }]
    this.framesByTopic = framesByTopic; // Map<topicId, [{ id, timestamp }]>
    this.messagesRoot = messagesRoot;
  }

  static async open(file, { onProgress } = {}) {
    const reader = new SqliteReader(fileSource(file));
    await reader.init();

    const schema = await reader.loadSchema();
    const topicsTable = schema.get('topics');
    const messagesTable = schema.get('messages');
    if (!topicsTable || !messagesTable) {
      throw new Error(
        'This .db3 does not look like a rosbag2 database (missing topics/messages tables).',
      );
    }

    // Read the (small) topics table fully.
    const topics = [];
    await reader.walkTable(
      topicsTable.rootpage,
      (buf, payloadStart, rowid, payloadLen) => {
        const rec = reader.localRecord(buf, payloadStart, payloadLen);
        // `id INTEGER PRIMARY KEY` is the rowid; SQLite stores a NULL
        // placeholder for it in the record, so body columns are shifted:
        // [id(NULL), name, type, serialization_format, ...]
        topics.push({ id: rowid, name: rec.text(1), type: rec.text(2) });
      },
    );

    // Only index frames for PointCloud2 topics.
    const pcTopicIds = new Set(
      topics.filter((t) => t.type === POINTCLOUD2_TYPE).map((t) => t.id),
    );
    const framesByTopic = new Map();
    for (const id of pcTopicIds) framesByTopic.set(id, []);

    if (pcTopicIds.size > 0) {
      // Scan the messages b-tree once, reading only the inline portion of each
      // cell (topic_id + timestamp live at the start; the data BLOB, which may
      // overflow, is skipped here).
      await reader.walkTable(
        messagesTable.rootpage,
        (buf, payloadStart, rowid, payloadLen) => {
          const rec = reader.localRecord(buf, payloadStart, payloadLen);
          // messages body: [id(NULL), topic_id, timestamp, data]
          const topicId = Number(rec.int(1));
          const frames = framesByTopic.get(topicId);
          if (frames) {
            frames.push({ id: rowid, timestamp: Number(rec.int(2)) });
          }
        },
        onProgress,
      );
    }

    // Frames are stored by ascending rowid, which is recording order.
    for (const frames of framesByTopic.values()) {
      frames.sort((a, b) => a.id - b.id);
    }

    return new Db3Source(reader, {
      topics,
      framesByTopic,
      messagesRoot: messagesTable.rootpage,
    });
  }

  listPointCloudTopics() {
    return this.topics
      .filter((t) => t.type === POINTCLOUD2_TYPE)
      .map((t) => ({
        id: t.id,
        name: t.name,
        type: t.type,
        count: (this.framesByTopic.get(t.id) || []).length,
      }))
      .sort((a, b) => a.name.localeCompare(b.name));
  }

  listFrames(topicId) {
    return this.framesByTopic.get(topicId) || [];
  }

  async getFrameData(_topicId, frame) {
    const payload = await this.reader.getRowPayload(this.messagesRoot, frame.id);
    const rec = this.reader.parse(payload);
    return rec.bytes(3); // messages body: [id(NULL), topic_id, timestamp, data]
  }

  close() {
    this.reader = null;
    this.framesByTopic = null;
  }
}
