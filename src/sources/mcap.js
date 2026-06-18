// Reads ROS2 rosbag2 MCAP storage (.mcap). Supports uncompressed and
// lz4/zstd-compressed chunks (decompressors loaded lazily via @mcap/support).

import { McapStreamReader } from '@mcap/core';

const POINTCLOUD2_TYPE = 'sensor_msgs/msg/PointCloud2';

export class McapSource {
  constructor({ channels, messagesByChannel }) {
    this.channels = channels; // Map<channelId, { topic, schemaName }>
    this.messagesByChannel = messagesByChannel; // Map<channelId, [{ timestamp, data }]>
  }

  static async open(arrayBuffer) {
    let decompressHandlers;
    try {
      const support = await import('@mcap/support');
      decompressHandlers = await support.loadDecompressHandlers();
    } catch (e) {
      // Decompressors are optional; uncompressed MCAPs still work.
      decompressHandlers = undefined;
    }

    const reader = new McapStreamReader({
      validateCrcs: false,
      decompressHandlers,
    });
    reader.append(new Uint8Array(arrayBuffer));

    const schemas = new Map(); // schemaId -> name
    const channels = new Map();
    const messagesByChannel = new Map();

    let record;
    while ((record = reader.nextRecord())) {
      switch (record.type) {
        case 'Schema':
          schemas.set(record.id, record.name);
          break;
        case 'Channel':
          channels.set(record.id, {
            topic: record.topic,
            schemaName: schemas.get(record.schemaId) || '',
          });
          if (!messagesByChannel.has(record.id)) {
            messagesByChannel.set(record.id, []);
          }
          break;
        case 'Message': {
          let arr = messagesByChannel.get(record.channelId);
          if (!arr) {
            arr = [];
            messagesByChannel.set(record.channelId, arr);
          }
          arr.push({
            timestamp: Number(record.logTime),
            // Copy out of the shared buffer so it survives the streaming reader.
            data: record.data.slice(),
          });
          break;
        }
        default:
          break;
      }
    }

    if (!reader.done()) {
      // Most ROS2 mcap files end with proper footer/magic; warn but continue.
      console.warn('MCAP stream did not reach a clean end-of-file marker.');
    }

    for (const arr of messagesByChannel.values()) {
      arr.sort((a, b) => a.timestamp - b.timestamp);
    }

    return new McapSource({ channels, messagesByChannel });
  }

  listPointCloudTopics() {
    const topics = [];
    for (const [id, ch] of this.channels) {
      if (ch.schemaName === POINTCLOUD2_TYPE) {
        const msgs = this.messagesByChannel.get(id) || [];
        topics.push({ id, name: ch.topic, type: ch.schemaName, count: msgs.length });
      }
    }
    topics.sort((a, b) => a.name.localeCompare(b.name));
    return topics;
  }

  listFrames(channelId) {
    const msgs = this.messagesByChannel.get(channelId) || [];
    return msgs.map((m, i) => ({ id: i, timestamp: m.timestamp }));
  }

  getFrameData(channelId, frame) {
    const msgs = this.messagesByChannel.get(channelId) || [];
    return msgs[frame.id] ? msgs[frame.id].data : null;
  }

  close() {
    this.channels.clear();
    this.messagesByChannel.clear();
  }
}
