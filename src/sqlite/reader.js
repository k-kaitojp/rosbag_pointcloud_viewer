// Minimal, read-only SQLite database reader that pulls only the pages it needs
// from an abstract byte source. This lets us index and fetch BLOBs from very
// large rosbag2 .db3 files (multi-GB) in the browser without loading the whole
// file into memory.
//
// Supports just enough of the SQLite file format for rosbag2:
//   - table b-trees (interior 0x05 / leaf 0x0d)
//   - record (serial type) decoding
//   - overflow page chains for large BLOBs
//
// SQLite file format reference: https://www.sqlite.org/fileformat2.html

const SQLITE_MAGIC = 'SQLite format 3\0';

/** A byte source backed by a browser File. */
export function fileSource(file) {
  return {
    size: file.size,
    async read(offset, length) {
      const end = Math.min(offset + length, file.size);
      const buf = await file.slice(offset, end).arrayBuffer();
      return new Uint8Array(buf);
    },
  };
}

function u16(buf, o) {
  return (buf[o] << 8) | buf[o + 1];
}

function u32(buf, o) {
  return (
    buf[o] * 0x1000000 + (buf[o + 1] << 16) + (buf[o + 2] << 8) + buf[o + 3]
  );
}

// SQLite varint: big-endian, 1..9 bytes. Returns [BigInt value, bytesConsumed].
function readVarint(buf, offset) {
  let result = 0n;
  for (let i = 0; i < 8; i++) {
    const byte = buf[offset + i];
    result = (result << 7n) | BigInt(byte & 0x7f);
    if ((byte & 0x80) === 0) return [result, i + 1];
  }
  result = (result << 8n) | BigInt(buf[offset + 8]);
  return [result, 9];
}

function serialTypeSize(st) {
  switch (st) {
    case 0:
    case 8:
    case 9:
      return 0;
    case 1:
      return 1;
    case 2:
      return 2;
    case 3:
      return 3;
    case 4:
      return 4;
    case 5:
      return 6;
    case 6:
    case 7:
      return 8;
    default:
      return st >= 12 ? Math.floor((st - 12) / 2) : 0;
  }
}

function readIntBE(buf, off, size) {
  if (size === 0) return 0n;
  let v = 0n;
  for (let i = 0; i < size; i++) v = (v << 8n) | BigInt(buf[off + i]);
  const bits = BigInt(size * 8);
  if (v & (1n << (bits - 1n))) v -= 1n << bits;
  return v;
}

const textDecoder = new TextDecoder('utf-8');

/**
 * Parse a record (row payload) into typed column accessors.
 * @param {Uint8Array} payload full record bytes (header + body)
 */
function parseRecord(payload) {
  const [headerLenBig, n0] = readVarint(payload, 0);
  const headerEnd = Number(headerLenBig);
  let p = n0;
  const cols = [];
  let bodyOffset = headerEnd;
  while (p < headerEnd) {
    const [stBig, used] = readVarint(payload, p);
    const st = Number(stBig);
    p += used;
    const size = serialTypeSize(st);
    cols.push({ st, offset: bodyOffset, size });
    bodyOffset += size;
  }
  return {
    columnCount: cols.length,
    int(i) {
      const c = cols[i];
      if (!c) return 0n;
      if (c.st === 8) return 0n;
      if (c.st === 9) return 1n;
      return readIntBE(payload, c.offset, c.size);
    },
    text(i) {
      const c = cols[i];
      if (!c) return '';
      return textDecoder.decode(payload.subarray(c.offset, c.offset + c.size));
    },
    bytes(i) {
      const c = cols[i];
      if (!c) return new Uint8Array(0);
      return payload.subarray(c.offset, c.offset + c.size);
    },
  };
}

export class SqliteReader {
  constructor(source) {
    this.source = source;
    this.pageSize = 0;
    this.reserved = 0;
    this.usable = 0;
    this.pageCount = 0;
    this.cache = new Map(); // pageNo -> Uint8Array
    this.cacheLimit = 256;
    this.pagesRead = 0;
  }

  async init() {
    const header = await this.source.read(0, 100);
    const magic = textDecoder.decode(header.subarray(0, 16));
    if (magic !== SQLITE_MAGIC) {
      throw new Error('Not a SQLite database file.');
    }
    let pageSize = u16(header, 16);
    if (pageSize === 1) pageSize = 65536;
    this.pageSize = pageSize;
    this.reserved = header[20];
    this.usable = pageSize - this.reserved;
    this.pageCount =
      u32(header, 28) || Math.floor(this.source.size / this.pageSize);
  }

  async readPage(pageNo) {
    const cached = this.cache.get(pageNo);
    if (cached) return cached;
    const buf = await this.source.read((pageNo - 1) * this.pageSize, this.pageSize);
    this.pagesRead++;
    this.cache.set(pageNo, buf);
    if (this.cache.size > this.cacheLimit) {
      // Evict oldest insertion.
      this.cache.delete(this.cache.keys().next().value);
    }
    return buf;
  }

  _btreeHeaderOffset(pageNo) {
    return pageNo === 1 ? 100 : 0;
  }

  /**
   * Walk a table b-tree, invoking onLeafCell(buf, cellOffset, rowid) for each
   * leaf cell. Interior pages are traversed but not reported.
   */
  async walkTable(rootPage, onLeafCell, onProgress) {
    await this._walk(rootPage, onLeafCell, onProgress);
  }

  async _walk(pageNo, onLeafCell, onProgress) {
    const buf = await this.readPage(pageNo);
    const h = this._btreeHeaderOffset(pageNo);
    const type = buf[h];
    const nCells = u16(buf, h + 3);

    if (type === 0x0d) {
      // leaf table
      const ptrBase = h + 8;
      for (let i = 0; i < nCells; i++) {
        const cellOff = u16(buf, ptrBase + i * 2);
        let p = cellOff;
        const [payloadLenBig, n1] = readVarint(buf, p);
        p += n1;
        const [rowidBig, n2] = readVarint(buf, p);
        p += n2;
        onLeafCell(buf, p, Number(rowidBig), Number(payloadLenBig));
      }
      if (onProgress) onProgress(this.pagesRead / this.pageCount);
    } else if (type === 0x05) {
      // interior table
      const ptrBase = h + 12;
      for (let i = 0; i < nCells; i++) {
        const cellOff = u16(buf, ptrBase + i * 2);
        const child = u32(buf, cellOff);
        await this._walk(child, onLeafCell, onProgress);
      }
      const rightMost = u32(buf, h + 8);
      await this._walk(rightMost, onLeafCell, onProgress);
    } else {
      throw new Error(`Unexpected b-tree page type 0x${type.toString(16)}`);
    }
  }

  // Local payload size for a table-leaf cell, per the SQLite spec.
  _localSize(payloadLen) {
    const maxLocal = this.usable - 35;
    if (payloadLen <= maxLocal) return payloadLen;
    const minLocal = Math.floor(((this.usable - 12) * 32) / 255) - 23;
    const surplus = minLocal + ((payloadLen - minLocal) % (this.usable - 4));
    return surplus <= maxLocal ? surplus : minLocal;
  }

  /** Assemble a full record payload, following overflow pages if needed. */
  async _assemblePayload(buf, payloadStart, payloadLen) {
    const local = this._localSize(payloadLen);
    if (local >= payloadLen) {
      return buf.subarray(payloadStart, payloadStart + payloadLen);
    }
    const out = new Uint8Array(payloadLen);
    out.set(buf.subarray(payloadStart, payloadStart + local), 0);
    let filled = local;
    let next = u32(buf, payloadStart + local);
    while (filled < payloadLen && next !== 0) {
      const opage = await this.readPage(next);
      next = u32(opage, 0);
      const chunk = Math.min(this.usable - 4, payloadLen - filled);
      out.set(opage.subarray(4, 4 + chunk), filled);
      filled += chunk;
    }
    return out;
  }

  /**
   * Read the inline (local) portion of a leaf cell payload. Safe for columns
   * known to live near the start of the record (no overflow needed).
   */
  _localPayload(buf, payloadStart, payloadLen) {
    const local = this._localSize(payloadLen);
    return buf.subarray(payloadStart, payloadStart + local);
  }

  /** Navigate a table b-tree by rowid and return the full record payload. */
  async getRowPayload(rootPage, rowid) {
    let pageNo = rootPage;
    // Guard against malformed/cyclic trees.
    for (let depth = 0; depth < 64; depth++) {
      const buf = await this.readPage(pageNo);
      const h = this._btreeHeaderOffset(pageNo);
      const type = buf[h];
      const nCells = u16(buf, h + 3);

      if (type === 0x0d) {
        const ptrBase = h + 8;
        for (let i = 0; i < nCells; i++) {
          const cellOff = u16(buf, ptrBase + i * 2);
          let p = cellOff;
          const [payloadLenBig, n1] = readVarint(buf, p);
          p += n1;
          const [rowidBig, n2] = readVarint(buf, p);
          p += n2;
          if (Number(rowidBig) === rowid) {
            return this._assemblePayload(buf, p, Number(payloadLenBig));
          }
        }
        throw new Error(`rowid ${rowid} not found in leaf page ${pageNo}`);
      } else if (type === 0x05) {
        const ptrBase = h + 12;
        let next = u32(buf, h + 8); // default: right-most pointer
        for (let i = 0; i < nCells; i++) {
          const cellOff = u16(buf, ptrBase + i * 2);
          const child = u32(buf, cellOff);
          const [keyBig] = readVarint(buf, cellOff + 4);
          if (rowid <= Number(keyBig)) {
            next = child;
            break;
          }
        }
        pageNo = next;
      } else {
        throw new Error(`Unexpected b-tree page type 0x${type.toString(16)}`);
      }
    }
    throw new Error(`rowid ${rowid} lookup exceeded max depth`);
  }

  /**
   * Load the schema (sqlite_master, rooted at page 1).
   * @returns {Map<string, {type, name, tblName, rootpage}>} keyed by name
   */
  async loadSchema() {
    const tables = new Map();
    await this.walkTable(1, async (buf, payloadStart, _rowid, payloadLen) => {
      // schema rows are small but `sql` can be long -> we only read inline
      // columns (type, name, tbl_name, rootpage all precede `sql`).
      const local = this._localPayload(buf, payloadStart, payloadLen);
      const rec = parseRecord(local);
      tables.set(rec.text(1), {
        type: rec.text(0),
        name: rec.text(1),
        tblName: rec.text(2),
        rootpage: Number(rec.int(3)),
      });
    });
    return tables;
  }

  /** Convenience: full payload -> parsed record. */
  parse(payload) {
    return parseRecord(payload);
  }

  localRecord(buf, payloadStart, payloadLen) {
    return parseRecord(this._localPayload(buf, payloadStart, payloadLen));
  }
}
