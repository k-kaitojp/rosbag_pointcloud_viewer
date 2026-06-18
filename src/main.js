import './style.css';
import { Db3Source } from './sources/db3.js';
import { McapSource } from './sources/mcap.js';
import { decodePointCloud2, extractPoints } from './pointcloud2.js';
import { PointCloudViewer } from './viewer.js';

const el = (id) => document.getElementById(id);

const dom = {
  fileInput: el('file-input'),
  dropZone: el('drop-zone'),
  dropOverlay: el('drop-overlay'),
  sidebar: el('sidebar'),
  fileName: el('file-name'),
  topicList: el('topic-list'),
  controls: el('controls'),
  colorMode: el('color-mode'),
  pointSize: el('point-size'),
  pointSizeVal: el('point-size-val'),
  timeline: el('timeline'),
  frameInfo: el('frame-info'),
  playBtn: el('play-btn'),
  status: el('status'),
  viewer: el('viewer'),
  stats: el('stats'),
};

const viewer = new PointCloudViewer(dom.viewer);

const state = {
  source: null,
  topic: null,
  frames: [],
  frameIndex: 0,
  playing: false,
  playTimer: null,
};

function setStatus(msg, isError = false) {
  dom.status.textContent = msg || '';
  dom.status.classList.toggle('error', isError);
}

async function loadFile(file) {
  setStatus(`Loading ${file.name} …`);
  stopPlayback();
  if (state.source) {
    state.source.close();
    state.source = null;
  }

  // 1) Read the file bytes. Large files / files changed after selection can
  // throw NotReadableError, so read in chunks and report clearly.
  let buf;
  try {
    buf = await readFileBuffer(file, (p) => {
      setStatus(`Reading ${file.name} … ${Math.round(p * 100)}%`);
    });
  } catch (err) {
    console.error(err);
    setStatus(describeReadError(err, file), true);
    return;
  }

  // 2) Parse the bag and list topics.
  try {
    const lower = file.name.toLowerCase();
    if (lower.endsWith('.mcap')) {
      state.source = await McapSource.open(buf);
    } else if (lower.endsWith('.db3') || lower.endsWith('.sqlite3')) {
      state.source = await Db3Source.open(buf);
    } else {
      // Heuristic: SQLite files start with "SQLite format 3\0".
      const head = new Uint8Array(buf.slice(0, 16));
      const isSqlite = String.fromCharCode(...head).startsWith('SQLite format 3');
      state.source = isSqlite
        ? await Db3Source.open(buf)
        : await McapSource.open(buf);
    }

    dom.fileName.textContent = file.name;
    dom.sidebar.hidden = false;

    const topics = state.source.listPointCloudTopics();
    renderTopicList(topics);

    if (topics.length === 0) {
      setStatus('No sensor_msgs/msg/PointCloud2 topics found in this bag.', true);
    } else {
      setStatus(`Found ${topics.length} PointCloud2 topic(s). Select one to view.`);
      selectTopic(topics[0]);
    }
  } catch (err) {
    console.error(err);
    setStatus(`Failed to parse bag: ${err.message}`, true);
  }
}

// Reads a File into an ArrayBuffer. For large files we read in chunks via
// Blob.slice() instead of a single Blob.arrayBuffer() call, which can throw
// NotReadableError in some browsers once the blob exceeds a size threshold.
async function readFileBuffer(file, onProgress) {
  const CHUNK = 128 * 1024 * 1024; // 128 MB
  if (file.size <= CHUNK) {
    return await file.arrayBuffer();
  }

  let out;
  try {
    out = new Uint8Array(file.size);
  } catch (e) {
    throw new RangeError(
      `File is too large to load into memory in the browser (${formatBytes(
        file.size,
      )}).`,
    );
  }

  let offset = 0;
  while (offset < file.size) {
    const end = Math.min(offset + CHUNK, file.size);
    const chunk = await file.slice(offset, end).arrayBuffer();
    out.set(new Uint8Array(chunk), offset);
    offset = end;
    if (onProgress) onProgress(offset / file.size);
  }
  return out.buffer;
}

function describeReadError(err, file) {
  const size = formatBytes(file.size);
  if (err && err.name === 'NotReadableError') {
    return (
      `Could not read "${file.name}" (${size}). ` +
      `The file may have been moved/modified after selection, may be on a ` +
      `synced or network drive (OneDrive/Google Drive/SMB), or may be too ` +
      `large for the browser. Copy it to a local folder and re-select it.`
    );
  }
  if (err instanceof RangeError) {
    return err.message;
  }
  return `Failed to read "${file.name}" (${size}): ${err.message}`;
}

function formatBytes(bytes) {
  if (!bytes) return '0 B';
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  const i = Math.floor(Math.log(bytes) / Math.log(1024));
  return `${(bytes / Math.pow(1024, i)).toFixed(i ? 1 : 0)} ${units[i]}`;
}

function renderTopicList(topics) {
  dom.topicList.innerHTML = '';
  for (const t of topics) {
    const li = document.createElement('li');
    li.className = 'topic';
    li.dataset.id = t.id;
    li.innerHTML = `<span class="topic-name">${t.name}</span>
      <span class="topic-meta">${t.count} msgs</span>`;
    li.addEventListener('click', () => selectTopic(t));
    dom.topicList.appendChild(li);
  }
}

function selectTopic(topic) {
  state.topic = topic;
  stopPlayback();

  for (const li of dom.topicList.children) {
    li.classList.toggle('active', String(li.dataset.id) === String(topic.id));
  }

  setStatus(`Indexing "${topic.name}" …`);
  state.frames = state.source.listFrames(topic.id);
  state.frameIndex = 0;

  if (state.frames.length === 0) {
    setStatus(`Topic "${topic.name}" has no messages.`, true);
    return;
  }

  dom.controls.hidden = false;
  dom.timeline.max = String(state.frames.length - 1);
  dom.timeline.value = '0';

  showFrame(0, /*frameCamera=*/ true);
  setStatus('');
}

function showFrame(index, frameCamera = false) {
  const frame = state.frames[index];
  if (!frame) return;
  state.frameIndex = index;

  const raw = state.source.getFrameData(state.topic.id, frame);
  if (!raw) {
    setStatus(`Could not read message for frame ${index}.`, true);
    return;
  }

  let extracted;
  try {
    const pc = decodePointCloud2(raw);
    extracted = extractPoints(pc);
    extracted.frame_id = pc.frame_id;
    extracted.stamp = pc.stamp;
  } catch (err) {
    console.error(err);
    setStatus(`Decode error on frame ${index}: ${err.message}`, true);
    return;
  }

  // Populate color-mode dropdown on first frame of a topic.
  if (frameCamera) {
    populateColorModes(extracted);
  }

  viewer.showCloud(extracted);
  if (frameCamera) viewer.frameCloud(extracted.bounds);

  dom.timeline.value = String(index);
  updateFrameInfo(extracted);
}

function populateColorModes(extracted) {
  const modes = viewer.availableColorModes(extracted);
  const prev = dom.colorMode.value;
  dom.colorMode.innerHTML = '';
  for (const m of modes) {
    const opt = document.createElement('option');
    opt.value = m.value;
    opt.textContent = m.label;
    dom.colorMode.appendChild(opt);
  }
  // Prefer embedded RGB if present, otherwise keep prior choice or default.
  const values = modes.map((m) => m.value);
  const chosen = extracted.rgb
    ? 'rgb'
    : values.includes(prev)
      ? prev
      : 'axis-z';
  dom.colorMode.value = chosen;
  viewer.setColorMode(chosen);
}

function updateFrameInfo(extracted) {
  const total = state.frames.length;
  const t = state.frames[state.frameIndex].timestamp;
  const stamp = extracted.stamp
    ? `${extracted.stamp.sec}.${String(extracted.stamp.nanosec).padStart(9, '0')}`
    : '—';
  dom.frameInfo.textContent = `Frame ${state.frameIndex + 1} / ${total}`;
  dom.stats.innerHTML = `
    <div><span>Points</span><b>${extracted.count.toLocaleString()}</b></div>
    <div><span>frame_id</span><b>${extracted.frame_id || '—'}</b></div>
    <div><span>stamp</span><b>${stamp}</b></div>
    <div><span>log time</span><b>${formatNs(t)}</b></div>`;
}

function formatNs(ns) {
  // rosbag2 timestamps are nanoseconds since epoch.
  if (ns == null) return '—';
  const seconds = Number(ns) / 1e9;
  return `${seconds.toFixed(3)} s`;
}

function startPlayback() {
  if (state.frames.length <= 1) return;
  state.playing = true;
  dom.playBtn.textContent = '⏸ Pause';
  const fps = 10;
  state.playTimer = setInterval(() => {
    let next = state.frameIndex + 1;
    if (next >= state.frames.length) next = 0;
    showFrame(next);
  }, 1000 / fps);
}

function stopPlayback() {
  state.playing = false;
  if (dom.playBtn) dom.playBtn.textContent = '▶ Play';
  if (state.playTimer) {
    clearInterval(state.playTimer);
    state.playTimer = null;
  }
}

// ---- Event wiring ----

dom.fileInput.addEventListener('change', (e) => {
  const file = e.target.files[0];
  if (file) loadFile(file);
});

// Whole-window drag & drop. A counter handles dragenter/dragleave firing on
// nested elements so the overlay doesn't flicker.
let dragDepth = 0;

const hasFiles = (e) =>
  e.dataTransfer && Array.from(e.dataTransfer.types || []).includes('Files');

window.addEventListener('dragenter', (e) => {
  if (!hasFiles(e)) return;
  e.preventDefault();
  dragDepth++;
  dom.dropOverlay.hidden = false;
});

window.addEventListener('dragover', (e) => {
  if (!hasFiles(e)) return;
  e.preventDefault();
  e.dataTransfer.dropEffect = 'copy';
});

window.addEventListener('dragleave', (e) => {
  if (!hasFiles(e)) return;
  e.preventDefault();
  dragDepth = Math.max(0, dragDepth - 1);
  if (dragDepth === 0) dom.dropOverlay.hidden = true;
});

window.addEventListener('drop', (e) => {
  e.preventDefault();
  dragDepth = 0;
  dom.dropOverlay.hidden = true;
  const file = e.dataTransfer.files[0];
  if (file) loadFile(file);
});

dom.colorMode.addEventListener('change', (e) =>
  viewer.setColorMode(e.target.value),
);

dom.pointSize.addEventListener('input', (e) => {
  const size = Number(e.target.value);
  viewer.setPointSize(size);
  dom.pointSizeVal.textContent = size.toFixed(3);
});

dom.timeline.addEventListener('input', (e) => {
  stopPlayback();
  showFrame(Number(e.target.value));
});

dom.playBtn.addEventListener('click', () => {
  if (state.playing) stopPlayback();
  else startPlayback();
});

setStatus('Open a ROS2 rosbag (.db3 or .mcap) to begin.');
