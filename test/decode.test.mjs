import { decodePointCloud2, extractPoints } from '../src/pointcloud2.js';

// Build a synthetic sensor_msgs/msg/PointCloud2 CDR message (little-endian).
// Layout mirrors rmw CDR serialization with 4-byte encapsulation header.
const buf = [];
const dv = new DataView(new ArrayBuffer(4096));
let off = 0;
const align = (n) => { const rel=(off-4)%n; if(rel) off+=n-rel; };
const u8=(v)=>{dv.setUint8(off,v);off+=1;};
const i32=(v)=>{align(4);dv.setInt32(off,v,true);off+=4;};
const u32=(v)=>{align(4);dv.setUint32(off,v,true);off+=4;};
const f32=(v)=>{align(4);dv.setFloat32(off,v,true);off+=4;};
const str=(s)=>{u32(s.length+1);for(const c of s){u8(c.charCodeAt(0));}u8(0);};

// encapsulation header: 0x00, 0x01 (CDR_LE), 0x00, 0x00
dv.setUint8(0,0);dv.setUint8(1,1);dv.setUint8(2,0);dv.setUint8(3,0);
off=4;

// Header
i32(12345);          // stamp.sec
u32(678900000);      // stamp.nanosec
str('lidar_frame');  // frame_id

const N = 3;
u32(1);              // height
u32(N);              // width

// fields: x,y,z (FLOAT32) + intensity (FLOAT32)
u32(4);
const field=(name,offset,dt)=>{str(name);u32(offset);u8(dt);u32(1);};
field('x',0,7);
field('y',4,7);
field('z',8,7);
field('intensity',12,7);

u8(0);               // is_bigendian = false
u32(16);             // point_step
u32(16*N);           // row_step

// data sequence
const pts=[[1,2,3,0.5],[4,5,6,0.7],[NaN,0,0,0.1]]; // last point invalid -> dropped
u32(16*N);           // data length
const dataStart=off;
for(const p of pts){
  for(const v of p){dv.setFloat32(off,v,true);off+=4;}
}
u8(1);               // is_dense

const msg = new Uint8Array(dv.buffer,0,off);
const pc = decodePointCloud2(msg);
console.log('frame_id:', pc.frame_id, '| stamp:', pc.stamp, '| width:', pc.width, '| fields:', pc.fields.map(f=>f.name).join(','));
console.log('point_step:', pc.point_step, '| data bytes:', pc.data.length);

const ex = extractPoints(pc);
console.log('valid points:', ex.count, '(expected 2)');
console.log('positions:', Array.from(ex.positions));
console.log('intensity:', Array.from(ex.scalars.intensity));
console.log('bounds.center:', ex.bounds.center, '| size:', ex.bounds.size.toFixed(3));

const ok = ex.count===2 &&
  ex.positions[0]===1 && ex.positions[1]===2 && ex.positions[2]===3 &&
  ex.positions[3]===4 && ex.positions[4]===5 && ex.positions[5]===6 &&
  pc.frame_id==='lidar_frame' && pc.stamp.sec===12345 && pc.stamp.nanosec===678900000;
console.log(ok ? '\n✅ DECODER TEST PASSED' : '\n❌ DECODER TEST FAILED');
process.exit(ok?0:1);
