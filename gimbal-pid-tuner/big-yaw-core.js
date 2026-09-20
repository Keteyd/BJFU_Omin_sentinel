/* USART1 big-yaw monitor protocol. No motor-enable or motion commands. */
(function (root) {
  'use strict';
  const fields = [
    {key: 'angle_kp', label: '角度环 Kp', max: 8, step: .001, scale: 1000, unit: ''},
    {key: 'speed_kp', label: '速度环 Kp', max: 5, step: .001, scale: 1000, unit: ''},
    {key: 'effort_limit', label: '输出上限', max: 30, step: .001, scale: 1000, unit: '驱动单位'},
    {key: 'filter_tau_s', label: '速度滤波时间常数', max: .1, step: .0001, scale: 10000, unit: 's'},
    {key: 'angle_ki', label: '角度环 Ki', max: 0, step: .000001, scale: 1, unit: ''},
    {key: 'angle_kd', label: '角度环 Kd', max: 0, step: .001, scale: 1, unit: ''},
    {key: 'speed_ki', label: '速度环 Ki', max: 0, step: .000001, scale: 1, unit: ''},
    {key: 'speed_kd', label: '速度环 Kd', max: 0, step: .001, scale: 1, unit: ''}
  ];
  const legacyFields = fields.slice(0, 4);
  function fullValues(values) {
    return fields.map(f => {
      const v = values[f.key], encoded = Math.fround(v);
      if (typeof v !== 'number' || !Number.isFinite(v) || v < 0 || !Number.isFinite(encoded) || (v !== 0 && encoded === 0))
        throw Error(`${f.label} 必须是可表示的非负有限浮点数`);
      if (f.key === 'effort_limit' && v > 30) throw Error('输出不能超过 GM6020 协议满量程 30');
      return encoded;
    });
  }
  function crc8(bytes) {
    let crc = 0;
    for (const byte of bytes) {
      crc ^= byte;
      for (let bit = 0; bit < 8; bit++) crc = ((crc << 1) ^ ((crc & 128) ? 0x31 : 0)) & 255;
    }
    return crc;
  }
  function frame(command, payload) {
    if (payload.length > 12) throw Error('Invalid payload');
    const bytes = new Uint8Array(16);
    bytes[0] = 255; bytes[1] = command; bytes.set(payload, 2);
    bytes[14] = crc8(bytes.subarray(0, 14)); bytes[15] = 13;
    return bytes;
  }
  function encoded(values) {
    return legacyFields.map(f => {
      const v = values[f.key];
      if (typeof v !== 'number' || !Number.isFinite(v) || v < 0 || v > f.max)
        throw Error(`${f.label} 必须在 0 至 ${f.max} 之间`);
      return Math.round(v * f.scale);
    });
  }
  function tune(values, id) {
    if (!Number.isInteger(id) || id < 1 || id > 65535) throw Error('Invalid request ID');
    const data = new Uint8Array(12), view = new DataView(data.buffer);
    encoded(values).forEach((v, i) => view.setUint16(i * 2, v, true));
    view.setUint16(8, id, true); data[10] = 0xB7; data[11] = 1;
    return frame(0x2D, data);
  }
  function decode(bytes) {
    if (bytes.length !== 16 || bytes[0] !== 255 || bytes[15] !== 13 ||
        crc8(bytes.subarray(0, 14)) !== bytes[14]) return null;
    const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (bytes[1] === 0x31) return {type:'full_part', id:v.getUint16(2,true), part:bytes[4],
      flags:bytes[5], pair:[v.getFloat32(6,true),v.getFloat32(10,true)]};
    if (bytes[1] === 0x2E) {
      if (!(bytes[13] & 128) || bytes[12] > 4) throw Error('大 Yaw 回读协议版本不兼容');
      if (bytes[13] & 64) return {type:'capability', fullPid:true};
      const c = {type: 'config', id: v.getUint16(10, true), status: bytes[12],
        safe: !!(bytes[13] & 1), off: !!(bytes[13] & 2), active: !!(bytes[13] & 4),
        saturated: !!(bytes[13] & 8), online: !!(bytes[13] & 16),
        effort_limit_max: bytes[13] & 32 ? 30 : 8, fullPid:false,
        angle_ki:0,angle_kd:0,speed_ki:0,speed_kd:0};
      legacyFields.forEach((f, i) => { c[f.key] = v.getUint16(2 + i * 2, true) / f.scale; });
      encoded(c);
      if (c.effort_limit > c.effort_limit_max) throw Error('输出回读超过固件声明的范围');
      return c;
    }
    if (bytes[1] === 0x2C && bytes[13] === 1) {
      return {type: 'sample', error: v.getInt16(2, true) / 100,
        speedRef: v.getInt16(4, true) / 100, speed: v.getInt16(6, true) / 100,
        effort: v.getInt16(8, true) / 1000, mode: bytes[10], active: !!bytes[11]};
    }
    if (bytes[1] === 0x2A && bytes[13] === 1) {
      const raw = v.getUint16(4, true);
      return {type: 'encoder', angle: raw * 360 / 8192,
        valid: !!(bytes[10] & 2) && raw < 8192 && v.getUint16(8, true) <= 50,
        monitor: !!bytes[12]};
    }
    return null;
  }
  class Parser {
    constructor() { this.buffer = []; this.parts = []; this.fullKey = ''; this.fullAt = 0; }
    fullPart(data) {
      const key = `${data.id}:${data.flags}`;
      if (data.part === 0) { this.parts = []; this.fullKey = key; this.fullAt = performance.now(); }
      if (data.part !== this.parts.length || data.part > 3 || key !== this.fullKey ||
          performance.now() - this.fullAt > 350 || (data.flags & 7) > 4) {
        this.parts = []; this.fullKey = ''; return null;
      }
      this.parts.push(data.pair);
      if (this.parts.length !== 4) return null;
      const c = Object.fromEntries(fields.map((f,i) => [f.key, this.parts.flat()[i]]));
      this.parts = []; this.fullKey = '';
      fullValues(c);
      return {...c,type:'config',id:data.id,status:data.flags&7,safe:!!(data.flags&8),
        off:!!(data.flags&16),active:!!(data.flags&32),online:!!(data.flags&64),
        saturated:!!(data.flags&128),fullPid:true,effort_limit_max:30};
    }
    push(chunk) {
      const result = [];
      for (const byte of chunk) {
        if (!this.buffer.length && byte !== 255) continue;
        this.buffer.push(byte);
        if (this.buffer.length < 16) continue;
        const b = Uint8Array.from(this.buffer);
        if (b[15] === 13 && crc8(b.subarray(0, 14)) === b[14]) {
          let data = decode(b);
          if (data?.type === 'full_part') data = this.fullPart(data);
          if (data) result.push(data);
          this.buffer = [];
        } else {
          this.buffer.shift();
          while (this.buffer.length && this.buffer[0] !== 255) this.buffer.shift();
        }
      }
      return result;
    }
  }
  function safe(c, ageMs) {
    return !!c && ageMs >= 0 && ageMs <= 400 && c.safe && c.off && !c.active && c.online;
  }
  function matches(c, values) {
    if (c.fullPid) {
      const expected = fullValues(values);
      return fullValues(c).every((v,i) => v === expected[i]);
    }
    return encoded(c).every((v, i) => v === encoded(values)[i]);
  }
  function tuneFrames(values, id, full) {
    if (!full) return [tune(values,id)];
    if (!Number.isInteger(id) || id < 1 || id > 65535) throw Error('Invalid request ID');
    const numbers=fullValues(values);
    return Array.from({length:4},(_,part) => {
      const data=new Uint8Array(12), v=new DataView(data.buffer);
      v.setUint16(0,id,true);data[2]=part;data[3]=0xC3;
      v.setFloat32(4,numbers[part*2],true);v.setFloat32(8,numbers[part*2+1],true);
      return frame(0x30,data);
    });
  }
  root.BigYawProtocol = {fields, legacyFields, fullValues, tuneFrames, crc8, frame, encoded, tune, decode, Parser, safe, matches,
    monitor: () => frame(0x24, new Uint8Array([0xA5, 1]))};
})(globalThis);
