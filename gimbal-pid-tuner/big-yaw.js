'use strict';
const P = BigYawProtocol;
const $ = id => document.getElementById(id);
let connection = null, config = null, configAt = -Infinity, sampleAt = -Infinity;
let encoderAt = -Infinity, encoder = null, latest = null, pending = null, reviewValues = null;
let requireRead = false, readRequested = false, initialized = false, paused = false;
let reviewBase = null, readRequestToken = 0;
let samples = [], frozen = [], requestCounter = 0;
const inputs = {};
const modes = ['禁用', '保持', '左侧跟随', '右侧跟随', '制动'];
const pretty = (v, digits = 3) => Number.isFinite(v) ? v.toFixed(digits) : '—';
function icons() { globalThis.lucide?.createIcons(); }
for (const index of [0,4,5,1,6,7,2,3]) {
  const f = P.fields[index];
  const row = document.createElement('div'); row.className = 'field';
  row.innerHTML = `<label for="${f.key}">${f.label}</label><div class="field-row"><input id="${f.key}" type="number" min="0" max="${f.max}" step="${f.step}" placeholder="—" required disabled><output id="read_${f.key}">—</output></div><div class="unit">${f.unit || '\u00a0'}</div>`;
  $('fields').append(row); inputs[f.key] = $(f.key);
  inputs[f.key].addEventListener('input', refresh);
}
function notice(text, type = '') { $('notice').textContent = text; $('notice').className = type; }
function log(text, error = false) {
  $('events').querySelector('.placeholder')?.remove();
  const li = document.createElement('li'), t = document.createElement('time');
  t.textContent = new Date().toLocaleTimeString('zh-CN', {hour12: false});
  li.append(t, document.createTextNode(text));
  if (error) li.className = 'error';
  $('events').prepend(li);
  while ($('events').children.length > 30) $('events').lastElementChild.remove();
}
function loadDraft(c) {
  P.fields.forEach(f => { inputs[f.key].value = c[f.key]; });
  initialized = true;
}
function draft() {
  const v = {};
  P.fields.forEach(f => {
    if (!inputs[f.key].value.trim()) throw Error(`${f.label} 不能为空`);
    v[f.key] = Number(inputs[f.key].value);
  });
  if (config?.fullPid) {
    const encoded = P.fullValues(v);
    P.fields.forEach((f,i) => {v[f.key]=encoded[i];});
  } else {
    if (P.fields.slice(4).some(f=>v[f.key]!==0)) throw Error('当前固件不支持 Ki/Kd，需要烧录完整 PID 版');
    const encoded = P.encoded(v);
    P.legacyFields.forEach((f, i) => { v[f.key] = encoded[i] / f.scale; });
  }
  return v;
}
function writable() {
  return !!connection?.writer && !connection.closing && !requireRead &&
    P.safe(config, performance.now() - configAt);
}
function writeBlockReason() {
  if (!connection?.writer || connection.closing) return '串口未连接';
  if (pending) return `等待写入确认 #${pending.id}`;
  if (readRequested) return '正在读取当前参数，等待新一帧回读';
  if (requireRead) return '参数状态待核对，请重新读取当前参数';
  if (!config || !initialized) return '等待大 Yaw 参数回读';
  if (performance.now() - configAt > 400) return '回读已过期，禁止写入';
  if (!config.online) return '电机反馈离线，禁止写入';
  if (!config.safe) return '右拨杆不在上置安全模式，禁止写入';
  if (!config.off || config.active) return 'Yaw 输出尚未关闭，禁止写入';
  try {
    const values = draft();
    if (values.effort_limit > config.effort_limit_max) return '当前固件上限为 8，超过 8 需要烧录满量程版云台固件';
    if (P.matches(config, values)) return '参数未变化（按协议精度），无需写入';
  } catch (error) { return error.message; }
  return '';
}
function gate() {
  return writeBlockReason() || 'SAFE · 参数已变化，可以检查并写入';
}
function refresh() {
  const now = performance.now(), fresh = !!config && now - configAt <= 400;
  $('connectionState').textContent = connection?.closing ? '正在断开' :
    connection?.writer ? (fresh ? '遥测在线' : '串口已连接 · 等待回读') :
    connection ? '正在连接' : '未连接';
  $('connect').disabled = !!connection && (!connection.writer || connection.closing);
  $('connect').querySelector('span').textContent = connection?.writer ? '断开串口' : '连接串口';
  $('remoteState').textContent = fresh ? (config.safe ? 'SAFE / 上置' : '非安全模式') : '遥控状态未知';
  $('remoteState').className = 'badge' + (fresh ? (config.safe ? ' safe' : ' unsafe') : '');
  $('motorState').textContent = '电机 ' + (fresh ? (config.online ? '在线' : '离线') : '—');
  $('activeState').textContent = '闭环 ' + (fresh ? (config.active ? '启用' : '关闭') : '—');
  $('saturationState').textContent = '输出 ' + (fresh ? (config.saturated ? '饱和' : '未饱和') : '—');
  $('age').textContent = config ? `回读 ${Math.round(now - configAt)} ms` : '回读 —';
  const blockReason = writeBlockReason();
  $('writeGate').textContent = blockReason || 'SAFE · 参数已变化，可以检查并写入';
  $('apply').disabled = !!blockReason || !writable();
  $('apply').title = blockReason || '检查大 Yaw 参数变更';
  $('read').disabled = !connection?.writer || connection.closing || !!pending || readRequested;
  for (const f of P.fields) {
    const unsupported = P.fields.indexOf(f)>=4 && !config?.fullPid;
    inputs[f.key].disabled = !initialized || !!pending || readRequested || unsupported;
    if (config?.fullPid && f.key !== 'effort_limit') inputs[f.key].removeAttribute('max');
    else inputs[f.key].max = f.max;
    inputs[f.key].step = config?.fullPid ? 'any' : f.step;
    $('read_' + f.key).textContent = fresh ? (config.fullPid ? Number(config[f.key].toPrecision(7)).toString() : pretty(config[f.key], f.scale === 10000 ? 4 : 3)) : '—';
    if (P.fields.indexOf(f)>=4) inputs[f.key].closest('.field').querySelector('.unit').textContent = config && unsupported ? '旧固件不支持' : '';
  }
  inputs.effort_limit.max = config ? config.effort_limit_max : 8;
  inputs.effort_limit.closest('.field').querySelector('.unit').textContent =
    config ? `驱动单位 · 当前固件上限 ${config.effort_limit_max}` : '驱动单位 · 固件上限未知';
  $('protocolState').textContent = config ? (config.fullPid ? '大 Yaw · 0x30 / 0x31' : '大 Yaw · 0x2D / 0x2E') : '大 Yaw · 接口待确认';
  const reviewChanged = !!reviewBase && !!config &&
    (config.fullPid !== reviewBase.fullPid || !P.matches(config, reviewBase));
  const reviewOutOfRange = !!reviewValues && !!config && reviewValues.effort_limit > config.effort_limit_max;
  $('reviewGate').textContent = reviewChanged ? '主控参数已变化，请取消后重新检查' : gate();
  if (!writable() || reviewChanged || reviewOutOfRange) $('confirmed').checked = false;
  $('confirmApply').disabled = !writable() || !!pending || !$('confirmed').checked || reviewChanged || reviewOutOfRange;
  $('angle').textContent = encoder?.valid && now - encoderAt <= 300 ? pretty(encoder.angle, 2) : '—';
  const live = latest && now - sampleAt <= 400;
  for (const key of ['error', 'speed', 'effort']) $(key).textContent = live ? pretty(latest[key], key === 'effort' ? 3 : 2) : '—';
  $('mode').textContent = live ? `大 Yaw / ${modes[latest.mode] || '未知模式'}${paused ? ' / 曲线暂停' : ''}` : '大 Yaw / 等待遥测';
  $('sampleRate').textContent = `${samples.length} 帧`;
  $('export').disabled = !samples.length;
  if (pending && now > pending.deadline) failPending('确认超时，结果未知；请重新读取，勿盲目重试');
  drawCharts();
}
function failPending(message) {
  if (pending) log(`#${pending.id} ${message}`, true);
  pending = null; requireRead = true;
  notice(message, 'error');
}
function handle(data) {
  const now = performance.now();
  if (data.type === 'capability') {
    if (!config?.fullPid) { config=null;configAt=-Infinity;initialized=false; }
  } else if (data.type === 'config') {
    config = data; configAt = now;
    if (pending && data.id === pending.id) {
      if (data.status !== 1) {
        failPending(`主控拒绝写入：${({0:'未确认',2:'非安全状态',3:'参数无效',4:'请求过期'})[data.status] || data.status}`);
      } else if (data.fullPid !== pending.fullPid || !P.matches(data, pending.values)) {
        failPending('回读不一致，参数状态待核对');
      } else {
        log(`#${pending.id} 写入成功，回读一致`);
        $('lastRequest').textContent = `#${pending.id}`;
        pending = null; requireRead = false; loadDraft(data);
        notice('大 Yaw 参数已确认写入 RAM', 'success');
      }
    }
    if (!initialized || readRequested) {
      loadDraft(data);
      if (readRequested) { requireRead = false; notice('已读取主控当前参数', 'success'); }
      readRequested = false;
    }
  } else if (data.type === 'sample') {
    latest = data; sampleAt = now;
    samples.push({...data, t: now / 1000, utc: new Date().toISOString()});
    if (samples.length > 6000) samples.shift();
  } else if (data.type === 'encoder') {
    encoder = data; encoderAt = now;
    if (!data.monitor) throw Error('当前串口不是只读监视会话，请断开其他调参程序');
  }
}
async function send(s, bytes, guard = null) {
  const action = async () => {
    if (connection !== s || s.closing || !s.writer) throw Error('串口已断开');
    if (guard) guard();
    await s.writer.write(bytes);
  };
  s.queue = s.queue.then(action);
  return s.queue;
}
async function readLoop(s) {
  let phase = 'transport';
  try {
    while (!s.closing) {
      const {value, done} = await s.reader.read();
      if (done) break;
      phase = 'protocol';
      for (const data of s.parser.push(value)) handle(data);
      refresh();
      phase = 'transport';
    }
    if (!s.closing) throw Error('串口数据流已关闭');
  } catch (e) {
    if (!s.closing) {
      const message = `${phase === 'protocol' ? '串口数据解析失败' : '串口读取失败'}：${e.name || 'Error'} ${e.message || '未提供错误详情'}`;
      notice(message, 'error');
      log(message, true);
      // Close outside this read task so cleanup can await it without deadlock.
      setTimeout(() => disconnect(s), 0);
    }
  } finally { s.reader.releaseLock(); s.reader = null; }
}
async function disconnect(s = connection) {
  if (!s || s.closing || connection !== s) return;
  s.closing = true; clearInterval(s.heartbeat);
  if (pending) failPending('连接已断开，写入结果待核对');
  $('review').close();
  config = null; configAt = -Infinity; sampleAt = -Infinity; encoderAt = -Infinity;
  initialized = false; readRequested = false;
  refresh();
  // A stream error must not skip releasing the writer or closing the port.
  const cleanup = async (label, action) => {
    try { await action(); }
    catch (e) { log(`${label}：${e.name || 'Error'} ${e.message}`, true); }
  };
  await cleanup('接收流取消异常', async () => { if (s.reader) await s.reader.cancel(); });
  await cleanup('接收任务退出异常', async () => { if (s.readTask) await s.readTask; });
  if (s.writer) {
    const writer = s.writer;
    await cleanup('发送流取消异常', () => writer.abort());
    await cleanup('发送锁释放异常', () => writer.releaseLock());
    s.writer = null;
  }
  await cleanup('串口关闭异常', async () => { if (s.opened) await s.port.close(); });
  if (connection === s) connection = null;
  refresh();
}
async function connect() {
  if (connection) { await disconnect(); return; }
  if (!navigator.serial) { notice('此浏览器不支持串口连接，请使用小车 PC 上的 Chrome', 'error'); return; }
  const s = {port: null, writer: null, reader: null, queue: Promise.resolve(), parser: new P.Parser(), closing: false};
  connection = s; refresh();
  try {
    s.port = await navigator.serial.requestPort({filters: [{usbVendorId: 0x10C4, usbProductId: 0xEA60}]});
    await s.port.open({baudRate:115200, dataBits:8, stopBits:1, parity:'none', flowControl:'none'});
    s.opened = true; s.writer = s.port.writable.getWriter(); s.reader = s.port.readable.getReader();
    samples = []; frozen = []; latest = null; encoder = null; config = null;
    initialized = false; readRequested = false;
    notice('等待大 Yaw 参数与遥测');
    s.readTask = readLoop(s);
    const heartbeat = () => {
      if (s.closing || s.heartbeatBusy) return;
      s.heartbeatBusy = true;
      send(s, P.monitor()).catch(e => {
        notice(`串口发送失败：${e.message}`, 'error'); disconnect(s);
      }).finally(() => { s.heartbeatBusy = false; });
    };
    heartbeat(); s.heartbeat = setInterval(heartbeat, 200); refresh();
  } catch (e) {
    notice(e.name === 'NotFoundError' ? '未选择串口' : `连接失败：${e.message}`, 'error');
    await disconnect(s);
  }
}
function review(event) {
  event.preventDefault();
  try {
    if (!writable() || pending) throw Error(gate());
    reviewValues = draft();
    reviewBase = {...config};
    if (P.matches(config, reviewValues)) throw Error('编辑值与当前参数一致');
    $('diff').replaceChildren();
    for (const f of P.fields) {
      const tr = document.createElement('tr');
      [f.label, config[f.key], reviewValues[f.key]].forEach((value, i) => {
        const td = document.createElement('td'); td.textContent = value;
        if (i === 2 && config[f.key] !== value) td.className = 'changed';
        tr.append(td);
      });
      $('diff').append(tr);
    }
    $('confirmed').checked = false; refresh(); $('review').showModal();
  } catch (e) { notice(e.message, 'error'); }
}
async function apply() {
  if (!writable() || pending || !$('confirmed').checked || !reviewValues ||
      !reviewBase || config.fullPid !== reviewBase.fullPid || !P.matches(config, reviewBase)) return;
  if (reviewValues.effort_limit > config.effort_limit_max) return;
  if (P.matches(config, reviewValues)) { $('review').close(); return; }
  let id;
  do { id = crypto.getRandomValues(new Uint16Array(1))[0]; } while (!id || id === config.id || id === requestCounter);
  requestCounter = id;
  const values = {...reviewValues};
  const frames = P.tuneFrames(values, id, config.fullPid), s = connection;
  pending = {id, values, fullPid: config.fullPid, deadline: performance.now() + 4000};
  $('review').close(); $('confirmed').checked = false; refresh();
  notice(`正在写入大 Yaw 参数 #${id}`);
  try {
    for (const bytes of frames) {
      // Leave an IDLE gap for older firmware; new RX also handles coalesced bursts.
      await new Promise(resolve => setTimeout(resolve, 20));
      await send(s, bytes, () => {
      if (!writable() || !pending || pending.id !== id || performance.now() > pending.deadline)
        throw Error('发送前安全状态失效，未继续写入');
      if (config.fullPid !== pending.fullPid) throw Error('固件接口已变化，未继续写入');
      if (!P.matches(config, reviewBase)) throw Error('主控参数已变化，未继续写入');
      if (values.effort_limit > config.effort_limit_max) throw Error('固件输出范围已变化，未继续写入');
      });
    }
  } catch (e) {
    failPending(`写入未确认：${e.message}`);
    await disconnect(s);
  }
}
function drawCharts() {
  const records = paused ? frozen : samples;
  const end = paused && records.length ? records[records.length - 1].t : performance.now() / 1000;
  const points = records.filter(s => s.t >= end - 30);
  for (const [id, series, minimum] of [
    ['errorChart', [['error','#c94b48']], 1],
    ['speedChart', [['speed','#357dc0'], ['speedRef','#278a6e']], 1],
    ['effortChart', [['effort','#b27c26']], .1]]) {
    const canvas = $(id), rect = canvas.getBoundingClientRect(), dpr = devicePixelRatio || 1;
    const w = rect.width, h = rect.height;
    if (!w || !h) continue;
    if (canvas.width !== Math.round(w*dpr) || canvas.height !== Math.round(h*dpr)) {
      canvas.width = Math.round(w*dpr); canvas.height = Math.round(h*dpr);
    }
    const ctx = canvas.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.clearRect(0,0,w,h);
    const left = 51, right = w - 12, top = 17, bottom = h - 23;
    let peak = minimum;
    points.forEach(p => series.forEach(([key]) => { peak = Math.max(peak, Math.abs(p[key])); }));
    peak *= 1.1;
    ctx.font = '10px Segoe UI'; ctx.lineWidth = 1;
    for (let i=0;i<5;i++) {
      const y = top + (bottom-top)*i/4;
      ctx.strokeStyle = i===2 ? '#c0ccd2' : '#e9edef';
      ctx.beginPath(); ctx.moveTo(left,y); ctx.lineTo(right,y); ctx.stroke();
      ctx.fillStyle='#88979e'; ctx.textAlign='right';
      ctx.fillText(((1-i/2)*peak).toFixed(peak<10 ? 2 : 1),left-7,y+3);
    }
    for(let i=0;i<=3;i++) {
      const x=left+(right-left)*i/3;
      ctx.textAlign=i===3?'right':'left'; ctx.fillStyle='#8a989f';
      ctx.fillText(`${-30+i*10}s`,x,h-7);
    }
    for (const [key,color] of series) {
      ctx.strokeStyle=color; ctx.lineWidth=1.6; ctx.beginPath();
      let previous=null;
      for (const p of points) {
        const x=left+(p.t-(end-30))/30*(right-left), y=(top+bottom)/2-p[key]/peak*(bottom-top)/2;
        if (!previous || p.t-previous.t>.3) ctx.moveTo(x,y); else ctx.lineTo(x,y);
        previous=p;
      }
      ctx.stroke();
    }
    if (!points.length) {
      ctx.textAlign='center'; ctx.fillStyle='#93a2a8'; ctx.font='12px sans-serif';
      ctx.fillText('等待遥测', (left+right)/2,(top+bottom)/2-12);
    }
  }
}
$('connect').addEventListener('click', connect);
$('parameters').addEventListener('submit', review);
$('confirmed').addEventListener('change', refresh);
$('confirmApply').addEventListener('click', apply);
$('read').addEventListener('click', () => {
  if (!connection?.writer || pending) return;
  const token = ++readRequestToken;
  readRequested = true; refresh(); notice('等待新一帧主控参数回读');
  setTimeout(() => {
    if (readRequested && token === readRequestToken) { readRequested = false; requireRead = true; notice('参数回读超时', 'error'); refresh(); }
  }, 3000);
});
$('pause').addEventListener('click', () => {
  paused = !paused; frozen = paused ? samples.slice() : [];
  $('pause').innerHTML = `<i data-lucide="${paused?'play':'pause'}"></i>`;
  $('pause').title = paused ? '继续曲线' : '暂停曲线';
  $('pause').setAttribute('aria-label', $('pause').title); icons(); refresh();
});
$('clear').addEventListener('click', () => { samples = []; frozen = []; refresh(); });
$('export').addEventListener('click', () => {
  const lines = ['utc,time_s,error_deg,speed_ref_rpm,speed_rpm,effort,mode,active'];
  samples.forEach(s => lines.push([s.utc,s.t.toFixed(3),s.error,s.speedRef,s.speed,s.effort,s.mode,Number(s.active)].join(',')));
  const url = URL.createObjectURL(new Blob([lines.join('\r\n')], {type:'text/csv;charset=utf-8'}));
  const a=document.createElement('a'); a.href=url;
  a.download=`big-yaw-${new Date().toISOString().replace(/[:.]/g,'-')}.csv`; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
window.addEventListener('resize', drawCharts);
window.addEventListener('pagehide', () => { disconnect(); });
navigator.serial?.addEventListener('disconnect', () => { notice('串口设备已移除', 'error'); disconnect(); });
icons(); refresh(); setInterval(refresh, 100);
