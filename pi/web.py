#!/usr/bin/env python3
"""Dashboard theo doi AGV (Flask). Chay tren Pi:  python3 web.py   roi mo http://<ip-pi>:8080

Trang chinh: trang thai xe (cap nhat 1 s), anh camera, pin, chuyen chay, su co, quan ly hang hoa, nut DUNG KHAN.
API JSON: /api/status /api/summary /api/trips /api/events /api/battery /api/items (GET/POST/DELETE) /api/estop
          /api/route (bang ma, loai hang -> khu) /api/mission (GET trang thai, POST giao nhiem vu)
"""
import os
import subprocess
import time

import yaml
from flask import Flask, jsonify, request, send_file

from db import AgvDB
from route_map import RouteMap

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
CFG = yaml.safe_load(open('config.yaml'))
db = AgvDB(CFG['database'])
app = Flask(__name__)


@app.get('/api/status')
def api_status():
    return jsonify(db.status())


@app.get('/api/summary')
def api_summary():
    return jsonify(db.summary())


@app.get('/api/trips')
def api_trips():
    return jsonify(db.trips(int(request.args.get('limit', 50))))


@app.get('/api/events')
def api_events():
    return jsonify(db.events(int(request.args.get('limit', 100)), request.args.get('kind')))


@app.get('/api/battery')
def api_battery():
    return jsonify(db.battery(float(request.args.get('hours', 24))))


@app.get('/api/items')
def api_items():
    return jsonify(db.items())


@app.post('/api/items')
def api_items_add():
    d = request.get_json(force=True)
    if not d.get('code') or not d.get('name') or not d.get('warehouse'):
        return jsonify(error='can code, name, warehouse'), 400
    db.upsert_item(d['code'].strip(), d['name'].strip(), d['warehouse'].strip(), d.get('note', '').strip())
    return jsonify(ok=True)


@app.delete('/api/items/<code>')
def api_items_del(code):
    db.delete_item(code)
    return jsonify(ok=True)


@app.post('/api/estop')
def api_estop():
    db.push_command('estop')
    open('STOP', 'w').close()          # line_follow.py / cac script thu deu dung khi thay file nay
    db.event('estop', 'nhan nut DUNG KHAN tren dashboard')
    return jsonify(ok=True)


def _follow_running():
    out = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True).stdout
    return any(l.startswith('python3 -u line_follow.py') for l in out.splitlines())


@app.get('/api/route')
def api_route():
    try:
        rm = RouteMap('station_map.yaml')
    except ValueError as e:
        return jsonify(error=str(e)), 500
    cargo = [{'type': c, 'zone': z, 'plan': rm.plan(cargo_type=c)} for c, z in sorted(rm.cargo_routing.items())]
    return jsonify(stations=[{'id': s['id'], 'name': n, 'action': s['action']} for n, s in rm.stations.items()],
                   cargo=cargo)


@app.get('/api/mission')
def api_mission():
    tail = ''
    if os.path.exists('follow.log'):
        with open('follow.log', 'rb') as f:
            tail = f.read()[-1500:].decode('utf-8', 'replace')
    return jsonify(running=_follow_running(), log=tail.splitlines()[-8:])


@app.post('/api/mission')
def api_mission_start():
    """Giao nhiem vu: kiem tra moi dieu kien TRUOC, roi moi khoi dong line_follow.py (1 tien trinh duy nhat)."""
    d = request.get_json(force=True)
    pin = CFG.get('web', {}).get('control_pin')
    if pin and str(d.get('pin', '')) != str(pin):
        return jsonify(error='Sai ma PIN dieu khien'), 403
    if _follow_running():
        return jsonify(error='Xe dang chay nhiem vu khac - dung truoc (nut DUNG KHAN) roi giao lai'), 409
    try:
        rm = RouteMap('station_map.yaml')
    except ValueError as e:
        return jsonify(error=str(e)), 500
    cargo, dest, item = d.get('cargo'), d.get('dest') or None, (d.get('item') or '').strip() or None
    if item and cargo in (None, '') and not dest:
        it = db.item(item)
        if not it:
            return jsonify(error='Khong co ma hang %s' % item), 400
        dest = it['warehouse']
    try:
        plan = rm.plan(cargo_type=int(cargo) if cargo not in (None, '') else None, dest=dest,
                       pickup=not d.get('no_pickup'), return_home=bool(d.get('return_home')))
    except (ValueError, TypeError) as e:
        return jsonify(error='Nhiem vu khong hop le: %s' % e), 400
    acts = [rm.stations[n]['action'] for n in plan]
    cfg = yaml.safe_load(open('config.yaml'))                       # doc lai: co the vua sua lift.up_mm
    if any(a in ('LIFT_UP', 'LIFT_DOWN') for a in acts) and not (cfg.get('lift', {}).get('up_mm') or cfg.get('lift', {}).get('up_s')) \
            and not d.get('no_lift'):
        return jsonify(error='Chua dat lift.up_mm / lift.up_s trong config.yaml - khong the nang/ha'), 400
    final = plan[-2] if d.get('return_home') and len(plan) > 1 and plan[-1] == 'HOME' else plan[-1]
    cmd = ['python3', '-u', 'line_follow.py', '--max-s', str(int(d.get('max_s') or 600))]
    cmd += ['--cargo', str(int(cargo))] if cargo not in (None, '') else ['--dest', final]
    if d.get('no_pickup'):
        cmd.append('--no-pickup')
    if d.get('no_lift'):
        cmd.append('--no-lift')
    if d.get('return_home'):
        cmd.append('--return-home')
    if item:
        cmd += ['--item', item]
    if os.path.exists('STOP'):
        os.remove('STOP')
    log = open('follow.log', 'w')
    subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True, cwd=HERE)
    db.event('mission', 'giao tu dashboard: %s (%s)' % (' -> '.join(plan), ' '.join(cmd[3:])))
    return jsonify(ok=True, plan=plan, cmd=' '.join(cmd))


@app.get('/cam.jpg')
def cam():
    if not os.path.exists('cam.jpg'):
        return ('', 204)
    return send_file(os.path.join(HERE, 'cam.jpg'), mimetype='image/jpeg', max_age=0)


@app.get('/')
def index():
    return PAGE


PAGE = """<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AGV Dashboard</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--text:#1d2330;--muted:#6b7280;--line:#e5e7eb;--ok:#15803d;--warn:#b45309;--bad:#b91c1c;--acc:#2563eb}
@media (prefers-color-scheme:dark){:root{--bg:#111318;--card:#1b1e25;--text:#e8eaf0;--muted:#9aa3b2;--line:#2b2f3a;--ok:#4ade80;--warn:#fbbf24;--bad:#f87171;--acc:#60a5fa}}
*{box-sizing:border-box}body{margin:0;font:14px/1.45 system-ui,sans-serif;background:var(--bg);color:var(--text)}
header{display:flex;align-items:center;justify-content:space-between;padding:12px 16px;border-bottom:1px solid var(--line);background:var(--card)}
h1{font-size:18px;margin:0}main{display:grid;gap:14px;padding:14px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}.card h2{font-size:14px;margin:0 0 10px;color:var(--muted);font-weight:600}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px}.kv b{font-weight:600}.big{font-size:22px;font-weight:700}
.st-ok{color:var(--ok)}.st-warn{color:var(--warn)}.st-bad{color:var(--bad)}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:5px 6px;border-bottom:1px solid var(--line);text-align:left}th{color:var(--muted);font-weight:600}
.scroll{max-height:300px;overflow:auto}button{font:inherit;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--text);padding:6px 12px;cursor:pointer}
.estop{background:var(--bad);color:#fff;border:0;font-weight:700;padding:10px 18px}input{font:inherit;padding:5px 7px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--text);width:100%}
.row{display:grid;grid-template-columns:1fr 1fr 1fr auto;gap:6px;margin-top:8px}img{width:100%;border-radius:8px;background:#000;aspect-ratio:4/3;object-fit:contain}
.wide{grid-column:1/-1}svg{width:100%;height:160px}select{font:inherit;padding:5px 7px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--text)}
</style></head><body>
<header><h1>AGV Dashboard</h1><button class="estop" onclick="estop()">DỪNG KHẨN</button></header>
<main>
 <section class="card"><h2>Trạng thái xe</h2><div id="state" class="big">-</div><div class="kv" id="kv" style="margin-top:8px"></div></section>
 <section class="card"><h2>Camera dò line</h2><img id="cam" alt="camera"></section>
 <section class="card"><h2>Giao nhiệm vụ</h2>
  <div class="kv"><span>Loại hàng</span><select id="mc"><option value="">— chọn —</option></select>
   <span>hoặc mã hàng</span><select id="mi"><option value="">— không —</option></select>
   <span>Về HOME sau</span><input id="mh" type="checkbox" style="width:auto">
   <span>Bỏ qua PICKUP</span><input id="mp" type="checkbox" style="width:auto">
   <span>Không nâng (test lộ trình)</span><input id="ml" type="checkbox" style="width:auto"></div>
  <div id="mplan" style="margin:8px 0;color:var(--muted)">Lộ trình: -</div>
  <button id="mgo" onclick="startMission()" style="background:var(--acc);color:#fff;border:0;font-weight:700;padding:9px 16px">BẮT ĐẦU NHIỆM VỤ</button>
  <div id="mrun" style="margin-top:8px"></div>
  <pre id="mlog" style="font-size:11px;max-height:120px;overflow:auto;white-space:pre-wrap;color:var(--muted)"></pre></section>
 <section class="card"><h2>Thống kê</h2><div class="kv" id="sum"></div></section>
 <section class="card wide"><h2>Pin 24 giờ (V)</h2><svg id="bat" viewBox="0 0 600 160" preserveAspectRatio="none"></svg></section>
 <section class="card wide"><h2>Chuyến chạy</h2><div class="scroll"><table id="trips"></table></div></section>
 <section class="card wide"><h2>Sự cố / nhật ký</h2><div class="scroll"><table id="events"></table></div></section>
 <section class="card wide"><h2>Hàng hóa (mã QR → kho)</h2><table id="items"></table>
  <div class="row"><input id="ic" placeholder="Mã (QR)"><input id="in" placeholder="Tên hàng"><input id="iw" placeholder="Kho đích"><button onclick="addItem()">Thêm / sửa</button></div></section>
</main>
<script>
const $=id=>document.getElementById(id), fmt=t=>t?new Date(t*1000).toLocaleString('vi-VN'):'-';
const esc=s=>String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const cls=s=>/LOST|ESTOP|ERROR|failed|aborted/.test(s)?'st-bad':/STATION|TIMEOUT|STOPPED/.test(s)?'st-warn':'st-ok';
async function j(u,o){const r=await fetch(u,o);return r.json()}
async function status(){const s=await j('/api/status');const live=s.age_s!=null&&s.age_s<3;
 $('state').textContent=live?(s.state||'-'):'KHÔNG HOẠT ĐỘNG';$('state').className='big '+(live?cls(s.state||''):'st-warn');
 const kv=[['Chuyến',s.trip_id??'-'],['Hàng',s.item??'-'],['Đích',s.dst??'-'],['Thấy line',s.line_found?'có':'không'],
  ['Lệch line',s.line_error??'-'],['Quãng đường',(s.distance_m??0)+' m'],['Pin',s.battery_v!=null?s.battery_v.toFixed(2)+' V':'-'],
  ['Hướng',s.heading!=null?s.heading.toFixed(1)+'°':'-'],['Camera',(s.fps??'-')+' fps'],
  ['Lộ trình',s.plan&&s.plan.length?s.plan.join(' → '):'-'],['Trạm đích',s.target??'-'],['Bước',s.mission_step??'-'],
  ['Đang chở hàng',s.loaded?'CÓ':'không'],['Vị trí càng',s.lift_pos??'-'],['Cập nhật',fmt(s.ts)]];
 $('kv').innerHTML=kv.map(([k,v])=>'<span>'+k+'</span><b>'+esc(v)+'</b>').join('');
 if(live)$('cam').src='/cam.jpg?'+Date.now()}
async function slow(){const m=await j('/api/summary');
 $('sum').innerHTML=[['Tổng chuyến',m.trips_total],['Thành công',m.trips_ok],['Chuyến 24 h',m.trips_24h],['Sự cố 24 h',m.incidents_24h],
  ['Tổng quãng đường',m.distance_m.toFixed(1)+' m']].map(([k,v])=>'<span>'+k+'</span><b>'+v+'</b>').join('');
 const t=await j('/api/trips?limit=50');$('trips').innerHTML='<tr><th>#</th><th>Bắt đầu</th><th>Kết thúc</th><th>Hàng</th><th>Từ → Đến</th><th>Kết quả</th><th>m</th></tr>'+
  t.map(r=>'<tr><td>'+r.id+'</td><td>'+fmt(r.start_ts)+'</td><td>'+fmt(r.end_ts)+'</td><td>'+esc(r.item_code)+'</td><td>'+esc(r.src)+' → '+esc(r.dst)+
  '</td><td class="'+cls(r.result)+'">'+esc(r.result)+'</td><td>'+(r.distance_m||0).toFixed(2)+'</td></tr>').join('');
 const e=await j('/api/events?limit=100');$('events').innerHTML='<tr><th>Thời gian</th><th>Loại</th><th>Chi tiết</th><th>Chuyến</th></tr>'+
  e.map(r=>'<tr><td>'+fmt(r.ts)+'</td><td class="'+(/trip_|info|station/.test(r.kind)?'':'st-bad')+'">'+esc(r.kind)+'</td><td>'+esc(r.detail)+'</td><td>'+(r.trip_id??'')+'</td></tr>').join('');
 const it=await j('/api/items');$('items').innerHTML='<tr><th>Mã</th><th>Tên</th><th>Kho</th><th></th></tr>'+
  it.map(r=>'<tr><td>'+esc(r.code)+'</td><td>'+esc(r.name)+'</td><td>'+esc(r.warehouse)+'</td><td><button onclick="delItem(\\''+esc(r.code)+'\\')">Xóa</button></td></tr>').join('');
 const b=await j('/api/battery?hours=24');const svg=$('bat');
 if(b.length<2){svg.innerHTML='<text x="10" y="80" fill="currentColor">Chưa có dữ liệu pin</text>';return}
 const t0=b[0].ts,t1=b[b.length-1].ts,lo=11,hi=13.2,X=ts=>(ts-t0)/(t1-t0||1)*590+5,Y=v=>150-(Math.min(hi,Math.max(lo,v))-lo)/(hi-lo)*140;
 svg.innerHTML='<line x1="0" x2="600" y1="'+Y(11.5)+'" y2="'+Y(11.5)+'" stroke="currentColor" stroke-dasharray="4 4" opacity=".4"/>'+
  '<polyline fill="none" stroke="var(--acc)" stroke-width="2" points="'+b.map(p=>X(p.ts)+','+Y(p.volt)).join(' ')+'"/>'+
  '<text x="6" y="14" fill="currentColor" font-size="12">'+b[b.length-1].volt.toFixed(2)+' V ('+Math.round(b[b.length-1].percent)+'%)</text>'}
async function estop(){await j('/api/estop',{method:'POST'});status()}
let ROUTE=null;
async function loadRoute(){ROUTE=await j('/api/route');if(ROUTE.error){$('mplan').textContent=ROUTE.error;return}
 $('mc').innerHTML='<option value="">— chọn —</option>'+ROUTE.cargo.map(c=>'<option value="'+c.type+'">Loại '+c.type+' → '+esc(c.zone)+'</option>').join('');
 const it=await j('/api/items');$('mi').innerHTML='<option value="">— không —</option>'+it.map(r=>'<option value="'+esc(r.code)+'">'+esc(r.code)+' · '+esc(r.name)+' → '+esc(r.warehouse)+'</option>').join('');
 ['mc','mi','mh','mp'].forEach(i=>$(i).onchange=showPlan)}
function showPlan(){if(!ROUTE||ROUTE.error)return;let p=[];const c=$('mc').value;
 if(c){p=ROUTE.cargo.find(x=>x.type==c).plan.slice()}else if($('mi').value){const o=$('mi').selectedOptions[0].text;p=['PICKUP',o.split('→').pop().trim()]}
 if(p.length&&$('mp').checked)p=p.filter(x=>x!='PICKUP');if(p.length&&$('mh').checked&&p[p.length-1]!='HOME')p.push('HOME');
 $('mplan').textContent='Lộ trình: '+(p.length?p.join(' → '):'-')}
async function startMission(pin){const d={cargo:$('mc').value,item:$('mi').value,return_home:$('mh').checked,no_pickup:$('mp').checked,no_lift:$('ml').checked,pin:pin||''};
 if(!d.cargo&&!d.item){alert('Chọn loại hàng hoặc mã hàng');return}
 if(!pin&&!confirm('Bắt đầu nhiệm vụ?\\n'+$('mplan').textContent+'\\n\\nXe sẽ CHẠY. Kiểm tra đường đi trống, hàng sẵn sàng.'))return;
 const r=await j('/api/mission',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});
 if(r.error&&/PIN/.test(r.error)&&!pin){const p=prompt('Nhập mã PIN điều khiển');if(p)return startMission(p)}
 if(r.error)alert('KHÔNG BẮT ĐẦU: '+r.error);else{$('mrun').textContent='Đã giao: '+r.plan.join(' → ');mission()}}
async function mission(){const m=await j('/api/mission');$('mgo').disabled=m.running;$('mgo').style.opacity=m.running?.4:1;
 $('mrun').innerHTML=m.running?'<b class="st-ok">ĐANG CHẠY NHIỆM VỤ</b> — muốn dừng bấm DỪNG KHẨN':'<span class="st-warn">Xe rảnh</span>';
 $('mlog').textContent=(m.log||[]).join('\\n')}
async function addItem(){const d={code:$('ic').value,name:$('in').value,warehouse:$('iw').value};
 const r=await j('/api/items',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});
 if(r.error)alert(r.error);else{['ic','in','iw'].forEach(i=>$(i).value='');slow()}}
async function delItem(c){if(confirm('Xóa '+c+'?')){await j('/api/items/'+encodeURIComponent(c),{method:'DELETE'});slow()}}
status();slow();loadRoute();mission();setInterval(status,1000);setInterval(slow,5000);setInterval(mission,2000);
</script></body></html>"""


if __name__ == '__main__':
    app.run(host=CFG['web']['host'], port=CFG['web']['port'], threaded=True)
