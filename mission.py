#!/usr/bin/env python3
"""WALL-E and EVE's Signal: autonomous grid-world search simulation."""
import argparse, json, math, random
from collections import deque
from pathlib import Path

DIRS = [(0,-1,"N"),(1,0,"E"),(0,1,"S"),(-1,0,"W")]
def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
def bearing(a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    if abs(dx)>=abs(dy): return "E" if dx>0 else "W" if dx<0 else "HERE"
    return "S" if dy>0 else "N" if dy<0 else "HERE"

class Environment:
    """Hidden ground truth is accessed only by the environment, not by the agent."""
    def __init__(self,cfg,seed=None):
        self.cfg=cfg; self.w=cfg["map"]["width"]; self.h=cfg["map"]["height"]
        self.start=tuple(cfg["map"]["start"]); self.pos=self.start
        self._eve=tuple(cfg["hidden_world"]["eve"])
        self._clues={tuple(c["position"]):c for c in cfg["hidden_world"]["clues"]}
        self.obstacles={tuple(p) for p in cfg["map"].get("obstacles",[])}
        self.chargers={tuple(p) for p in cfg["map"].get("chargers",[])}
        self.capacity=cfg["resources"]["battery_capacity"]; self.battery=self.capacity
        self.time_budget=cfg["resources"]["time_budget"]; self.elapsed=0; self.step_no=0
        self.noise=cfg.get("sensor_noise",0.0); self.rng=random.Random(seed if seed is not None else cfg.get("seed",7))
        self.visited={self.pos}; self.found_clues=[]; self.eve_found=False; self.scan_history=[]
    def inside(self,p): return 0<=p[0]<self.w and 0<=p[1]<self.h
    def observe(self):
        clues=[]
        for p,c in self._clues.items():
            if manhattan(self.pos,p)<=self.cfg["agent"].get("sensor_radius",1) and c["id"] not in self.found_clues:
                clues.append({"kind":"clue","id":c["id"],"bearing":bearing(self.pos,p),
                              "strength":round(max(.05,1/(1+manhattan(self.pos,p))+self.rng.uniform(-self.noise,self.noise)),3)})
        if manhattan(self.pos,self._eve)<=self.cfg["agent"].get("sensor_radius",1):
            self.eve_found=True
            return {"kind":"eve_signal_detected","position":list(self.pos),"clues":clues}
        return {"kind":"scan_result","position":list(self.pos),"signal_strength":
                round(max(0,1/(1+manhattan(self.pos,self._eve))+self.rng.uniform(-self.noise,self.noise)),3),"clues":clues}
    def step(self,action):
        self.step_no+=1; kind=action["type"]; obs={"kind":"no_observation"}
        if kind=="move":
            dx,dy=action["delta"]; dest=(self.pos[0]+dx,self.pos[1]+dy)
            if not self.inside(dest) or dest in self.obstacles: obs={"kind":"blocked_move","attempted":list(dest)}
            else:
                self.pos=dest; self.visited.add(dest); self.battery=max(0,self.battery-1); self.elapsed+=1; obs=self.observe()
        elif kind=="scan":
            if self.battery<2: obs={"kind":"low_battery"}
            else:
                self.battery-=2; self.elapsed+=1; obs=self.observe(); self.scan_history.append(obs)
                for c in obs.get("clues",[]):
                    if c["id"] not in self.found_clues:self.found_clues.append(c["id"])
        elif kind=="recharge":
            if self.pos in self.chargers:self.battery=self.capacity;obs={"kind":"recharged","battery":self.battery}
            else:obs={"kind":"no_charger_here"}
            self.elapsed+=1
        elif kind=="turn_back": self.elapsed+=1; self.battery=max(0,self.battery-1);obs={"kind":"turn_back_acknowledged"}
        elif kind=="wait": self.elapsed+=1;obs=self.observe()
        if self.elapsed>self.time_budget:obs={"kind":"time_budget_exceeded"}
        return obs

class Agent:
    def __init__(self,env):
        self.env=env; self.pos=env.pos; self.base=env.start; self.done=False; self.returning=False
        self.belief={(x,y):1.0 for y in range(env.h) for x in range(env.w) if (x,y) not in env.obstacles}
        self.evidence=[];self.trace=[];self.status="search_in_progress";self.confidence=0
        self._normalize()
    def _normalize(self):
        total=sum(max(0,v) for v in self.belief.values())
        if total:self.belief={p:max(0,v)/total for p,v in self.belief.items()}
    def update(self,obs):
        if obs["kind"]=="blocked_move":
            self.belief.pop(tuple(obs["attempted"]),None);self._normalize()
        elif obs["kind"] in ("scan_result","eve_signal_detected"):
            pos=tuple(obs["position"])
            if obs["kind"]=="eve_signal_detected":
                self.belief={p:(.92 if p==pos else .08/max(1,len(self.belief)-1)) for p in self.belief}
                self.evidence.append({"step":self.env.step_no,"type":"direct_signal","at":list(pos),"detail":"EVE signal detected"})
                self.returning=True
            else:
                s=obs["signal_strength"]
                for p in self.belief:
                    expected=1/(1+manhattan(pos,p));self.belief[p]*=max(.0001,math.exp(-((s-expected)**2)/.08))
            for c in obs.get("clues",[]):
                self.evidence.append({"step":self.env.step_no,"type":"clue","id":c["id"],"bearing":c["bearing"],"at":list(pos)})
                for p in self.belief:
                    if bearing(pos,p)==c["bearing"]:self.belief[p]*=2.4
            self._normalize()
        if self.belief:self.confidence=max(self.belief.values())
    def path(self,start,goal):
        start,goal=tuple(start),tuple(goal);q=deque([start]);prev={start:None}
        while q:
            p=q.popleft()
            if p==goal:break
            for dx,dy,_ in DIRS:
                n=(p[0]+dx,p[1]+dy)
                if self.env.inside(n) and n not in self.env.obstacles and n not in prev:prev[n]=p;q.append(n)
        if goal not in prev:return []
        out=[];p=goal
        while p is not None:out.append(p);p=prev[p]
        return out[::-1]
    def move_toward(self,target,reason):
        path=self.path(self.pos,target)
        if len(path)<2:return {"type":"scan","reason":"No path to target; scan for new evidence."}
        n=path[1];return {"type":"move","delta":[n[0]-self.pos[0],n[1]-self.pos[1]],"reason":reason}
    def choose(self):
        e=self.env
        if e.eve_found:self.returning=True
        if self.returning:
            if self.pos==self.base:
                self.done=True;self.status="eve_located_returned" if e.eve_found else "returned_without_confirmation"
                return {"type":"wait","reason":"At base; finish and report mission."}
            if self.pos in e.chargers and e.battery<e.capacity:return {"type":"recharge","reason":"Recharge before returning."}
            return self.move_toward(self.base,"Return to base after signal detection or resource warning.")
        if e.elapsed>=e.time_budget-3:
            self.returning=True;return {"type":"turn_back","reason":"Time budget nearly exhausted; return to base."}
        charger=min(e.chargers or {self.base},key=lambda p:manhattan(self.pos,p))
        if e.battery<=max(4,manhattan(self.pos,charger)+1):
            if self.pos in e.chargers:return {"type":"recharge","reason":"Battery reserve reached; recharge."}
            return self.move_toward(charger,"Battery reserve reached; navigate to nearest charger.")
        if e.battery>=2 and (not e.scan_history or e.step_no%3==0):
            return {"type":"scan","reason":"Sample signal and nearby clue bearings; update belief."}
        candidates=[p for p in self.belief if p not in e.visited]
        if not candidates:candidates=[p for p in self.belief if p!=self.pos]
        if not candidates:
            self.returning=True;return {"type":"turn_back","reason":"Search space exhausted; return to base."}
        target=max(candidates,key=lambda p:self.belief[p]/(1+manhattan(self.pos,p)))
        return self.move_toward(target,f"Explore promising unvisited cell {target}.")
    def run(self,max_steps=300):
        while not self.done and self.env.step_no<max_steps:
            action=self.choose();obs=self.env.step(action);self.pos=self.env.pos;self.update(obs)
            self.trace.append({"step":self.env.step_no,"action":action,"position":list(self.pos),"observation":obs,
                               "decision":action.get("reason",""),"battery":self.env.battery,"elapsed":self.env.elapsed})
            if obs["kind"]=="time_budget_exceeded" or self.env.battery<=0:self.returning=True
            if self.returning and self.pos==self.base:
                self.done=True;self.status="eve_located_returned" if self.env.eve_found else "returned_without_confirmation"
        if not self.done:self.status="step_limit_reached"
        best,prob=max(self.belief.items(),key=lambda item:item[1])
        return {"mission_status":self.status,"eve_confirmed":self.env.eve_found,"inferred_location":list(best),
          "confidence":round(prob,4),"last_position":list(self.pos),"steps":self.env.step_no,"elapsed_time_units":self.env.elapsed,
          "battery_remaining":self.env.battery,"clues_found":self.env.found_clues,"evidence":self.evidence,
          "visited_cells":len(self.env.visited),"recommended_next_step":
          "Verify the detected signal with a second scan and transmit coordinates." if self.env.eve_found else
          "Continue scanning high-probability reachable cells; review resource budget and sensor noise.",
          "trace":self.trace}

def write_outputs(report,env,out):
    out.mkdir(parents=True,exist_ok=True)
    (out/"trace.json").write_text(json.dumps(report["trace"],indent=2))
    (out/"report.json").write_text(json.dumps({k:v for k,v in report.items() if k!="trace"},indent=2))
    lines=["# Mission Report: WALL-E and EVE's Signal","",f"- **Status:** `{report['mission_status']}`",
      f"- **EVE confirmed:** {report['eve_confirmed']}",f"- **Inferred location:** `{tuple(report['inferred_location'])}`",
      f"- **Confidence:** {report['confidence']:.1%}",f"- **Steps:** {report['steps']}",
      f"- **Elapsed time units:** {report['elapsed_time_units']}",f"- **Battery remaining:** {report['battery_remaining']}",
      f"- **Clues found:** {', '.join(report['clues_found']) or 'None'}",f"- **Cells visited:** {report['visited_cells']}","","## Evidence",""]
    lines += [f"- Step {x['step']}: **{x['type']}** — {x.get('detail',x.get('id',''))}; {x.get('at','')} {x.get('bearing','')}" for x in report["evidence"]] or ["- No decisive clue recorded."]
    lines += ["","## Recommended next step","",report["recommended_next_step"],"","## Action trace","","| Step | Action | Position | Battery | Observation |","|---:|---|---|---:|---|"]
    lines += [f"| {t['step']} | {t['action']['type']} | {tuple(t['position'])} | {t['battery']} | {t['observation']['kind']} |" for t in report["trace"]]
    (out/"mission_report.md").write_text("\n".join(lines)+"\n")
    cells=[]
    route={tuple(t["position"]) for t in report["trace"]}
    for y in range(env.h):
        for x in range(env.w):
            p=(x,y);cls="cell"
            if p in env.obstacles:cls+=" wall"
            if p in env.chargers:cls+=" charger"
            if p==env.start:cls+=" base"
            if p==env._eve:cls+=" eve"
            if p in env._clues:cls+=" clue"
            if p in route:cls+=" visited"
            cells.append(f'<div class="{cls}" title="({x},{y})"></div>')
    html=f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Mission visualization</title><style>body{{font:16px system-ui;background:#f7eee8;color:#182033;padding:24px}}.map{{display:grid;grid-template-columns:repeat({env.w},minmax(12px,1fr));gap:3px;max-width:850px}}.cell{{aspect-ratio:1;border-radius:4px;background:#e8e0d9}}.wall{{background:#263247}}.visited{{box-shadow:inset 0 0 0 3px #5abfa0}}.base{{background:#7d8ff2}}.eve{{background:#ff6882}}.clue{{background:#ffc45e}}.charger{{background:#59c7d5}}.legend span{{margin-right:12px}}</style></head><body><h1>WALL-E and EVE's Signal</h1><p>Status: <b>{report['mission_status']}</b> · Steps: {report['steps']}</p><div class="map">{''.join(cells)}</div><p class="legend">Blue=base · Red=EVE · Yellow=clue · Cyan=charger · Green outline=visited · Dark=obstacle</p><p>Debug map displays ground truth for evaluation; the agent receives hidden locations only through interaction.</p></body></html>"""
    (out/"visualization.html").write_text(html)

def main():
    p=argparse.ArgumentParser();p.add_argument("--config",default="configs/sample_mission.json");p.add_argument("--out",default="outputs");p.add_argument("--seed",type=int)
    a=p.parse_args();cfg=json.loads(Path(a.config).read_text());env=Environment(cfg,a.seed);report=Agent(env).run()
    write_outputs(report,env,Path(a.out))
    print(json.dumps({k:v for k,v in report.items() if k not in ("trace","evidence")},indent=2))
    print("Artifacts written to",Path(a.out).resolve())
if __name__=="__main__":main()
