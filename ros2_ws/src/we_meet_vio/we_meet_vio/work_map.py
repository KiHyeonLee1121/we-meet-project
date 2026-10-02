"""Small session-local task map, not a surveyed geographic map or SLAM backend."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Panel:
    id: int
    xyz: np.ndarray
    last_seen: float
    hits: int=1
    state: str="seen"


class WorkMap:
    def __init__(self, association_m=.65, max_entries=32):
        self.panels={}
        self.radius=association_m
        self.max_entries=max_entries
        self.selected=None
        self.serial=0
        self.last_frame=-1.

    def observe(self, stamp, candidates):
        if stamp <= self.last_frame:
            return []
        self.last_frame=stamp
        matched=set()
        observations=[]
        for xyz, ex, ey, score in sorted(candidates, key=lambda x:-x[3]):
            xyz=np.asarray(xyz,dtype=float)
            if not np.all(np.isfinite(xyz)) or not np.isfinite([ex,ey,score]).all():
                continue
            available=[p for p in self.panels.values() if p.id not in matched and np.linalg.norm(p.xyz[:2]-xyz[:2]) <= self.radius]
            panel=min(available,key=lambda p:np.linalg.norm(p.xyz[:2]-xyz[:2])) if available else None
            if panel is None:
                if len(self.panels)>=self.max_entries:
                    continue
                self.serial+=1
                panel=Panel(self.serial,xyz,stamp)
                self.panels[panel.id]=panel
            else:
                # Gaps break acquisition; do not accumulate unrelated sightings forever.
                panel.hits=panel.hits+1 if stamp-panel.last_seen <= .35 else 1
                panel.xyz=.7*panel.xyz+.3*xyz
                panel.last_seen=stamp
            matched.add(panel.id)
            observations.append((panel,ex,ey,score,xyz))
        return observations

    def select(self, panel_id):
        if self.selected is not None and self.selected != panel_id:
            raise ValueError("cannot switch target mid-flight")
        self.selected=panel_id
        self.panels[panel_id].state="selected"

    def complete(self):
        if self.selected is not None:
            self.panels[self.selected].state="held_5s"

    def dump(self):
        return {"frame":"map", "session_only":True, "selected":self.selected,
                "panels":[{"id":p.id,"xyz":p.xyz.tolist(),"last_seen_s":p.last_seen,
                           "hits":p.hits,"state":p.state} for p in self.panels.values()]}
