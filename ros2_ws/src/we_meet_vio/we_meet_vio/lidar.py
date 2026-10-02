"""TF-Luna binary protocol. Receipt timestamp is an approximation, not sensor clock."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    distance: float
    strength: int


class Parser:
    def __init__(self):
        self.buffer=bytearray()
        self.errors=0

    def feed(self,data):
        self.buffer.extend(data)
        frames=[]
        while len(self.buffer)>=9:
            offset=self.buffer.find(b'\x59\x59')
            if offset<0:
                self.buffer[:]=self.buffer[-1:] if self.buffer[-1:]==b'\x59' else b''
                break
            del self.buffer[:offset]
            if len(self.buffer)<9:
                break
            f=self.buffer[:9]
            if sum(f[:8]) & 255 != f[8]:
                del self.buffer[0]
                self.errors+=1
                continue
            frames.append(Frame((f[2]+256*f[3])/100.,f[4]+256*f[5]))
            del self.buffer[:9]
        return frames
