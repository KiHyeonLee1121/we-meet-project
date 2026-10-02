"""Bounded asynchronous JSONL: disk I/O never runs in the control callback."""
import json
import queue
import threading
from pathlib import Path


class Recorder:
    def __init__(self,directory,config):
        self.directory=Path(directory).expanduser()
        self.directory.mkdir(parents=True,exist_ok=False)
        (self.directory/"settings.json").write_text(json.dumps(config,indent=2))
        self.queue=queue.Queue(512)
        self.failed=""
        self.thread=threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def record(self,data):
        try:
            self.queue.put_nowait(data)
        except queue.Full:
            self.failed="flight log queue overflow"

    def _write(self):
        try:
            with (self.directory/"flight.jsonl").open("w",buffering=65536) as file:
                count=0
                while True:
                    data=self.queue.get()
                    if data is None:
                        break
                    file.write(json.dumps(data,separators=(",",":"))+"\n")
                    count+=1
                    if count%40==0:
                        file.flush()
        except (OSError,ValueError) as e:
            self.failed="flight log writer: "+str(e)

    def close(self):
        try:
            self.queue.put(None,timeout=.2)
            self.thread.join(timeout=1.)
        except queue.Full:
            self.failed="flight log close overflow"
