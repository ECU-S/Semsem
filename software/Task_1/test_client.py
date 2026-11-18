import mock_robot
import socket, json
s = socket.create_connection(("127.0.0.1", 5000)); f = s.makefile("rw")
def call(**kw):
    f.write(json.dumps(kw) + "\n"); f.flush()
    return json.loads(f.readline())

print(call(id=1, cmd="scan"))
print(call(id=2, cmd="walk", n=10))
print(call(id=3, cmd="turn", deg=90))
print(call(id=4, cmd="walk", n=500))   # BAD_ARGS
print(call(id=5, cmd="nope"))          # UNKNOWN_CMD
print(call(id=6, cmd="truth"))