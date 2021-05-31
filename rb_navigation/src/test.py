import queue

class car:
    def __init__(self,ball):
        self.ball = ball
        self.color = "none"

c1 = car(12)
c2 = car(13)
q1 = queue.Queue()

print(q1.put(c1))

print(q1.put(c2))

print(q1.get())

print(q1.qsize())
print(q1.empty())
print(q1.get())
print(q1.empty())