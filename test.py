from queue import PriorityQueue

a = PriorityQueue()
a.put((1, 2))
a.put((2, 3))
a.put((3, 4))
b = {3:'a', 4:'b', 5:'c'}

def func(a, b):
    del a[3]
    b.put((4,5))

func(b, a)
print(b.values())
print(a.queue)