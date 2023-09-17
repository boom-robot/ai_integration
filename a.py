import redis
import time

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

while True:
    operation = r.get('operation')
    x = r.get('x')
    y = r.get('y')
    print(operation, x, y)
    time.sleep(0.08)