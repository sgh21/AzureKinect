import threading
import queue
import time

# 创建一个队列用于线程间通信
message_queue = queue.Queue()

def worker_thread():
    """子线程函数，轮询检查消息队列"""
    while True:
        try:
            # 阻塞等待消息队列中的消息
            message = message_queue.get()
            print(f"子线程收到消息: {message}")
        except queue.Empty:
            # 如果队列空，继续轮询
            continue

def main_thread():
    """主线程函数，发送消息到子线程"""
    for i in range(5):
        time.sleep(2)  # 模拟不定时发送消息
        message = f"消息 {i+1}"
        print(f"主线程发送消息: {message}")
        message_queue.put(message)

if __name__ == "__main__":
    # 创建并启动子线程
    thread = threading.Thread(target=worker_thread, daemon=True)
    thread.start()

    # 启动主线程的消息发送逻辑
    main_thread()

    # 等待子线程处理完所有消息
    thread.join(timeout=5)
    print("主线程结束")
